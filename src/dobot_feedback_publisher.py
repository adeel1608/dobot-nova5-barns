#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Dobot Nova (V3) feedback publisher.

This node connects to the Dobot feedback port (30005 by default) and publishes:
- /dobot/tool_vector_actual              (dobot_msgs_v3/msg/ToolVectorActual)
- /dobot/joint_states_feedback           (sensor_msgs/msg/JointState) with effort if available
- /dobot/tcp_wrench                      (geometry_msgs/msg/WrenchStamped) from actual_TCP_force
- /dobot/sixaxis_wrench                  (geometry_msgs/msg/WrenchStamped) from six_force_value (if online)
- /dobot/six_force_online                (std_msgs/msg/Bool)
- /dobot/load                            (std_msgs/msg/Float64)

Notes:
- If your robot has NO F/T sensor and firmware doesn't provide estimated wrench/torque,
  the force/effort fields may be zeros or not meaningful.
- This does NOT magically "measure weight"; it only exposes whatever the controller reports.

Env vars:
- IP_address: robot IP (required)
- FEEDBACK_PORT: feedback port, default 30005
"""

import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from dobot_msgs_v3.msg import ToolVectorActual

# Import the patched feedback parser
from .feedback_updated import fankuis


def _now(node: Node):
    return node.get_clock().now().to_msg()


class DobotFeedbackPublisher(Node):
    def __init__(self):
        super().__init__("dobot_feedback_publisher")

        ip = str(os.getenv("IP_address") or "")
        if not ip:
            raise RuntimeError("IP_address env var is not set. Example: export IP_address=192.168.5.1")

        port = int(os.getenv("FEEDBACK_PORT") or "30005")

        self.get_logger().info(f"Connecting feedback to {ip}:{port}")
        self.fb = fankuis(ip, port)

        # Publishers
        self.pub_tool = self.create_publisher(ToolVectorActual, "/dobot/tool_vector_actual", 10)
        self.pub_js = self.create_publisher(JointState, "/dobot/joint_states_feedback", 10)
        self.pub_tcp_wrench = self.create_publisher(WrenchStamped, "/dobot/tcp_wrench", 10)
        self.pub_six_wrench = self.create_publisher(WrenchStamped, "/dobot/sixaxis_wrench", 10)
        self.pub_six_online = self.create_publisher(Bool, "/dobot/six_force_online", 10)
        self.pub_load = self.create_publisher(Float64, "/dobot/load", 10)

        # Joint naming order used by your current /joint_states output
        self.joint_names = ["joint2", "joint3", "joint1", "joint4", "joint5", "joint6"]

        # Timer (50 Hz)
        self.timer = self.create_timer(0.02, self._tick)

    def _tick(self):
        data = self.fb.feed()
        if not data or data[0] == "NG":
            return

        tool_v = data[0]     # (6,)
        q_target = data[1]   # (6,)
        q_actual = data[2] if len(data) > 2 else None
        qd_actual = data[3] if len(data) > 3 else None
        m_actual = data[4] if len(data) > 4 else None
        tcp_force = data[5] if len(data) > 5 else None
        six_online = data[6] if len(data) > 6 else 0
        six_force = data[7] if len(data) > 7 else None
        load = data[8] if len(data) > 8 else 0.0

        # Publish ToolVectorActual (pose)
        tv = ToolVectorActual()
        tv.x, tv.y, tv.z, tv.rx, tv.ry, tv.rz = [float(x) for x in tool_v]
        self.pub_tool.publish(tv)

        # Publish JointState feedback (use q_actual if present, else q_target)
        js = JointState()
        js.header.stamp = _now(self)

        # Reorder from controller order (joint1..6) to your ROS order (joint2, joint3, joint1, joint4, joint5, joint6)
        # Controller arrays are assumed joint1..6
        if q_actual is None:
            q = q_target
        else:
            q = q_actual

        def reorder(arr6):
            j1, j2, j3, j4, j5, j6 = [float(v) for v in arr6]
            return [j2, j3, j1, j4, j5, j6]

        js.name = self.joint_names
        js.position = reorder(q)

        if qd_actual is not None:
            js.velocity = reorder(qd_actual)
        else:
            js.velocity = [0.0] * 6

        # Effort: publish m_actual (torque estimate) if finite numbers, else NaNs
        if m_actual is not None:
            eff = reorder(m_actual)
            # If any value is NaN/inf, keep as NaN to signal "not provided"
            import math
            js.effort = [v if (math.isfinite(v)) else float("nan") for v in eff]
        else:
            js.effort = [float("nan")] * 6

        self.pub_js.publish(js)

        # TCP wrench (actual_TCP_force)
        if tcp_force is not None:
            w = WrenchStamped()
            w.header.stamp = _now(self)
            w.header.frame_id = "tool0"
            w.wrench.force.x = float(tcp_force[0])
            w.wrench.force.y = float(tcp_force[1])
            w.wrench.force.z = float(tcp_force[2])
            w.wrench.torque.x = float(tcp_force[3])
            w.wrench.torque.y = float(tcp_force[4])
            w.wrench.torque.z = float(tcp_force[5])
            self.pub_tcp_wrench.publish(w)

        # 6-axis FT wrench (six_force_value) if online
        online_msg = Bool()
        online_msg.data = bool(int(six_online) == 1)
        self.pub_six_online.publish(online_msg)

        if six_force is not None and int(six_online) == 1:
            w2 = WrenchStamped()
            w2.header.stamp = _now(self)
            w2.header.frame_id = "tool0"
            w2.wrench.force.x = float(six_force[0])
            w2.wrench.force.y = float(six_force[1])
            w2.wrench.force.z = float(six_force[2])
            w2.wrench.torque.x = float(six_force[3])
            w2.wrench.torque.y = float(six_force[4])
            w2.wrench.torque.z = float(six_force[5])
            self.pub_six_wrench.publish(w2)

        # Load estimate (controller-reported)
        load_msg = Float64()
        load_msg.data = float(load)
        self.pub_load.publish(load_msg)


def main():
    rclpy.init()
    node = DobotFeedbackPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
