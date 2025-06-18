#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Int8, Float32
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import DisplayTrajectory
from dobot_msgs_v3.srv import EnableRobot, ServoJ
from std_srvs.srv import Trigger

import trajectory_msgs


def deg(rad: float) -> float:
    return rad * 180.0 / 3.14159


def rad(deg_: float) -> float:
    return deg_ * 3.14159 / 180.0


class MoveCircleActionServer(Node):
    def __init__(self):
        super().__init__('action_move_server')

        # -------------------- STATE --------------------
        self._lock = threading.Lock()
        self.servo_controller_status = 0      # 0 = READY · 2 = BUSY · 1 = ERROR
        self.executing = False
        self.adc = []                         # cached trajectory (deg)
        self.goal_last_rad = None             # final point (rad)
        self.cache_timestamp = None
        self.jstate_window = deque(maxlen=10)  # for stability checks

        # -------------------- NEW “Mirror‐Back” STATE --------------------
        self._latest_joint = None         # most recent JointState[0:6] (rad)
        self._last_sent_joint = None      # last six‐joint list we actually forwarded
        self._change_threshold = 0.01     # rad tolerance per joint (~0.57°)

        # -------------------- PARAMS --------------------
        self.max_points = 50
        self.sleep_timing = 0.12
        # Watchdog disabled: no cache expiry enforced
        self.cache_expiry = None
        self.verify_tolerance_rad = rad(0.1)  # 0.1° each ⇒ 0.6° total
        self.epsilon_stable = 0.0001          # ≈ 0.006°
        self.settle_timeout = 2.0             # max time to wait for stability

        # -------------------- ROS INTERFACES --------------------
        robot_type = os.getenv("DOBOT_TYPE", "dobot")

        # 1) Action Server (original)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            f'{robot_type}_group_controller/follow_joint_trajectory',
            self.execute_callback,
        )

        # 2) DisplayTrajectory listener (caching incoming planned path)
        self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.listener_callback,
            10
        )

        # 3) JointState subscription for stability window (original)
        self.create_subscription(
            JointState,
            '/joint_states_robot',
            self.jstate_callback,
            qos_profile_sensor_data
        )

        # 4) Servo status topic & reset handling (original)
        self.status_pub = self.create_publisher(Int8, '/servo_controller_status', 10)
        self.create_subscription(
            Int8,
            '/servo_controller_status',
            self.status_callback,
            10
        )

        # 5) max_points param as topic & service (original)
        self.max_points_pub = self.create_publisher(Int8, '/max_points', 10)
        self.create_subscription(
            Int8,
            '/max_points',
            self.max_points_callback,
            10
        )
        self.create_service(Trigger, 'get_max_points', self.handle_get_max_points)

        # 6) sleep_timing param as topic & service (original)
        self.sleep_timing_pub = self.create_publisher(Float32, '/sleep_timing', 10)
        self.create_subscription(
            Float32,
            '/sleep_timing',
            self.sleep_timing_callback,
            10
        )
        self.create_service(Trigger, 'get_sleep_timing', self.handle_get_sleep_timing)

        # Publish initial states
        self.publish_status(0)         # READY
        self.publish_max_points()
        self.publish_sleep_timing()

        # 7) Clients for robot services (original)
        self.enable_cli = self.create_client(
            EnableRobot, '/dobot_bringup_v3/srv/EnableRobot'
        )
        self.servoj_cli = self.create_client(
            ServoJ, '/dobot_bringup_v3/srv/ServoJ'
        )
        while not self.enable_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for EnableRobot service…')

        # 8) Service for querying current servo status (original)
        self.create_service(Trigger, 'get_servo_status', self.handle_get_status)

        # ————————————————————————————————
        # 9) NEW: ActionClient for mirror‐back + JointState subscription
        self._traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'{robot_type}_group_controller/follow_joint_trajectory'
        )
        self._send_goal_future = None

        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states_robot',
            self._listener_callback,  # private callback for mirror‐back
            qos_profile_sensor_data
        )

        # 10) NEW: Timer to poll latest joint every 0.1 s
        self.create_timer(0.1, self._on_timer_tick)
        # ————————————————————————————————

        self.get_logger().info('Servo action node initialised.')

    # ---------- PUBLISH HELPERS (ORIGINAL) ----------
    def publish_status(self, code: int):
        self.status_pub.publish(Int8(data=code))
        with self._lock:
            self.servo_controller_status = code
        self.get_logger().info(f"Servo action node status → {code}")

    def publish_max_points(self):
        self.max_points_pub.publish(Int8(data=self.max_points))
        self.get_logger().info(f"Max points → {self.max_points}")

    def publish_sleep_timing(self):
        self.sleep_timing_pub.publish(Float32(data=self.sleep_timing))
        self.get_logger().info(f"Sleep timing → {self.sleep_timing}")

    def _reset_exec(self):
        with self._lock:
            self.executing = False
            self.adc.clear()
            self.goal_last_rad = None

    # ---------- TOPIC CALLBACKS (ORIGINAL) ----------
    def status_callback(self, msg: Int8):
        if msg.data == 0:
            with self._lock:
                if self.servo_controller_status != 0:
                    self.adc.clear()
                    self.goal_last_rad = None
                    self.executing = False
                    self.publish_status(0)
                    self.get_logger().info("DATALOG: external reset → READY")

    def max_points_callback(self, msg: Int8):
        with self._lock:
            if msg.data != self.max_points:
                self.max_points = msg.data
                self.publish_max_points()
                self.get_logger().info("DATALOG: external set max_points")

    def sleep_timing_callback(self, msg: Float32):
        with self._lock:
            if msg.data != self.sleep_timing:
                self.sleep_timing = msg.data
                self.publish_sleep_timing()
                self.get_logger().info("DATALOG: external set sleep_timing")
                # compute and log corresponding inter‐command delay
                min_t, max_t = 0.12, 0.3
                min_f, max_f = 0.6, 1.0
                if self.sleep_timing <= min_t:
                    f = min_f
                elif self.sleep_timing >= max_t:
                    f = max_f
                else:
                    f = min_f + (self.sleep_timing - min_t) / (max_t - min_t) * (max_f - min_f)
                inter_command_delay = self.sleep_timing * f
                self.get_logger().info(f"DATALOG: inter_command_delay → {inter_command_delay:.3f}s")

    def listener_callback(self, msg: DisplayTrajectory):
        """(Original) Cache incoming MoveIt! DisplayTrajectory as degrees."""
        with self._lock:
            ready_to_cache = (
                self.servo_controller_status == 0 or
                (self.servo_controller_status == 2 and not self.executing)
            )
        if not ready_to_cache:
            self.get_logger().info("DATALOG: Incoming traj ignored")
            return

        pts = msg.trajectory[0].joint_trajectory.points
        adc = [[deg(a) for a in p.positions] for p in pts]
        last_rad = list(pts[-1].positions)

        with self._lock:
            self.adc = adc
            self.goal_last_rad = last_rad
            self.cache_timestamp = self.get_clock().now().nanoseconds / 1e9
        self.publish_status(2)  # mark BUSY (2)
        self.get_logger().info(f"DATALOG: Cached traj ({len(adc)} pts)")

    def jstate_callback(self, msg: JointState):
        """(Original) Keep a sliding window of the last few JointState msgs (rad)."""
        if len(msg.position) >= 6:
            with self._lock:
                self.jstate_window.append(list(msg.position[:6]))

    # ---------- SERVICE HANDLERS (ORIGINAL) ----------
    def handle_get_status(self, request, response):
        with self._lock:
            response.success = True
            response.message = str(self.servo_controller_status)
        return response

    def handle_get_max_points(self, request, response):
        with self._lock:
            response.success = True
            response.message = str(self.max_points)
        return response

    def handle_get_sleep_timing(self, request, response):
        with self._lock:
            response.success = True
            response.message = str(self.sleep_timing)
        return response

    # ---------- ACTION EXECUTION (ORIGINAL) ----------
    def execute_callback(self, goal_handle):
        with self._lock:
            if (self.servo_controller_status != 2 or
                    not self.adc or
                    not self.goal_last_rad):
                self.get_logger().warn("DATALOG: No cached trajectory")
                goal_handle.succeed()
                res = FollowJointTrajectory.Result()
                res.error_code = 1
                return res

            adc = list(self.adc)
            goal_last_rad = list(self.goal_last_rad)
            max_pts = self.max_points
            sleep_t = self.sleep_timing
            self.executing = True

        self.get_logger().info(f"DATALOG: Executing ({len(adc)} pts)")

        if len(adc) > max_pts:
            self.get_logger().error("DATALOG: overflow")
            self.publish_status(1)
            goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = 3
            self._reset_exec()
            return res

        # send trajectory with interpolated inter‐command delay
        for idx, angles in enumerate(adc):
            req = ServoJ.Request()
            req.j1, req.j2, req.j3, req.j4, req.j5, req.j6 = (
                float(a) for a in angles
            )
            req.t = sleep_t
            self.servoj_cli.call_async(req)
            self.get_logger().info(f"DATALOG: Sent pt {idx}: {angles}")
            # compute factor and delay exactly as in the callback
            min_t, max_t = 0.12, 0.3
            min_f, max_f = 0.6, 1.0
            if sleep_t <= min_t:
                f = min_f
            elif sleep_t >= max_t:
                f = max_f
            else:
                f = min_f + (sleep_t - min_t) / (max_t - min_t) * (max_f - min_f)
            inter_command_delay = sleep_t * f
            time.sleep(inter_command_delay)

        # sliding stability triplet
        time.sleep(0.05)
        start = time.time()
        samples = deque(maxlen=3)
        for _ in range(3):
            with self._lock:
                if self.jstate_window:
                    samples.append(list(self.jstate_window[-1]))
            time.sleep(0.02)

        stable = False
        while time.time() - start < self.settle_timeout:
            diffs_ab = [abs(samples[0][j] - samples[1][j]) for j in range(6)]
            diffs_bc = [abs(samples[1][j] - samples[2][j]) for j in range(6)]
            if (all(d < self.epsilon_stable for d in diffs_ab) and
                    all(d < self.epsilon_stable for d in diffs_bc)):
                stable = True
                break
            with self._lock:
                if self.jstate_window:
                    samples.append(list(self.jstate_window[-1]))
            time.sleep(0.02)

        self.get_logger().info(f"DATALOG: Stability result → {'stable' if stable else 'not stable'}")

        success = False
        if stable and len(samples) == 3:
            latest = samples[-1]
            diffs_rad = [abs(latest[i] - goal_last_rad[i]) for i in range(6)]
            sum_rad = sum(diffs_rad)
            self.get_logger().info(
                f"DATALOG: Σ|Δ| = {deg(sum_rad):.3f}°"
                f"  (threshold {deg(self.verify_tolerance_rad*6):.2f}°)"
            )
            if sum_rad <= self.verify_tolerance_rad * 6:
                success = True
            self.get_logger().info(f"DATALOG: actual (°): {[deg(v) for v in latest]}")
            self.get_logger().info(f"DATALOG: goal   (°): {[deg(v) for v in goal_last_rad]}")

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        if success:
            result.error_code = 0
            self.publish_status(0)
            self.get_logger().info("DATALOG: verification OK")
        else:
            result.error_code = 2
            self.publish_status(1)
            self.get_logger().error("DATALOG: verification FAILED")

        self._reset_exec()
        return result

    # ---------- NEW: JointState “Mirror‐Back” CALLBACK ----------
    def _listener_callback(self, msg: JointState):
        """
        Runs every time a JointState arrives (~8 ms). We only store the
        six‐joint array here if servo_controller_status == 0; the actual
        “send goal” happens on a 0.1s timer instead.
        """
        with self._lock:
            if self.servo_controller_status != 0:
                return

        if len(msg.position) >= 6:
            # Store the raw six‐joint array (radians). Rounding happens later.
            with self._lock:
                self._latest_joint = list(msg.position[:6])

    # ---------- NEW: TIMER CALLBACK (0.1 s) ----------
    def _on_timer_tick(self):
        """
        Fires every 0.1 s. If status == 0 and we have a latest_joint, compare
        it to last_sent_joint. If any joint moved > threshold, round to 3 decimals
        and send a new one‐point trajectory goal. Otherwise, skip.
        """
        with self._lock:
            # 1) Only mirror‐back when READY
            if self.servo_controller_status != 0:
                return

            # 2) Must have received at least one JointState
            if self._latest_joint is None:
                return

            current = self._latest_joint
            prev = self._last_sent_joint

            # 3) Decide whether to send
            if prev is None:
                should_send = True
            else:
                diffs = [abs(current[i] - prev[i]) for i in range(6)]
                should_send = any(diff > self._change_threshold for diff in diffs)

            if not should_send:
                return

            # 4) Round each to 3 decimals (matching original SubscriberNode logic)
            to_send = [round(x, 3) for x in current]
            self._last_sent_joint = list(to_send)

        # 5) Actually send outside the lock
        self._send_goal(to_send)

    # ---------- NEW: BUILD & SEND A SINGLE‐POINT GOAL ----------
    def _send_goal(self, joint_list: list[float]):
        """
        Create a one‐point FollowJointTrajectory goal with joint_list (six floats),
        wait for the action server, then send it asynchronously.
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
        ]
        point = trajectory_msgs.msg.JointTrajectoryPoint(
            positions=[*joint_list]
            # time_from_start left at zero (controller must accept zero‐time)
        )
        goal_msg.trajectory.points.append(point)

        self._traj_client.wait_for_server()
        self._send_goal_future = self._traj_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )

    # ---------- NEW: ACTION CLIENT FEEDBACK (mirror‐back) ----------
    def _feedback_callback(self, feedback_msg):
        """
        Feedback from the FollowJointTrajectory ActionServer. We simply log it.
        """
        self.get_logger().debug("Mirror‐back: action feedback received")

    # -----------------------------------------------------------------
    # The rest of your node (existing code) follows unchanged.
    # -----------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = MoveCircleActionServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
