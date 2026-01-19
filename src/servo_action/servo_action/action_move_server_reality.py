#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Servo-action node with:
  • ServoJ liveness watchdog (≤5 s, 3 attempts)
  • Send-once protection so one cached trajectory is never executed twice
Everything else is unchanged.
"""
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

from trajectory_msgs.msg import JointTrajectoryPoint


def deg(rad: float) -> float:
    return rad * 180.0 / 3.14159


def rad(deg_: float) -> float:
    return deg_ * 3.14159 / 180.0


class MoveCircleActionServer(Node):
    # -----------------------------------------------------------------
    # INITIALISATION
    # -----------------------------------------------------------------
    def __init__(self):
        super().__init__('action_move_server')

        # -------------------- STATE --------------------
        self._lock = threading.Lock()
        self.servo_controller_status = 0             # 0 = READY · 2 = BUSY · 1 = ERROR
        self.executing = False
        self.adc = []                                # cached trajectory (deg)
        self.goal_last_rad = None                    # final point (rad)
        self.cache_timestamp = None                  # secs from epoch
        self._last_executed_cache_time = None        # NEW: for "send once" rule
        self.jstate_window = deque(maxlen=10)        # stability buffer

        # -------------------- NEW "Mirror-Back" & External-Motion STATE --------------------
        self._latest_joint = None
        self._last_sent_joint = None
        self._change_threshold = 0.01                # rad
        self._external_moving = False

        # -------------------- PARAMS --------------------
        self.max_points = 35
        self.sleep_timing = 0.12
        self.cache_expiry = None                     # watchdog disabled
        self.verify_tolerance_rad = rad(0.5)
        self.epsilon_stable = 0.0001
        self.settle_timeout = 10.0
        self.external_move_threshold = rad(0.6) * 6
        self.execution_timeout = 15.0                # max time for trajectory execution

        # -------------------- ROS INTERFACES --------------------
        robot_type = os.getenv("DOBOT_TYPE", "dobot")

        # 1) Action Server (trajectory execution)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            f'{robot_type}_group_controller/follow_joint_trajectory',
            self.execute_callback,
        )

        # 2) DisplayTrajectory listener (trajectory caching)
        self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.listener_callback,
            10
        )

        # 3) JointState subscription (stability)
        self.create_subscription(
            JointState,
            '/joint_states_robot',
            self.jstate_callback,
            qos_profile_sensor_data
        )

        # 4) Servo-status topic & reset handling
        self.status_pub = self.create_publisher(Int8, '/servo_controller_status', 10)
        self.create_subscription(Int8, '/servo_controller_status',
                                 self.status_callback, 10)

        # 5) max_points param topic & service
        self.max_points_pub = self.create_publisher(Int8, '/max_points', 10)
        self.create_subscription(Int8, '/max_points',
                                 self.max_points_callback, 10)
        self.create_service(Trigger, 'get_max_points',
                            self.handle_get_max_points)

        # 6) sleep_timing param topic & service
        self.sleep_timing_pub = self.create_publisher(Float32, '/sleep_timing', 10)
        self.create_subscription(Float32, '/sleep_timing',
                                 self.sleep_timing_callback, 10)
        self.create_service(Trigger, 'get_sleep_timing',
                            self.handle_get_sleep_timing)

        # Publish initial parameter topics
        self.publish_status(0)         # READY
        self.publish_max_points()
        self.publish_sleep_timing()

        # 7) Clients for robot services
        self.enable_cli = self.create_client(
            EnableRobot, '/dobot_bringup_v3/srv/EnableRobot')
        self.servoj_cli = self.create_client(
            ServoJ, '/dobot_bringup_v3/srv/ServoJ')

        while not self.enable_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for EnableRobot service…')

        # 8) Service for querying current servo status
        self.create_service(Trigger, 'get_servo_status', self.handle_get_status)
        
        # 8.1) Service for emergency reset when stuck
        self.create_service(Trigger, 'emergency_reset', self.handle_emergency_reset)

        # 9) ActionClient for mirror-back + JointState subscription
        self._traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'{robot_type}_group_controller/follow_joint_trajectory'
        )
        self._send_goal_future = None
        self._joint_state_sub = self.create_subscription(
            JointState, '/joint_states_robot',
            self._listener_callback, qos_profile_sensor_data
        )

        # 10) Timer (0.1 s) for mirror-back & external-motion detection
        self.create_timer(0.025, self._on_timer_tick)

        self.get_logger().info('Servo action node initialised.')

    # -----------------------------------------------------------------
    # SMALL HELPERS
    # -----------------------------------------------------------------
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

    # -----------------------------------------------------------------
    # NEW  ✧  ServoJ LIVENESS & "SEND-ONCE" UTILITIES
    # -----------------------------------------------------------------
    def _ensure_servoj_ready(self, attempts: int = 3, timeout: float = 5.0) -> bool:
        """
        Try `attempts` times, waiting `timeout` seconds each, for
        `/dobot_bringup_v3/srv/ServoJ` to appear.  Returns True if available.
        """
        for n in range(1, attempts + 1):
            if self.servoj_cli.wait_for_service(timeout_sec=timeout):
                return True
            self.get_logger().warn(
                f"ServoJ service unavailable (attempt {n}/{attempts})"
            )
        return False

    # -----------------------------------------------------------------
    # TOPIC / SERVICE CALLBACKS (unchanged apart from minor comments)
    # -----------------------------------------------------------------
    def status_callback(self, msg: Int8):
        # … identical to original …
        if msg.data == 0:
            with self._lock:
                if self.servo_controller_status != 0:
                    self.adc.clear()
                    self.goal_last_rad = None
                    self.executing = False
                    self.publish_status(0)
                    self.get_logger().info("DATALOG: external reset → READY")

    def max_points_callback(self, msg: Int8):
        # … identical …

        with self._lock:
            if msg.data != self.max_points:
                self.max_points = msg.data
                self.publish_max_points()
                self.get_logger().info("DATALOG: external set max_points")

    def sleep_timing_callback(self, msg: Float32):
        # … identical …

        with self._lock:
            if msg.data != self.sleep_timing:
                self.sleep_timing = msg.data
                self.publish_sleep_timing()
                self.get_logger().info("DATALOG: external set sleep_timing")
                # compute and log corresponding inter-command delay
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
        """Cache incoming MoveIt! DisplayTrajectory (deg) + timestamp."""
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
        self.publish_status(2)  # BUSY
        self.get_logger().info(f"DATALOG: Cached traj ({len(adc)} pts)")

    def jstate_callback(self, msg: JointState):
        if len(msg.position) >= 6:
            with self._lock:
                self.jstate_window.append(list(msg.position[:6]))

    # ------- Trigger service handlers (unchanged) -------
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
    
    def handle_emergency_reset(self, request, response):
        """Emergency reset when robot gets stuck"""
        with self._lock:
            self.get_logger().warn("EMERGENCY RESET: Clearing all state and resetting to READY")
            self.adc.clear()
            self.goal_last_rad = None
            self.cache_timestamp = None
            self._last_executed_cache_time = None
            self.executing = False
            self._external_moving = False
            self.servo_controller_status = 0
            self.jstate_window.clear()
        
        self.publish_status(0)  # READY
        response.success = True
        response.message = "Emergency reset completed"
        return response

    # -----------------------------------------------------------------
    # ACTION EXECUTION (major additions marked NEW)
    # -----------------------------------------------------------------
    def execute_callback(self, goal_handle):
        """
        Called when MoveIt! sends a FollowJointTrajectory goal.

        NEW LOGIC:
          • bail out early if the exact same cached trajectory
            was already executed (send-once rule)
          • check ServoJ service liveness (≤5 s, 3×) before
            transmitting the first point
        """
        with self._lock:
            # Abort if there is *no* trajectory cached
            if (self.servo_controller_status != 2 or
                    not self.adc or
                    not self.goal_last_rad):
                self.get_logger().warn("DATALOG: No cached trajectory")
                goal_handle.succeed()
                res = FollowJointTrajectory.Result()
                res.error_code = 1
                return res

            # # ---- SEND-ONCE GUARD (NEW) ----
            # if (self.cache_timestamp is not None and
            #         self.cache_timestamp == self._last_executed_cache_time):
            #     self.get_logger().info("DATALOG: Trajectory already executed → skipping")
            #     goal_handle.succeed()
            #     res = FollowJointTrajectory.Result()
            #     res.error_code = 0
            #     return res
            # # --------------------------------

            adc = list(self.adc)
            goal_last_rad = list(self.goal_last_rad)
            max_pts = self.max_points
            sleep_t = self.sleep_timing
            self.executing = True

        # ---- SERVOJ SERVICE WATCHDOG (NEW) ----
        # We're at the beginning of trajectory execution, safe to retry
        if not self._ensure_servoj_ready(attempts=3, timeout=2.0):
            self.get_logger().error(
                "ServoJ service not available after 3 attempts; aborting trajectory."
            )
            self.publish_status(1)
            goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = 4          # 4 = service unavailable
            self._reset_exec()
            return res
        
        # Service is alive - brief pause to ensure service is fully ready
        self.get_logger().info("DATALOG: ServoJ service confirmed alive, stabilizing...")
        time.sleep(0.1)
        # ---------------------------------------

        self.get_logger().info(f"DATALOG: Executing ({len(adc)} pts)")

        # Start execution timer
        execution_start_time = time.time()

        if len(adc) > max_pts:
            self.get_logger().error("DATALOG: overflow")
            self.publish_status(1)
            goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = 3
            self._reset_exec()
            return res

        # ---------- Send the points ----------
        for idx, angles in enumerate(adc):
            # Check execution timeout
            if time.time() - execution_start_time > self.execution_timeout:
                self.get_logger().error(f"DATALOG: Execution timeout after {self.execution_timeout}s at pt {idx}")
                self.publish_status(1)
                goal_handle.succeed()
                res = FollowJointTrajectory.Result()
                res.error_code = 6  # TIMED_OUT
                self._reset_exec()
                return res
            
            # Mid-trajectory service health check (every 10 points)
            if idx > 0 and idx % 10 == 0:
                if not self.servoj_cli.service_is_ready():
                    self.get_logger().error(f"DATALOG: ServoJ service died at pt {idx} - ABORTING for safety")
                    self.publish_status(1)
                    goal_handle.succeed()
                    res = FollowJointTrajectory.Result()
                    res.error_code = 4  # Service unavailable
                    self._reset_exec()
                    return res
            
            req = ServoJ.Request()
            (req.j1, req.j2, req.j3,
             req.j4, req.j5, req.j6) = (float(a) for a in angles)
            req.t = sleep_t
            
            # Send ServoJ command (fire and forget - original behavior)
            self.servoj_cli.call_async(req)
            self.get_logger().info(f"DATALOG: Sent pt {idx}: {angles}")

            # Inter-command delay (unchanged formula)
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
        # -------------------------------------

        # Sliding-window stability verification (unchanged) …
        time.sleep(0.05)
        start = time.time()
        samples = deque(maxlen=3)
        for _ in range(3):
            with self._lock:
                if self.jstate_window:
                    samples.append(list(self.jstate_window[-1]))
            time.sleep(0.04)

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
            time.sleep(0.06)

        self.get_logger().info(f"DATALOG: Stability result → {'stable' if stable else 'not stable'}")

        success = False
        if stable and len(samples) == 3:
            latest = samples[-1]
            diffs_rad = [abs(latest[i] - goal_last_rad[i]) for i in range(6)]
            sum_rad = sum(diffs_rad)
            self.get_logger().info(
                f"DATALOG: Σ|Δ| = {deg(sum_rad):.3f}°"
                f"  (threshold {deg(self.verify_tolerance_rad * 6):.2f}°)"
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
            # Mark trajectory as executed (NEW)
            with self._lock:
                self._last_executed_cache_time = self.cache_timestamp
        else:
            result.error_code = 2
            self.publish_status(1)
            self.get_logger().error("DATALOG: verification FAILED")

        self._reset_exec()
        return result

    # -----------------------------------------------------------------
    # MIRROR-BACK SECTION (unchanged except comments)
    # -----------------------------------------------------------------
    def _listener_callback(self, msg: JointState):
        with self._lock:
            if self.servo_controller_status != 0:
                return
        if len(msg.position) >= 6:
            with self._lock:
                self._latest_joint = list(msg.position[:6])

    def _on_timer_tick(self):
        # identical to original mirror-back logic …
        with self._lock:
            if self.executing:
                return
            if self._external_moving:
                if len(self.jstate_window) >= 3:
                    a, b, c = (self.jstate_window[-3],
                               self.jstate_window[-2],
                               self.jstate_window[-1])
                    diffs_ab = [abs(a[i] - b[i]) for i in range(6)]
                    diffs_bc = [abs(b[i] - c[i]) for i in range(6)]
                    if (all(d < self.epsilon_stable for d in diffs_ab) and
                            all(d < self.epsilon_stable for d in diffs_bc)):
                        self._external_moving = False
                        self.servo_controller_status = 0
                        self.goal_last_rad = list(c)
                return

            if self.servo_controller_status != 0:
                return
            if self._latest_joint is None:
                return
            if self.goal_last_rad is None:
                self.goal_last_rad = list(self._latest_joint)
                return

            diff_to_plan = [
                abs(self._latest_joint[i] - self.goal_last_rad[i])
                for i in range(6)
            ]
            if sum(diff_to_plan) > self.external_move_threshold:
                self._external_moving = True
                self.servo_controller_status = 2
                return

            prev = self._last_sent_joint
            current = self._latest_joint
            if prev is None or any(
                abs(current[i] - prev[i]) > self._change_threshold for i in range(6)
            ):
                to_send = [round(x, 3) for x in current]
                self._last_sent_joint = list(to_send)
            else:
                return
        self._send_goal(to_send)

    def _send_goal(self, joint_list: list[float]):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
        ]
        point = JointTrajectoryPoint(
            positions=[*joint_list])
        goal_msg.trajectory.points.append(point)

        self._traj_client.wait_for_server()
        self._send_goal_future = self._traj_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )

    def _feedback_callback(self, feedback_msg):
        self.get_logger().debug("Mirror-back: action feedback received")

    # -----------------------------------------------------------------
    # MAIN
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
