import math
import time
import os
import signal
import sys
from threading import Thread

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from tf2_ros import Buffer, TransformListener, LookupException
import tf_transformations

from pymoveit2 import MoveIt2, MoveIt2State

from std_srvs.srv import Trigger  # For the capture_point service


class DirectTfWaypointsExample(Node):
    def __init__(self):
        super().__init__("ex_direct_tf_waypoints_goal")

        # ───────────────────────────── Parameters ──────────────────────────────
        self.declare_parameter("target_frame", "calibration_pose")
        self.declare_parameter("reference_frame", "base_link")
        self.declare_parameter("offset_reference_frame", "base_link")
        self.declare_parameter("calibration_tag", "calibration_tag")
        self.declare_parameter("planner_id", "OMPL")
        self.declare_parameter("cartesian", True)
        self.declare_parameter("cartesian_max_step", 0.001)
        self.declare_parameter("cartesian_fraction_threshold", 0.9)
        self.declare_parameter("cartesian_jump_threshold", 0.0)
        self.declare_parameter("cartesian_avoid_collisions", True)
        self.declare_parameter("velocity_scaling", 1.0)
        self.declare_parameter("acceleration_scaling", 1.0)
        self.declare_parameter("synchronous", True)
        self.declare_parameter("post_waypoint_delay", 2.5)
        self.declare_parameter("waypoints", [
            " 0.00, 0.15, 0.00, 20, 05, 05",  # top
            "-0.15, 0.15, 0.00, 20, 20, 05",  # top right 
            "-0.15, 0.00, 0.00,-01, 20, 05",  # right
            "-0.15,-0.15, 0.00,-25, 20, 05",  # bot right
            " 0.00,-0.15, 0.00,-25,-05, 05",  # bot
            " 0.15,-0.15, 0.00,-25,-20,-05",  # bot left
            " 0.15, 0.00, 0.00, 00,-20,-05",  # left
            " 0.15, 0.15, 0.00, 20,-20,-05",  # top left
            " 0.00, 0.15,-0.06, 15, 05, 05",  # top +
            "-0.15, 0.15,-0.06, 15, 15, 05",  # top right +
            "-0.15, 0.00,-0.06,-01, 15, 05",  # right + 
            "-0.15,-0.15,-0.06,-20, 15, 05",  # bot right +
            " 0.00,-0.15,-0.06,-20,-05, 05",  # bot +
            " 0.15,-0.15,-0.06,-20,-15,-05",  # bot left +
            " 0.15, 0.00,-0.06, 00,-15,-05",  # left +
            " 0.15, 0.15,-0.06, 15,-15,-05",  # top left +
            " 0.00, 0.15,-0.10, 15, 05, 05",  # top ++
            "-0.15, 0.15,-0.10, 15, 15, 05",  # top right ++
            "-0.15, 0.00,-0.10,-01, 15, 05",  # right ++
            "-0.15,-0.15,-0.10,-20, 15, 05",  # bot right ++
            " 0.00,-0.15,-0.10,-20,-05, 05",  # bot ++
            " 0.15,-0.15,-0.10,-20,-15,-05",  # bot left ++
            " 0.15, 0.00,-0.10, 00,-15,-05",  # left ++
            " 0.15, 0.15,-0.10, 15,-15,-05",  # top left ++
            " 0.00, 0.10, 0.00, 15, 05, 05",  # top
            "-0.10, 0.10, 0.00, 15, 15, 05",  # top right 
            "-0.10, 0.00, 0.00,-01, 15, 05",  # right
            "-0.10,-0.10, 0.00,-20, 15, 05",  # bot right
            " 0.00,-0.10, 0.00,-20,-05, 05",  # bot
            " 0.10,-0.10, 0.00,-20,-15,-05",  # bot left
            " 0.10, 0.00, 0.00, 00,-15,-05",  # left
            " 0.10, 0.10, 0.00, 15,-15,-05",  # top left
            " 0.00, 0.00, 0.00,-03, 02, 30",  # center
            " 0.00, 0.00,-0.06,-02,-03,-30",  # center +
            " 0.00, 0.00,-0.10,-04, 04, 30",  # center ++
            " 0.00, 0.10,-0.06, 10, 05, 05",  # top +
            "-0.10, 0.10,-0.06, 10, 10, 05",  # top right +
            "-0.10, 0.00,-0.06,-01, 10, 05",  # right + 
            "-0.10,-0.10,-0.06,-15, 10, 05",  # bot right +
            " 0.00,-0.10,-0.06,-15,-05, 05",  # bot +
            " 0.10,-0.10,-0.06,-15,-10,-05",  # bot left +
            " 0.10, 0.00,-0.06, 00,-10,-05",  # left +
            " 0.10, 0.10,-0.06, 10,-10,-05",  # top left +
            " 0.00, 0.10,-0.10, 10, 05, 05",  # top ++
            "-0.10, 0.10,-0.10, 10, 10, 05",  # top right ++
            "-0.10, 0.00,-0.10,-01, 10, 05",  # right ++
            "-0.10,-0.10,-0.10,-15, 10, 05",  # bot right ++
            " 0.00,-0.10,-0.10,-15,-05, 05",  # bot ++
            " 0.10,-0.10,-0.10,-15,-10,-05",  # bot left ++
            " 0.10, 0.00,-0.10, 00,-10,-05",  # left ++
            " 0.10, 0.10,-0.10, 10,-10,-05",  # top left ++

        ])
        self.declare_parameter("initialized", False)

        # ───────────────────────────── Readback ────────────────────────────────
        self.target_frame           = self.get_parameter("target_frame").get_parameter_value().string_value
        self.reference_frame        = self.get_parameter("reference_frame").get_parameter_value().string_value
        self.offset_reference_frame = self.get_parameter("offset_reference_frame").get_parameter_value().string_value
        self.calibration_tag        = self.get_parameter("calibration_tag").get_parameter_value().string_value
        self.planner_id             = self.get_parameter("planner_id").get_parameter_value().string_value
        self.cartesian              = self.get_parameter("cartesian").get_parameter_value().bool_value
        self.cartesian_max_step     = self.get_parameter("cartesian_max_step").get_parameter_value().double_value
        self.cartesian_fraction_threshold = self.get_parameter("cartesian_fraction_threshold").get_parameter_value().double_value
        self.cartesian_jump_threshold     = self.get_parameter("cartesian_jump_threshold").get_parameter_value().double_value
        self.cartesian_avoid_collisions   = self.get_parameter("cartesian_avoid_collisions").get_parameter_value().bool_value
        self.velocity_scaling      = self.get_parameter("velocity_scaling").get_parameter_value().double_value
        self.acceleration_scaling  = self.get_parameter("acceleration_scaling").get_parameter_value().double_value
        self.synchronous           = self.get_parameter("synchronous").get_parameter_value().bool_value
        self.post_waypoint_delay   = self.get_parameter("post_waypoint_delay").get_parameter_value().double_value
        self.waypoints_param       = self.get_parameter("waypoints").get_parameter_value().string_array_value
        self.INITIALIZED           = self.get_parameter("initialized").get_parameter_value().bool_value

        # ─────────────────────────── House‑keeping ─────────────────────────────
        self.create_timer(3.0, self.support_warning_callback)
        self.add_on_set_parameters_callback(self.parameter_callback)

        # ───────────────────────────── TF Buffer ───────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ─────────────────────────── Callback group ────────────────────────────
        self.callback_group = ReentrantCallbackGroup()

        # ───────────────────────────── MoveIt2 ─────────────────────────────────
        self.moveit2 = MoveIt2(
            node=self,
            callback_group=self.callback_group,
            joint_names=[
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
            ],
            base_link_name=self.reference_frame,
            end_effector_name="Link6",
            group_name="nova5_group",
        )
        self.moveit2.planner_id       = self.planner_id
        self.moveit2.max_velocity     = self.velocity_scaling
        self.moveit2.max_acceleration = self.acceleration_scaling
        self.moveit2.cartesian_jump_threshold   = self.cartesian_jump_threshold
        self.moveit2.cartesian_avoid_collisions = self.cartesian_avoid_collisions

    # ────────────────────────── Parameter callback ────────────────────────────
    def parameter_callback(self, params):
        for p in params:
            if p.name == "initialized" and p.type_ == Parameter.Type.BOOL:
                self.INITIALIZED = p.value
        return SetParametersResult(successful=True)

    # ─────────────────────────── Misc. helpers ────────────────────────────────
    def support_warning_callback(self):
        if not self.INITIALIZED:
            self.get_logger().warn(
                "This is the support node for axxb_calibration. "
                "Run 'ros2 run pickn_place axxb_calibration' instead."
            )

    def measure_distance_from_calibration_tag(self):
        self.get_logger().info(
            f"Measuring distance from '{self.calibration_tag}' to '{self.reference_frame}'..."
        )
        while not self.tf_buffer.can_transform(
            self.reference_frame,
            self.calibration_tag,
            rclpy.time.Time(),
            timeout=Duration(seconds=2.0)
        ):
            self.get_logger().warn(
                f"Waiting for transform from '{self.calibration_tag}' to '{self.reference_frame}'..."
            )
            time.sleep(1.0)
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.calibration_tag,
                rclpy.time.Time()
            )
            trans = transform_stamped.transform.translation
            distance = math.sqrt(trans.x**2 + trans.y**2 + trans.z**2)
            self.get_logger().info(
                f"Distance from '{self.calibration_tag}' to '{self.reference_frame}': {distance:.3f} m"
            )
            return distance
        except Exception as e:
            self.get_logger().error(f"Failed to measure distance: {e}")
            return None

    def apply_offset_to_pose(self, base_pos, base_quat, offset):
        # 1. Rotate the translation offset into the calibration_pose axes
        rot_mat = tf_transformations.quaternion_matrix(base_quat)[:3, :3]
        local_offset = rot_mat.dot(np.array(offset[:3]))

        # 2. Compute the new position in world frame
        new_pos_array = np.array(base_pos) + local_offset

        # 3. Compute the orientation offset quaternion
        drot_quat = tf_transformations.quaternion_from_euler(
            math.radians(offset[3]),
            math.radians(offset[4]),
            math.radians(offset[5])
        )

        # 4. Apply the small rotation on top of the base orientation
        new_quat = tf_transformations.quaternion_multiply(base_quat, drot_quat)

        # Convert to Python lists
        return new_pos_array.tolist(), list(new_quat)

    def parse_waypoints(self):
        waypoints = []
        if self.waypoints_param:
            for line in self.waypoints_param:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(',')
                if len(parts) != 6:
                    self.get_logger().warn(f"Invalid waypoint format: {line}")
                    continue
                try:
                    offset = [float(x.strip()) for x in parts]
                    waypoints.append(offset)
                except Exception as e:
                    self.get_logger().warn(f"Error parsing waypoint '{line}': {e}")
            return waypoints
        else:
            wp_str = self.get_parameter("waypoints").get_parameter_value().string_value
            for line in wp_str.splitlines():
                line = line.strip()
                if not line:
                    continue
                parts = line.split(',')
                if len(parts) != 6:
                    self.get_logger().warn(f"Invalid waypoint format: {line}")
                    continue
                try:
                    offset = [float(x.strip()) for x in parts]
                    waypoints.append(offset)
                except Exception as e:
                    self.get_logger().warn(f"Error parsing waypoint '{line}': {e}")
            return waypoints

    def call_capture_point_service(self):
        client = self.create_client(Trigger, "/capture_point")
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Service /capture_point not available")
            return False
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info("Service /capture_point successful: " + response.message)
                return True
            else:
                self.get_logger().warn("Service /capture_point failed: " + response.message)
                return False
        else:
            self.get_logger().warn("Service /capture_point call failed (no response)")
            return False

    def call_capture_point_service_with_retries(self):
        max_attempts = 20
        attempt = 0
        while attempt < max_attempts:
            self.get_logger().info("Calling capture_point service...")
            if self.call_capture_point_service():
                return True
            attempt += 1
            self.get_logger().warn(f"Attempt {attempt} failed. Retrying in 2 sec...")
            time.sleep(2.0)
        self.get_logger().error("All attempts to call capture_point failed.")
        return False

    def run(self):
        # Run the main calibration process.
        self.measure_distance_from_calibration_tag()

        valid_calibration = False
        while not valid_calibration:
            self.get_logger().info("Waiting for TF data...")
            self.create_rate(2).sleep()
            while not self.tf_buffer.can_transform(
                self.reference_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0)
            ):
                self.get_logger().warn(
                    f"No transform from '{self.target_frame}' to '{self.reference_frame}'. Retrying..."
                )
                time.sleep(2.0)
            while True:
                try:
                    transform_stamped = self.tf_buffer.lookup_transform(
                        self.reference_frame,
                        self.target_frame,
                        rclpy.time.Time()
                    )
                    break
                except LookupException as ex:
                    self.get_logger().warn(f"TF lookup failed: {ex}. Retrying...")
                    time.sleep(2.0)
            base_position = [
                transform_stamped.transform.translation.x,
                transform_stamped.transform.translation.y,
                transform_stamped.transform.translation.z,
            ]
            base_quat = [
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w,
            ]
            self.get_logger().info(
                f"Snapped TF pose from '{self.target_frame}' relative to " 
                f"'{self.reference_frame}':\n  Position: {base_position}\n  "
                f"Orientation (xyzw): {base_quat}"
            )
            poses = [(base_position, base_quat)]
            waypoint_offsets = self.parse_waypoints()
            if waypoint_offsets:
                self.get_logger().info(f"Found {len(waypoint_offsets)} waypoint offset(s).")
                if self.offset_reference_frame != self.reference_frame:
                    try:
                        tf_offset = self.tf_buffer.lookup_transform(
                            self.reference_frame,
                            self.offset_reference_frame,
                            rclpy.time.Time()
                        )
                        offset_position = [
                            tf_offset.transform.translation.x,
                            tf_offset.transform.translation.y,
                            tf_offset.transform.translation.z
                        ]
                        offset_quat = [
                            tf_offset.transform.rotation.x,
                            tf_offset.transform.rotation.y,
                            tf_offset.transform.rotation.z,
                            tf_offset.transform.rotation.w
                        ]
                        self.get_logger().info(
                            f"Using '{self.offset_reference_frame}' frame as offset reference"
                        )
                        base_position, base_quat = offset_position, offset_quat
                    except Exception as e:
                        self.get_logger().warn(
                            f"Failed to get transform for offset_reference_frame: {e}"
                        )
                for offset in waypoint_offsets:
                    new_pos, new_quat = self.apply_offset_to_pose(
                        base_position, base_quat, offset
                    )
                    poses.append((new_pos, new_quat))
            else:
                self.get_logger().info("No additional waypoint offsets provided.")
            valid_calibration = True

        for i, (pos, quat) in enumerate(poses):
            self.get_logger().info(f"Moving to waypoint {i}...")
            self.moveit2.move_to_pose(
                position=pos,
                quat_xyzw=quat,
                cartesian=self.cartesian,
                cartesian_max_step=self.cartesian_max_step,
                cartesian_fraction_threshold=self.cartesian_fraction_threshold,
            )
            if self.synchronous:
                self.moveit2.wait_until_executed()
                state = self.moveit2.query_state()
                if state == MoveIt2State.IDLE:
                    self.get_logger().info(f"Waypoint {i} reached.")
                else:
                    self.get_logger().warn(
                        f"Motion to waypoint {i} ended with state: {state}"
                    )
            else:
                self.get_logger().info("Asynchronous execution started.")
                rate = self.create_rate(10)
                while self.moveit2.query_state() not in [
                    MoveIt2State.EXECUTING, MoveIt2State.IDLE
                ]:
                    rate.sleep()
                future = self.moveit2.get_execution_future()
                while not future.done():
                    self.get_logger().info(
                        f"Current state: {self.moveit2.query_state()}"
                    )
                    rate.sleep()
                result = future.result()
                self.get_logger().info(
                    f"Asynchronous motion done for waypoint {i}. "
                    f"Status: {result.status}, error_code: {result.result.error_code}"
                )
            if i > 0:
                self.get_logger().info(
                    f"Waiting for post waypoint delay: {self.post_waypoint_delay:.2f} sec."
                )
                time.sleep(self.post_waypoint_delay)
                if not self.call_capture_point_service_with_retries():
                    return

def main(args=None):
    # Setup signal handlers for graceful shutdown.
    def signal_handler(signum, frame):
        rclpy.shutdown()
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    rclpy.init(args=args)
    node = DirectTfWaypointsExample()
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Perform a single distance check.
    distance = node.measure_distance_from_calibration_tag()
    if distance is None or not (0.83 <= distance <= 0.87):
        node.get_logger().error(
            "Distance not in range [0.83, 0.87]. Exiting node due to invalid distance."
        )
        rclpy.shutdown()
        executor_thread.join()
        node.destroy_node()
        sys.exit(1)

    node.get_logger().info(f"Distance {distance:.3f} m is valid. Proceeding...")
    node.create_rate(1.0).sleep()
    node.run()

    rclpy.shutdown()
    executor_thread.join()
    node.destroy_node()

if __name__ == "__main__":
    main()
