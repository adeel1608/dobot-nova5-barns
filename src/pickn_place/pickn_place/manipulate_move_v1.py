#!/usr/bin/env python3
# type: ignore  # ROS2 types not recognized by static linter
##############################
# CONFIGURATION
##############################

# MOVE FUNCTION
REFERENCE_FRAME = "base_link"               # Reference frame for TF lookup.
PLANNER_ID = "OMPL"                         # Planner ID for MoveIt2.
CARTESIAN = True                            # Use Cartesian planning?
CARTESIAN_MAX_STEP = 0.001                  # Maximum step size for Cartesian motion.
CARTESIAN_FRACTION_THRESHOLD = 0.8          # Fraction threshold for Crtesian planning.
CARTESIAN_JUMP_THRESHOLD = 0.0              # Jump threshold for Cartesian planning.
CARTESIAN_AVOID_COLLISIONS = True           # Enable collision avoidance.
VELOCITY_SCALING = 1.0                      # Velocity scaling factor.
ACCELERATION_SCALING = 1.0                  # Acceleration scaling factor.
SYNCHRONOUS = True                          # Wait for motion to complete?
END_EFFECTOR_NAME = "tool_link"             # End effector name.
DEFAULT_TCP_LINK = END_EFFECTOR_NAME 
GROUP_NAME = "portafilter_center"           # MoveIt2 planning group name.

# TF Stability Sampling Settings
NUM_CONSECUTIVE_TF = 9                      # Number of consecutive TF samples
TRANSLATION_THRESHOLD = 0.002               # Maximum allowed translation diff (meters)
ROTATION_THRESHOLD = 2.1                    # Maximum allowed rotation diff (degrees)
SAMPLE_DELAY = 0.1                          # Delay between samples (seconds)

##############################
# IMPORTS
##############################
import time
import os
import math
import yaml
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionClient

from tf2_ros import Buffer, TransformListener
import tf_transformations
from scipy.spatial.transform import Rotation as Rot

from pymoveit2 import MoveIt2, MoveIt2State

from dobot_msgs_v3.srv import (
    StartDrag,
    StopDrag,
    SetGripperPosition,
    GetGripperPosition,
    GetPose,
    GetAngle
)
from control_msgs.action import FollowJointTrajectory
from std_srvs.srv import Trigger
from std_msgs.msg import Float32,Int8

# For loading the YAML file from the package share folder.
from ament_index_python.packages import get_package_share_directory
import numpy as np
import time
from tf_transformations import euler_matrix, quaternion_from_matrix

##############################
# HELPER FUNCTIONS (Module Level)
##############################
def get_transform_list(tf_stamped):
    """
    Converts a geometry_msgs TransformStamped message into a list: 
    [tx, ty, tz, qx, qy, qz, qw].
    """
    t = tf_stamped.transform.translation
    r = tf_stamped.transform.rotation
    return [t.x, t.y, t.z, r.x, r.y, r.z, r.w]

def compute_translation_rotation_diff(tf1, tf2):
    """
    Computes the translation and rotation difference between two transforms.
    Returns a tuple: (translation_difference_in_meters, rotation_difference_in_degrees)
    """
    t1 = np.array(tf1[:3])
    t2 = np.array(tf2[:3])
    trans_diff = np.linalg.norm(t2 - t1)
    
    r1 = Rot.from_quat(tf1[3:])
    r2 = Rot.from_quat(tf2[3:])
    relative_rotation = r1.inv() * r2
    rot_deg = math.degrees(relative_rotation.magnitude())
    
    return trans_diff, rot_deg

def compute_max_spread(samples):
    """
    Computes the maximum translation and rotation differences among a list of transforms.
    """
    max_trans_diff = 0.0
    max_rot_diff = 0.0
    n = len(samples)
    for i in range(n):
        for j in range(i+1, n):
            trans_diff, rot_diff = compute_translation_rotation_diff(samples[i], samples[j])
            max_trans_diff = max(max_trans_diff, trans_diff)
            max_rot_diff = max(max_rot_diff, rot_diff)
    return max_trans_diff, max_rot_diff

def average_quaternions(quaternions):
    """
    Computes the average quaternion (using the Markley method) from a list of quaternions.
    Each quaternion is assumed to be in [qx, qy, qz, qw] format.
    """
    aligned = []
    for q in quaternions:
        if np.dot(q, quaternions[0]) < 0:
            aligned.append(-q)
        else:
            aligned.append(q)
    aligned = np.array(aligned)
    
    M = np.zeros((4, 4))
    for q in aligned:
        M += np.outer(q, q)
    M /= aligned.shape[0]
    
    eigenvalues, eigenvectors = np.linalg.eig(M)
    max_index = np.argmax(eigenvalues)
    avg_q = eigenvectors[:, max_index]
    norm = np.linalg.norm(avg_q)
    if norm < 1e-8:
        return np.array([0., 0., 0., 1.])
    return avg_q / norm

def average_transforms(transforms):
    """
    Averages a list of transforms (each as [tx, ty, tz, qx, qy, qz, qw]) and returns a single averaged transform.
    """
    translations = np.array([t[:3] for t in transforms])
    avg_translation = np.mean(translations, axis=0)
    quaternions = np.array([t[3:] for t in transforms])
    avg_quat = average_quaternions(quaternions)
    return list(avg_translation) + list(avg_quat)

##############################
# MAIN NODE DEFINITION
##############################
class DirectTfMotionNode(Node):
    def __init__(self):
        super().__init__("direct_tf_motion_node")
        
        # Configuration variables.
        self.reference_frame = REFERENCE_FRAME
        self.planner_id = PLANNER_ID
        self.cartesian = CARTESIAN
        self.cartesian_max_step = CARTESIAN_MAX_STEP
        self.cartesian_fraction_threshold = CARTESIAN_FRACTION_THRESHOLD
        self.cartesian_jump_threshold = CARTESIAN_JUMP_THRESHOLD
        self.cartesian_avoid_collisions = CARTESIAN_AVOID_COLLISIONS
        self.velocity_scaling = VELOCITY_SCALING
        self.acceleration_scaling = ACCELERATION_SCALING
        self.synchronous = SYNCHRONOUS
        # TF stability flag.
        self.ignore_orientation = False

        # TF Buffer and Listener.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize MoveIt2 with robot details.
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            base_link_name=self.reference_frame,
            end_effector_name=END_EFFECTOR_NAME,
            group_name=GROUP_NAME,
        )
        self.moveit2.planner_id = self.planner_id
        self.moveit2.max_velocity = self.velocity_scaling
        self.moveit2.max_acceleration = self.acceleration_scaling
        self.moveit2.cartesian_jump_threshold = self.cartesian_jump_threshold
        self.moveit2.cartesian_avoid_collisions = self.cartesian_avoid_collisions

        # ─── Pre-create all clients and action clients ─────────────────────
        mane = os.getenv("DOBOT_TYPE", "default_dobot")
        action_topic = f"{mane}_group_controller/follow_joint_trajectory"

        # Drag mode services
        self.start_drag_cli   = self.create_client(StartDrag, '/dobot_bringup_v3/srv/StartDrag')
        self.stop_drag_cli    = self.create_client(StopDrag,  '/dobot_bringup_v3/srv/StopDrag')
        # Gripper services
        self.set_gripper_cli  = self.create_client(SetGripperPosition, '/dobot_bringup_v3/srv/SetGripperPosition')
        self.get_gripper_cli  = self.create_client(GetGripperPosition, '/dobot_bringup_v3/srv/GetGripperPosition')
        # Pose-query service
        self.get_pose_cli     = self.create_client(GetPose, '/dobot_bringup_v3/srv/GetPose')
        self.get_angles_cli   = self.create_client(GetAngle, '/dobot_bringup_v3/srv/GetAngle')
        # Trajectory Action
        self.traj_cli         = ActionClient(self, FollowJointTrajectory, action_topic)

        # Wait once for each
        for cli, name in [
            (self.get_pose_cli,    "GetPose"),
        ]:
            if not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f"{name} service not available at init")
        if not self.traj_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Trajectory action server not available at init")

    def get_tf(self, target_frame: str, max_retries: int = 1, sleep_time: float = 0.5):
        """
        Directly retrieves the transform from the robot's base (reference frame) to the target_frame.
        Returns a tuple (pose, timestamp) where:
        - pose: [tx, ty, tz, qx, qy, qz, qw]
        - timestamp: time in seconds when the transform was captured
        """
        for attempt in range(max_retries + 1):
            try:
                # Lookup transform directly: from self.reference_frame (base_link) to target_frame.
                tf_obj = self.tf_buffer.lookup_transform(
                    self.reference_frame,  # e.g., "base_link"
                    target_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5)
                )
                now_sec = self.get_clock().now().nanoseconds * 1e-9
                stamp_sec = tf_obj.header.stamp.sec + tf_obj.header.stamp.nanosec * 1e-9
                # Check that the transform is fresh (within 0.2 seconds)
                if now_sec - stamp_sec > 0.2:
                    raise RuntimeError(f"Direct TF stale ({now_sec - stamp_sec:.2f}s old)")

                # Extract translation and rotation to build the pose.
                t = tf_obj.transform.translation
                r = tf_obj.transform.rotation
                pose = [t.x, t.y, t.z, r.x, r.y, r.z, r.w]

                self.get_logger().info(f"Aquiring TF: ({now_sec:.3f}), pose: {pose}")
                return pose, now_sec

            except Exception as e:
                self.get_logger().warn(f"get_tf(): Attempt {attempt+1}/{max_retries+1} failed: {e}")
                time.sleep(sleep_time)

        return None, None

    def wait_for_stable_tf(self, target_tf: str):
        """
        Synchronously waits indefinitely for a stable transform from target_tf.
        This version uses a local sample buffer so that each operation is independent.
        It continuously collects samples until a sliding window of NUM_CONSECUTIVE_TF samples meets the stability criteria,
        and the last sample's timestamp is within the freshness threshold.
        Then it returns the averaged transform computed from the window.
        """
        samples_buffer = []
        freshness_threshold = 0.2  
        while True:
            transform, stamp = self.get_tf(target_tf, max_retries=1, sleep_time=0.1)
            if transform is not None:
                samples_buffer.append((transform, stamp))
                if len(samples_buffer) >= NUM_CONSECUTIVE_TF:
                    window = samples_buffer[-NUM_CONSECUTIVE_TF:]
                    last_sample_stamp = window[-1][1]
                    current_time = self.get_clock().now().nanoseconds * 1e-9
                    if current_time - last_sample_stamp > freshness_threshold:
                        self.get_logger().warn(
                            f"wait_for_stable_tf(): Latest sample is stale (delta: {current_time - last_sample_stamp:.3f}s), waiting for fresh data."
                        )
                        time.sleep(0.5)
                        continue
                    window_transforms = [sample[0] for sample in window]
                    max_trans_diff, max_rot_diff = compute_max_spread(window_transforms)
                    if self.ignore_orientation:
                        if max_trans_diff <= TRANSLATION_THRESHOLD:
                            return average_transforms(window_transforms)
                    else:
                        if max_trans_diff <= TRANSLATION_THRESHOLD and max_rot_diff <= ROTATION_THRESHOLD:
                            return average_transforms(window_transforms)
            time.sleep(0.1)

    def wait_for_servo_ready(self, timeout: float = 15.0) -> bool:
        """
        Poll /get_servo_status (Trigger) until status == 0 (READY) or timeout.
        """
        client = self.create_client(Trigger, 'get_servo_status')
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("Servo status service unavailable")
            return False

        start = self.get_clock().now().nanoseconds * 1e-9
        while True:
            req = Trigger.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            res = future.result()
            if res is not None:
                try:
                    code = int(res.message)
                except:
                    code = None
                if code == 0:
                    return True
                self.get_logger().info(f"Servo busy (status={code}), waiting…")
            else:
                self.get_logger().warn("No response from /get_servo_status")

            if (self.get_clock().now().nanoseconds * 1e-9 - start) > timeout:
                self.get_logger().error("Timeout waiting for servo READY")
                return False
            time.sleep(0.1)

    def verify_goal_pose(self, expected_goal_pose, tolerance, offset):
        expected_mm = np.array(expected_goal_pose) * 1000.0
        self.get_logger().info(f"Goal pose (expected) in mm: {expected_mm}")
        client = self.create_client(GetPose, '/dobot_bringup_v3/srv/GetPose')
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error("verify_goal_pose(): Pose service not available.")
            return False
        req = GetPose.Request()
        req.user = 0
        req.tool = 0
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(1.0)
        if future.result() is None:
            self.get_logger().error("verify_goal_pose(): Failed to retrieve pose from service.")
            return False
        response = future.result()
        pose_str = response.pose.strip('{}')
        parts = pose_str.split(',')
        if len(parts) < 3:
            self.get_logger().error("verify_goal_pose(): Invalid pose received.")
            return False
        try:
            current_translation = np.array([float(parts[i]) for i in range(3)])
        except Exception as e:
            self.get_logger().error(f"verify_goal_pose(): Error parsing current translation: {e}")
            return False

        diff = np.linalg.norm(expected_mm - current_translation) - offset
        diff = max(diff, 0.0)
        self.get_logger().info(f"Computed diff after offset: {diff:.2f} mm")
        if diff <= tolerance:
            self.get_logger().info(f"verify_goal_pose(): Arrived, Translation match OK (diff: {diff:.2f} mm).")
            return True
        else:
            self.get_logger().warn(f"verify_goal_pose(): Approaching to goal (diff: {diff:.2f} mm).")
            return False
        
    def verify_joint_positions(self, expected_joints, tolerance_deg=1.0):
        """
        Verifies that current joint positions match expected positions within tolerance.
        
        Args:
            expected_joints: List of 6 joint angles in degrees
            tolerance_deg: Acceptable deviation in degrees (default: 1.0°)
        
        Returns:
            bool: True if all joints are within tolerance
        """
        joint_state_msg = self.wait_for_joint_state("/joint_states_robot", timeout_sec=2.0)
        if joint_state_msg is None:
            self.get_logger().error("verify_joint_positions(): No joint state message received.")
            return False
        
        current_joints_deg = [math.degrees(j) for j in joint_state_msg.position[0:6]]
        diffs = [abs(c - e) for c, e in zip(current_joints_deg, expected_joints)]
        
        max_diff = max(diffs)
        within_tolerance = max_diff <= tolerance_deg
        
        if within_tolerance:
            self.get_logger().info("verify_joint_positions(): Joint positions verified within tolerance.")
        else:
            self.get_logger().warn(
                f"verify_joint_positions(): Joint differences exceed tolerance:\n"
                f"Current (deg): {current_joints_deg}\n"
                f"Expected (deg): {expected_joints}\n"
                f"Max difference: {max_diff:.2f}°"
            )
        
        return within_tolerance

    def wait_for_joint_state(self, topic_name: str, timeout_sec: float = 2.0):
        """
        Synchronously waits for a JointState message from the given topic.
        """
        from sensor_msgs.msg import JointState
        import rclpy
        msg_container = {"msg": None}

        def callback(msg):
            msg_container["msg"] = msg

        sub = self.create_subscription(JointState, topic_name, callback, 10)
        start_time = self.get_clock().now().nanoseconds * 1e-9
        while msg_container["msg"] is None:
            current_time = self.get_clock().now().nanoseconds * 1e-9
            if (current_time - start_time) > timeout_sec:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return msg_container["msg"]
    
    def set_servo_timing(self, timing_sec: float,
                        settle_time: float = 0.15) -> bool:
        """
        Publish one Float32 on /sleep_timing, wait ≥50 ms, then
        verify with /get_sleep_timing (std_srvs/Trigger).

        * timing_sec must be 0.012 … 0.030  (12 – 30 ms)
        * A minimum 0.05 s pause is enforced after publishing.
        * Publisher/client are created lazily on first use.
        """
        # 0) bounds check ---------------------------------------------------
        if not (0.10 <= timing_sec <= 0.30):
            self.get_logger().error(
                f"set_servo_timing(): {timing_sec:.3f}s outside 0.12 – 0.30 s")
            return False

        # 1) lazy-create pub & client --------------------------------------
        if not hasattr(self, "_sleep_pub"):
            self._sleep_pub = self.create_publisher(Float32,
                                                    "/sleep_timing", 10)
        if not hasattr(self, "_sleep_cli"):
            self._sleep_cli = self.create_client(Trigger,
                                                "/get_sleep_timing")

        # 2) publish once ---------------------------------------------------
        msg = Float32(data=float(timing_sec))
        self._sleep_pub.publish(msg)
        self.get_logger().info(
            f"set_servo_timing(): published {msg.data*1000:.1f} ms")

        # ensure the datagram is processed by the rcl layer
        rclpy.spin_once(self, timeout_sec=0.05)

        # 3) wait ≥50 ms before verification -------------------------------
        rclpy.spin_once(self, timeout_sec=max(settle_time, 0.05))

        # 4) verify via /get_sleep_timing ----------------------------------
        if not self._sleep_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("set_servo_timing(): verification service unavailable")
            return False

        fut = self._sleep_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.5)

        if fut.result() is None or not fut.result().success:
            self.get_logger().error("set_servo_timing(): service call failed")
            return False

        # parse numeric value (if driver echoes it)
        import re, math
        m = re.search(r"([\d\.]+)", fut.result().message)
        if m and math.isclose(float(m.group(1)), timing_sec, abs_tol=1e-4):
            self.get_logger().info(
                f"set_servo_timing(): verified {float(m.group(1))*1000:.1f} ms")
            return True

        self.get_logger().warn("set_servo_timing(): could not confirm timing value")
        return False

    def reset_servo_error(self, pause_after_pub: float = 0.10) -> bool:
        """
        Reset the Dobot's servo-controller from ERROR ➜ READY.

        Steps
        -----
        1)  publish Int8(data=0) once on /servo_controller_status
        2)  wait ≥ pause_after_pub seconds (default 100 ms, enforced ≥ 50 ms)
        3)  call /get_servo_status to confirm status == 0
        """
        # 1) lazy resources -------------------------------------------------
        if not hasattr(self, "_servo_status_pub"):
            self._servo_status_pub = self.create_publisher(
                Int8, "/servo_controller_status", 10)
        if not hasattr(self, "_servo_status_cli"):
            self._servo_status_cli = self.create_client(
                Trigger, "/get_servo_status")

        # 2) publish once ---------------------------------------------------
        self._servo_status_pub.publish(Int8(data=0))
        self.get_logger().info("reset_servo_error(): sent RESET (data=0)")

        # push ROS queue; flush() is not available in rclpy
        rclpy.spin_once(self, timeout_sec=0.05)

        # 3) mandatory pause ≥ 50 ms ---------------------------------------
        rclpy.spin_once(self, timeout_sec=max(pause_after_pub, 0.05))

        # 4) verify ---------------------------------------------------------
        if not self._servo_status_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("reset_servo_error(): verification service unavailable")
            return False

        fut = self._servo_status_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.5)

        if fut.result() is None or not fut.result().success:
            self.get_logger().error("reset_servo_error(): service call failed")
            return False

        import re
        m = re.search(r"(-?\d+)", fut.result().message)
        if m and int(m.group(1)) == 0:
            self.get_logger().info("reset_servo_error(): status now READY (0)")
            return True

        self.get_logger().warn("reset_servo_error(): could not confirm READY state")
        return False

    def release_tension(self) -> bool:
        """
        Pulse drag‐mode once to let the Dobot relax, then refresh the robot's current position.
        Retries StartDrag/StopDrag up to max_attempts times each, with proper error handling.
        Returns True on full success, False on any failure.
        """
        max_attempts = 10
        timeout_sec = 1.0
        min_wait_sec = 0.2
        settling_time = 0.2

        def _execute_drag_command(is_start: bool) -> bool:
            """Helper function to execute StartDrag or StopDrag with retries."""
            cli = self.start_drag_cli if is_start else self.stop_drag_cli
            cmd_name = "StartDrag" if is_start else "StopDrag"
            req = StartDrag.Request() if is_start else StopDrag.Request()
            
            for attempt in range(1, max_attempts + 1):
                self.get_logger().info(f"release_tension: {cmd_name} (attempt {attempt}/{max_attempts})")
                
                start_time = time.monotonic()
                future = cli.call_async(req)
                
                # Wait for response with timeout
                rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
                
                # Check result
                if future.done() and future.result() is not None:
                    result = future.result()
                    if getattr(result, "res", 1) == 0:
                        self.get_logger().info(f"release_tension: {cmd_name} succeeded")
                        return True
                    else:
                        error_code = getattr(result, "res", "unknown")
                        self.get_logger().warn(f"release_tension: {cmd_name} attempt {attempt} returned error code {error_code}")
                else:
                    self.get_logger().warn(f"release_tension: {cmd_name} attempt {attempt} timed out after {timeout_sec}s")
                
                # Enforce minimum dwell time for StartDrag
                if is_start:
                    elapsed = time.monotonic() - start_time
                    if elapsed < min_wait_sec:
                        time.sleep(min_wait_sec - elapsed)
                
                # Brief pause before retry
                if attempt < max_attempts:
                    time.sleep(0.1)
            
            self.get_logger().error(f"release_tension: {cmd_name} failed after {max_attempts} attempts")
            return False

        # 1) Activate drag mode
        if not _execute_drag_command(is_start=True):
            return False

        # 2) Settling delay
        self.get_logger().info(f"release_tension: Waiting {settling_time:.2f}s for settling...")
        time.sleep(settling_time)

        # 3) Deactivate drag mode
        if not _execute_drag_command(is_start=False):
            return False

        # 4) Success
        self.get_logger().info("release_tension: Completed successfully.")
        return True

###################################### DONE NEW VERIFICATION ###########################################
    from tf_transformations import quaternion_from_matrix, euler_matrix
    import numpy as np
    import time

    def move_to(
        self,
        target_tf: str,
        distance: float,
        offset_x_mm: float = 0.0,
        offset_y_mm: float = 0.0,
        offset_z_mm: float = 0.0,
        offset_roll_deg: float = 0.0,
        offset_pitch_deg: float = 0.0,
        offset_yaw_deg: float = 0.0,
    ) -> bool:
        """
        Move the end-effector away from the target frame by a specified distance along
        the vector from a base reference point to the target, then apply additional
        translation and orientation offsets.

        This function effectively performs a "retraction" motion away from the target,
        which is useful for safely backing away from objects after interaction.

        Args:
            target_tf:        TF frame name to move away from
            distance:         Distance to retract from target in metres (positive = away from target)
            offset_x_mm:      Additional X offset in millimetres (applied after retraction)
            offset_y_mm:      Additional Y offset in millimetres (applied after retraction)
            offset_z_mm:      Additional Z offset in millimetres (applied after retraction)
            offset_roll_deg:  Additional roll rotation offset in degrees
            offset_pitch_deg: Additional pitch rotation offset in degrees
            offset_yaw_deg:   Additional yaw rotation offset in degrees
            
        Returns:
            bool: True if motion completed successfully, False otherwise
        """
        # Input validation
        if not target_tf or not isinstance(target_tf, str):
            self.get_logger().error("move_to(): Invalid target_tf parameter")
            return False
        
        if distance < 0:
            self.get_logger().error("move_to(): Distance must be non-negative")
            return False

        self.get_logger().info(
            f"move_to(): frame={target_tf}, distance={distance}m, "
            f"offsets=({offset_x_mm}mm, {offset_y_mm}mm, {offset_z_mm}mm), "
            f"orientation offsets=({offset_roll_deg}\u00b0, {offset_pitch_deg}\u00b0, {offset_yaw_deg}\u00b0)"
        )

        try:
            # 1) Get a stable transform
            self.ignore_orientation = True
            stable_tf = self.wait_for_stable_tf(target_tf)
            self.ignore_orientation = False
            
            if stable_tf is None:
                self.get_logger().error("move_to(): failed to get stable TF.")
                return False

            # 2) Compute goal position
            target_pos = np.array(stable_tf[:3])
            base_ref = np.array([0.0, 0.0, target_pos[2] + 0.075])
            approach_vec = target_pos - base_ref
            
            # Validate approach vector
            approach_norm = np.linalg.norm(approach_vec)
            if approach_norm < 1e-6:
                self.get_logger().error("move_to(): approach vector too small.")
                return False
            
            unit_vec = approach_vec / approach_norm
            goal_pos = target_pos - distance * unit_vec

            # 3) Apply translation offsets (convert mm to m)
            if any(offset != 0.0 for offset in [offset_x_mm, offset_y_mm, offset_z_mm]):
                offsets_m = np.array([offset_x_mm, offset_y_mm, offset_z_mm]) / 1000.0
                goal_pos += offsets_m

            # 4) Compute orientation
            goal_quat = self._compute_goal_orientation(unit_vec, offset_roll_deg, offset_pitch_deg, offset_yaw_deg)
            if goal_quat is None:
                return False

            # 5) Execute Cartesian move
            self.moveit2.move_to_pose(
                position=goal_pos.tolist(),
                quat_xyzw=goal_quat,
                cartesian=self.cartesian,
                cartesian_max_step=self.cartesian_max_step,
                cartesian_fraction_threshold=self.cartesian_fraction_threshold,
            )

            # 6) Verify arrival
            if not self._wait_for_motion_completion():
                return False

            self.get_logger().info("move_to(): completed successfully.")
            return True

        except Exception as e:
            self.get_logger().error(f"move_to(): Unexpected error: {e}")
            return False

    def _compute_goal_orientation(self, unit_vec: np.ndarray, roll_deg: float, pitch_deg: float, yaw_deg: float) -> list | None:
        """
        Compute the goal orientation quaternion based on unit vector and rotation offsets.
        
        Args:
            unit_vec: Unit vector defining the approach direction
            roll_deg, pitch_deg, yaw_deg: Rotation offsets in degrees
            
        Returns:
            Quaternion as [x, y, z, w] or None on error
        """
        try:
            # Build base orientation matrix
            z_axis = unit_vec
            up = np.array([0, 0, 1])
            
            # Handle near-vertical approach vectors
            if abs(np.dot(z_axis, up)) > 0.99:
                up = np.array([0, 1, 0])
            
            x_axis = np.cross(up, z_axis)
            x_axis /= np.linalg.norm(x_axis)
            y_axis = np.cross(z_axis, x_axis)
            R_base = np.column_stack((x_axis, y_axis, z_axis))

            # Apply rotation offsets if any
            if any(angle != 0.0 for angle in [roll_deg, pitch_deg, yaw_deg]):
                roll = np.deg2rad(roll_deg)
                pitch = np.deg2rad(pitch_deg)
                yaw = np.deg2rad(yaw_deg)
                R_offset = tf_transformations.euler_matrix(roll, pitch, yaw, axes='sxyz')[0:3, 0:3]
                R_total = R_base.dot(R_offset)
            else:
                R_total = R_base

            # Convert to quaternion
            H = np.eye(4)
            H[0:3, 0:3] = R_total
            return tf_transformations.quaternion_from_matrix(H)
            
        except Exception as e:
            self.get_logger().error(f"_compute_goal_orientation(): Error computing orientation: {e}")
            return None

    def _wait_for_motion_completion(self, timeout: float = 15.0) -> bool:
        """
        Wait for motion completion and verify servo readiness.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            True if motion completed successfully, False otherwise
        """
        try:
            self.moveit2.wait_until_executed()
            
            start_time = time.monotonic()
            while not self.wait_for_servo_ready(timeout=timeout):
                if time.monotonic() - start_time > timeout:
                    self.get_logger().error("_wait_for_motion_completion(): Timeout waiting for servo ready")
                    return False
                self.get_logger().warn("_wait_for_motion_completion(): not arrived, rechecking...")
                time.sleep(0.2)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"_wait_for_motion_completion(): Error: {e}")
            return False

    def approach_tool(self, target_tf: str, gripper_position: int = 255) -> bool:
        """
        Move the end-effector to a pre-defined "approach" offset for the given target_tf,
        then command the gripper to the specified position.

        Args:
            target_tf (str): name of the TF frame to approach
            gripper_position (int): 0–255 gripper position (0=open, 255=closed)
            
        Returns:
            bool: True if successful, False otherwise
        """
        # Input validation
        if not target_tf or not isinstance(target_tf, str):
            self.get_logger().error("approach_tool(): Invalid target_tf parameter")
            return False
        
        gripper_position = max(0, min(255, int(gripper_position)))
        
        self.get_logger().info(f"approach_tool(): Using transform for '{target_tf}'")

        try:
            # 1) Wait for a stable transform to the target frame
            stable_tf = self.wait_for_stable_tf(target_tf)
            if stable_tf is None:
                self.get_logger().error("approach_tool(): Failed to obtain a stable transform.")
                return False

            # 2) Load and validate YAML offsets
            offsets_data = self._load_tool_offsets()
            if offsets_data is None:
                return False
                
            if target_tf not in offsets_data:
                self.get_logger().error(f"approach_tool(): No offset points for target TF '{target_tf}'")
                return False

            tf_offsets = offsets_data[target_tf]
            if "approach_pose" not in tf_offsets:
                self.get_logger().error(f"approach_tool(): Missing approach_pose offset for {target_tf}")
                return False

            # 3) Compute approach pose
            approach_pose = self._compute_approach_pose(stable_tf, tf_offsets["approach_pose"])
            if approach_pose is None:
                return False

            # 4) Execute motion
            if not self._execute_approach_motion(approach_pose):
                return False

            # 5) Command gripper
            if not self._command_gripper(gripper_position):
                return False

            self.get_logger().info("approach_tool(): Function completed successfully.")
            return True

        except Exception as e:
            self.get_logger().error(f"approach_tool(): Unexpected error: {e}")
            return False

    def _load_tool_offsets(self) -> dict | None:
        """Load tool offset data from YAML file."""
        try:
            package_share = get_package_share_directory("pickn_place")
            file_path = os.path.join(package_share, "tool_offset_points.yaml")
            with open(file_path, "r") as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"_load_tool_offsets(): Failed to load YAML file: {e}")
            return None

    def _compute_approach_pose(self, stable_tf: list, approach_offset: dict) -> tuple | None:
        """Compute the approach pose from stable transform and offset data."""
        try:
            # Build input transform matrix
            t, q = stable_tf[:3], stable_tf[3:]
            input_matrix = tf_transformations.quaternion_matrix(q)
            input_matrix[0:3, 3] = t

            # Build offset matrix
            off_t = [approach_offset["translation"][axis] for axis in ("x", "y", "z")]
            off_q = [approach_offset["rotation"][axis] for axis in ("x", "y", "z", "w")]
            approach_matrix = tf_transformations.quaternion_matrix(off_q)
            approach_matrix[0:3, 3] = off_t

            # Compute goal pose
            goal_matrix = np.dot(input_matrix, approach_matrix)
            approach_position = goal_matrix[0:3, 3].tolist()
            approach_quaternion = tf_transformations.quaternion_from_matrix(goal_matrix)

            self.get_logger().info(
                f"_compute_approach_pose(): Computed approach pose:\n"
                f"  Position: {approach_position}\n"
                f"  Orientation (xyzw): {approach_quaternion}"
            )
            
            return approach_position, approach_quaternion
            
        except Exception as e:
            self.get_logger().error(f"_compute_approach_pose(): Error computing pose: {e}")
            return None

    def _execute_approach_motion(self, approach_pose: tuple) -> bool:
        """Execute the approach motion and verify completion."""
        try:
            position, quaternion = approach_pose
            
            # Execute Cartesian move
            self.moveit2.move_to_pose(
                position=position,
                quat_xyzw=quaternion,
                cartesian=self.cartesian,
                cartesian_max_step=self.cartesian_max_step,
                cartesian_fraction_threshold=self.cartesian_fraction_threshold,
            )
            
            # Wait for execution
            self.moveit2.wait_until_executed()
            state = self.moveit2.query_state()
            
            if state == MoveIt2State.IDLE:
                self.get_logger().info("_execute_approach_motion(): Motion reached successfully.")
            else:
                self.get_logger().warn(f"_execute_approach_motion(): Motion ended with state: {state}")

            # Verify arrival within tolerance
            max_attempts = 10
            for attempt in range(max_attempts):
                if self.verify_goal_pose(expected_goal_pose=position, tolerance=2.0, offset=136.55):
                    return True
                if attempt < max_attempts - 1:
                    self.get_logger().warn(f"_execute_approach_motion(): Goal pose verification failed (attempt {attempt + 1}/{max_attempts}), retrying.")
                    time.sleep(0.2)
            
            self.get_logger().error("_execute_approach_motion(): Failed to verify goal pose after all attempts")
            return False
            
        except Exception as e:
            self.get_logger().error(f"_execute_approach_motion(): Error: {e}")
            return False

    def _command_gripper(self, gripper_position: int) -> bool:
        """Command the gripper to the specified position."""
        try:
            if not self.set_gripper_cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("_command_gripper(): SetGripperPosition service unavailable.")
                return False
                
            req = SetGripperPosition.Request()
            req.position = gripper_position
            req.speed = 255
            req.force = 255
            
            future = self.set_gripper_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is None:
                self.get_logger().error("_command_gripper(): Failed to command gripper")
                return False
                
            self.get_logger().info(f"_command_gripper(): Gripper commanded to position {gripper_position}")
            time.sleep(0.5)  # Gripper settling time
            return True
            
        except Exception as e:
            self.get_logger().error(f"_command_gripper(): Error: {e}")
            return False

    def grab_tool(
        self,
        target_tf: str,
        verify_min: int = 130,
        verify_max: int = 140,
        close_position: int = 255,
    ) -> bool:
        """
        Close the gripper on the object located at *target_tf* and verify the
        grasp. On verification failure the gripper is reopened to its original
        width, the robot retreats to the saved fallback pose, and the function
        returns False.

        Parameters
        ----------
        target_tf : str
            Dynamic TF frame attached to the object to grab.
        verify_min, verify_max : int
            Acceptable range for the gripper reading after closing.
        close_position : int
            Raw position to send to the gripper (0 = fully closed).

        Returns
        -------
        bool
            True on success, False otherwise.
        """
        # Input validation
        if not target_tf or not isinstance(target_tf, str):
            self.get_logger().error("grab_tool(): Invalid target_tf parameter")
            return False
        
        if verify_min > verify_max:
            self.get_logger().error("grab_tool(): verify_min must be <= verify_max")
            return False
        
        close_position = max(0, min(255, int(close_position)))

        fallback_pose = None
        open_ref_pos = None
        fallback_moved = False

        try:
            # 1) Get stable TF for the object
            stable_tf = self.wait_for_stable_tf(target_tf)
            if stable_tf is None:
                self.get_logger().error("grab_tool(): failed to obtain stable TF.")
                return False

            # 2) Save current pose as fallback
            fallback_pose = self._get_current_pose_as_fallback()
            if fallback_pose is None:
                return False

            # 3) Get current gripper position
            open_ref_pos = self._get_current_gripper_position()

            # 4) Load grab offset from YAML
            grab_offset = self._load_grab_offset(target_tf)
            if grab_offset is None:
                return False

            # 5) Compute and execute goal pose
            goal_pose = self._compute_grab_pose(stable_tf, grab_offset)
            if goal_pose is None:
                return False

            if not self._execute_grab_motion(goal_pose, fallback_pose):
                fallback_moved = True
                return False

            # 6) Close gripper and verify
            if not self._close_gripper_and_verify(close_position, verify_min, verify_max, open_ref_pos):
                if fallback_pose:
                    self._retreat_to_fallback(fallback_pose)
                    fallback_moved = True
                return False

            self.get_logger().info("grab_tool(): grasp verified – success.")
            return True

        except Exception as e:
            self.get_logger().error(f"grab_tool(): exception {e}")
            if fallback_pose and not fallback_moved:
                self._retreat_to_fallback(fallback_pose)
            return False

    def _get_current_pose_as_fallback(self) -> list | None:
        """Get current robot pose and convert to fallback format."""
        try:
            if not self.get_pose_cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("_get_current_pose_as_fallback(): GetPose service not available.")
                return None

            req = GetPose.Request()
            req.user = 0
            req.tool = 0
            future = self.get_pose_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.result() is None:
                self.get_logger().error("_get_current_pose_as_fallback(): GetPose call failed.")
                return None

            pose_vals = [float(v) for v in future.result().pose.strip("{}").split(",")]
            if len(pose_vals) < 6:
                self.get_logger().error("_get_current_pose_as_fallback(): malformed GetPose response.")
                return None

            tx, ty, tz, rx, ry, rz = pose_vals[:6]
            fallback_translation = [tx / 1000.0, ty / 1000.0, tz / 1000.0]
            R_fb = tf_transformations.euler_matrix(
                math.radians(rx), math.radians(ry), math.radians(rz)
            )
            fallback_quat = tf_transformations.quaternion_from_matrix(R_fb)
            fallback_pose = fallback_translation + list(fallback_quat)
            
            self.get_logger().info(f"_get_current_pose_as_fallback(): saved fallback pose: {fallback_pose}")
            return fallback_pose

        except Exception as e:
            self.get_logger().error(f"_get_current_pose_as_fallback(): Error: {e}")
            return None

    def _get_current_gripper_position(self) -> int | None:
        """Get current gripper position."""
        try:
            if not self.get_gripper_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("_get_current_gripper_position(): GetGripperPosition service unavailable.")
                return None
                
            req = GetGripperPosition.Request()
            req.index = 0
            future = self.get_gripper_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.result() is None:
                return None
                
            position = future.result().position
            self.get_logger().info(f"_get_current_gripper_position(): ref gripper pos={position}")
            return position
            
        except Exception as e:
            self.get_logger().error(f"_get_current_gripper_position(): Error: {e}")
            return None

    def _load_grab_offset(self, target_tf: str) -> dict | None:
        """Load grab offset from YAML file."""
        try:
            pkg_share = get_package_share_directory("pickn_place")
            yfile = os.path.join(pkg_share, "tool_offset_points.yaml")
            tf_offsets = yaml.safe_load(open(yfile, "r")) or {}

            if target_tf not in tf_offsets or "grab_pose" not in tf_offsets[target_tf]:
                self.get_logger().error("_load_grab_offset(): grab_pose missing in YAML.")
                return None

            return tf_offsets[target_tf]["grab_pose"]

        except Exception as e:
            self.get_logger().error(f"_load_grab_offset(): Error loading YAML: {e}")
            return None

    def _compute_grab_pose(self, stable_tf: list, grab_offset: dict) -> tuple | None:
        """Compute the grab pose from stable transform and offset data."""
        try:
            # Build dynamic transform matrix
            t_dyn, q_dyn = stable_tf[:3], stable_tf[3:]
            M_dyn = tf_transformations.quaternion_matrix(q_dyn)
            M_dyn[0:3, 3] = t_dyn

            # Build offset matrix
            off_t = [grab_offset["translation"][k] for k in ("x", "y", "z")]
            off_q = [grab_offset["rotation"][k] for k in ("x", "y", "z", "w")]
            M_off = tf_transformations.quaternion_matrix(off_q)
            M_off[0:3, 3] = off_t

            # Compute goal pose
            M_goal = M_dyn.dot(M_off)
            goal_pos = M_goal[0:3, 3].tolist()
            goal_quat = tf_transformations.quaternion_from_matrix(M_goal)
            
            self.get_logger().info(f"_compute_grab_pose(): goal pose\n  pos={goal_pos}\n  quat={goal_quat}")
            return goal_pos, goal_quat

        except Exception as e:
            self.get_logger().error(f"_compute_grab_pose(): Error computing pose: {e}")
            return None

    def _execute_grab_motion(self, goal_pose: tuple, fallback_pose: list) -> bool:
        """Execute the grab motion and verify completion."""
        try:
            goal_pos, goal_quat = goal_pose
            
            # Execute motion
            self.moveit2.move_to_pose(
                position=goal_pos,
                quat_xyzw=goal_quat,
                cartesian=self.cartesian,
                cartesian_max_step=self.cartesian_max_step,
                cartesian_fraction_threshold=self.cartesian_fraction_threshold,
            )
            self.moveit2.wait_until_executed()
            
            state = self.moveit2.query_state()
            if state != MoveIt2State.IDLE:
                self.get_logger().warn(f"_execute_grab_motion(): motion finished with state {state}")

            # Verify arrival
            max_wait = 2.0
            start_time = time.monotonic()
            while not self.verify_goal_pose(goal_pos, 2.0, 136.55):
                if time.monotonic() - start_time > max_wait:
                    self.get_logger().warn("_execute_grab_motion(): pose not verified, retreating.")
                    return False
                self.get_logger().warn("_execute_grab_motion(): waiting for pose verification...")
                time.sleep(0.2)
            
            return True

        except Exception as e:
            self.get_logger().error(f"_execute_grab_motion(): Error: {e}")
            return False

    def _close_gripper_and_verify(self, close_position: int, verify_min: int, verify_max: int, open_ref_pos: int | None) -> bool:
        """Close gripper and verify the grasp."""
        try:
            # Close gripper
            if not self.set_gripper_cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("_close_gripper_and_verify(): SetGripperPosition unavailable.")
                return False
                
            req = SetGripperPosition.Request()
            req.position = close_position
            req.speed = 255
            req.force = 255
            self.set_gripper_cli.call_async(req)
            time.sleep(0.2)

            # Verify gripper position
            if not self.get_gripper_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("_close_gripper_and_verify(): GetGripperPosition unavailable.")
                return False
                
            g_req = GetGripperPosition.Request()
            g_req.index = 0
            g_fut = self.get_gripper_cli.call_async(g_req)
            rclpy.spin_until_future_complete(self, g_fut, timeout_sec=1.0)
            
            pos = g_fut.result().position if g_fut.result() else 255
            self.get_logger().info(f"_close_gripper_and_verify(): gripper reading = {pos}")

            # Check if grasp is successful
            if verify_min <= pos <= verify_max:
                return True
            else:
                self.get_logger().warn("_close_gripper_and_verify(): verification failed – reopening gripper.")
                # Reopen gripper to original position
                if open_ref_pos is not None and self.set_gripper_cli.wait_for_service(timeout_sec=1.0):
                    reopen_req = SetGripperPosition.Request()
                    reopen_req.position = open_ref_pos
                    reopen_req.speed = 255
                    reopen_req.force = 255
                    self.set_gripper_cli.call_async(reopen_req)
                    time.sleep(0.2)
                return False

        except Exception as e:
            self.get_logger().error(f"_close_gripper_and_verify(): Error: {e}")
            return False

    def _retreat_to_fallback(self, fallback_pose: list) -> bool:
        """Retreat to the saved fallback pose."""
        try:
            self.get_logger().info("_retreat_to_fallback(): returning to fallback pose.")
            
            ee6 = MoveIt2(
                node=self,
                joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                base_link_name=self.reference_frame,
                end_effector_name="Link6",
                group_name=GROUP_NAME,
            )
            ee6.planner_id = self.planner_id
            ee6.max_velocity = self.velocity_scaling
            ee6.max_acceleration = self.acceleration_scaling
            ee6.cartesian_jump_threshold = self.cartesian_jump_threshold
            ee6.cartesian_avoid_collisions = self.cartesian_avoid_collisions

            ee6.move_to_pose(
                position=fallback_pose[:3],
                quat_xyzw=fallback_pose[3:],
                cartesian=True,
                cartesian_max_step=self.cartesian_max_step,
                cartesian_fraction_threshold=self.cartesian_fraction_threshold
            )
            ee6.wait_until_executed()
            return True

        except Exception as e:
            self.get_logger().error(f"_retreat_to_fallback(): failed to retreat to fallback: {e}")
            return False

    # ---------------------------------------------------------------------------
    # ♦ set_gripper_position  – single‐shot gripper command + status read
    # ---------------------------------------------------------------------------
    def set_gripper_position(
            self,
            speed: int = 255,
            position: int = 255,
            force: int = 255,
            settling_time: float = 0.2
    ) -> int | None:
        """
        Command the Dobot gripper to *position* (0 = open … 255 = closed),
        wait for settling, then return the current gripper reading.

        Returns
        -------
        reading : int
            The gripper position read back after settling, or None on failure.
        """
        # Input validation and clamping
        position = max(0, min(255, int(position)))
        speed = max(0, min(255, int(speed)))
        force = max(0, min(255, int(force)))
        settling_time = max(0.1, float(settling_time))  # Minimum 100ms settling time

        try:
            # 1) Ensure services are available
            if not self._check_gripper_services():
                return None

            # 2) Send gripper command
            if not self._send_gripper_command(position, speed, force):
                return None

            # 3) Wait for settling
            time.sleep(settling_time)

            # 4) Read back gripper position
            return self._read_gripper_position(position)

        except Exception as e:
            self.get_logger().error(f"set_gripper_position(): Unexpected error: {e}")
            return None

    def _check_gripper_services(self) -> bool:
        """Check if gripper services are available."""
        if not self.set_gripper_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("_check_gripper_services(): SetGripperPosition service unavailable.")
            return False
        if not self.get_gripper_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("_check_gripper_services(): GetGripperPosition service unavailable.")
            return False
        return True

    def _send_gripper_command(self, position: int, speed: int, force: int) -> bool:
        """Send gripper command and wait for acceptance."""
        try:
            req = SetGripperPosition.Request()
            req.position = position
            req.speed = speed
            req.force = force
            
            future = self.set_gripper_cli.call_async(req)
            
            # Wait up to 2s for the command to be accepted
            start_time = time.monotonic()
            while not future.done() and (time.monotonic() - start_time) < 2.0:
                rclpy.spin_once(self, timeout_sec=0.01)

            if not future.done():
                self.get_logger().warn("_send_gripper_command(): command timed out.")
                return False
                
            return True
            
        except Exception as e:
            self.get_logger().error(f"_send_gripper_command(): Error: {e}")
            return False

    def _read_gripper_position(self, requested_position: int) -> int | None:
        """Read back gripper position after command."""
        try:
            req = GetGripperPosition.Request()
            req.index = 0
            future = self.get_gripper_cli.call_async(req)

            # Wait up to 2s for response
            start_time = time.monotonic()
            while not future.done() and (time.monotonic() - start_time) < 2.0:
                rclpy.spin_once(self, timeout_sec=0.01)

            if not future.done() or future.result() is None:
                self.get_logger().error("_read_gripper_position(): failed to read back gripper position.")
                return None

            reading = future.result().position
            self.get_logger().info(f"_read_gripper_position(): requested {requested_position}, read back {reading}")
            return reading
            
        except Exception as e:
            self.get_logger().error(f"_read_gripper_position(): Error: {e}")
            return None

    def enforce_rxry(self) -> bool:
        """
        Override Link6's Rx→90°, Ry→0° (keep current Rz) while freezing
        the world‐space position of portafilter_link to ±0.5 mm.
        Returns True on success.
        """
        import numpy as np
        import time
        from tf_transformations import euler_matrix, quaternion_from_matrix

        try:
            # Constants
            d_rel = np.array([0.0, 0.0, 0.2825])  # Fixed offset from Link6 origin → portafilter_link origin (m)
            tolerance = 0.002  # 0.5 mm in metres

            # 1) Get current Link6 pose
            current_pose = self._get_link6_pose_with_retries()
            if current_pose is None:
                return False

            tx_mm, ty_mm, tz_mm, rx_curr, ry_curr, rz_curr = current_pose

            # 2) Compute current portafilter world position
            p_pf_world = self._compute_portafilter_world_position(
                np.array([tx_mm, ty_mm, tz_mm]) * 1e-3,
                np.radians([rx_curr, ry_curr, rz_curr]),
                d_rel
            )

            # 3) Compute goal pose for Link6
            goal_pose = self._compute_link6_goal_pose(rz_curr, p_pf_world, d_rel)
            if goal_pose is None:
                return False

            # 4) Execute motion
            if not self._execute_enforce_motion(goal_pose):
                return False

            # 5) Verify portafilter position accuracy
            if not self._verify_portafilter_position(p_pf_world, d_rel, tolerance):
                return False

            self.get_logger().info("enforce_rxry(): Completed with SUCCESS.")
            return True

        except Exception as e:
            self.get_logger().error(f"enforce_rxry(): Unexpected error: {e}")
            return False

    def _get_link6_pose_with_retries(self, max_attempts: int = 3) -> tuple | None:
        """Get current Link6 pose with retry logic."""
        if not self.get_pose_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("_get_link6_pose_with_retries(): GetPose service unavailable.")
            return None

        req = GetPose.Request()
        req.user = 0
        req.tool = 0

        for attempt in range(1, max_attempts + 1):
            try:
                future = self.get_pose_cli.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                resp = future.result()
                
                if resp is not None and hasattr(resp, "pose"):
                    # Parse pose string
                    parts = resp.pose.strip("{}").split(",")
                    if len(parts) < 6:
                        self.get_logger().error("_get_link6_pose_with_retries(): Invalid pose format.")
                        return None
                    
                    pose_values = [float(p) for p in parts[:6]]
                    return tuple(pose_values)
                
            except Exception as e:
                self.get_logger().warn(f"_get_link6_pose_with_retries(): Attempt {attempt}/{max_attempts} failed: {e}")
            
            if attempt < max_attempts:
                time.sleep(0.5)

        self.get_logger().error("_get_link6_pose_with_retries(): Failed to retrieve pose after all attempts.")
        return None

    def _compute_portafilter_world_position(self, p_link6: np.ndarray, rads: np.ndarray, d_rel: np.ndarray) -> np.ndarray:
        """Compute world position of portafilter_link given Link6 position and orientation."""
        try:
            R6_curr = euler_matrix(*rads)[:3, :3]
            return p_link6 + R6_curr.dot(d_rel)
        except Exception as e:
            self.get_logger().error(f"_compute_portafilter_world_position(): Error: {e}")
            raise

    def _compute_link6_goal_pose(self, rz_curr: float, p_pf_world: np.ndarray, d_rel: np.ndarray) -> tuple | None:
        """Compute goal pose for Link6 to achieve desired orientation while keeping portafilter position fixed."""
        try:
            # Define desired Link6 orientation: Rx=90°, Ry=0°, keep Rz
            rx_target, ry_target = 90.0, 0.0
            rads_goal = np.radians([rx_target, ry_target, rz_curr])
            
            M_goal = euler_matrix(*rads_goal)
            R6_goal = M_goal[:3, :3]
            quat_goal = list(quaternion_from_matrix(M_goal))

            # Back-solve Link6 goal position so portafilter_link stays put
            p6_goal = p_pf_world - R6_goal.dot(d_rel)
            
            self.get_logger().info(f"_compute_link6_goal_pose(): Goal Link6 pos (m): {p6_goal.tolist()}")
            return p6_goal.tolist(), quat_goal
            
        except Exception as e:
            self.get_logger().error(f"_compute_link6_goal_pose(): Error: {e}")
            return None

    def _execute_enforce_motion(self, goal_pose: tuple) -> bool:
        """Execute the enforce motion using MoveIt2."""
        try:
            position, quaternion = goal_pose
            
            # Create temporary MoveIt2 instance for Link6
            temp_moveit2 = MoveIt2(
                node=self,
                joint_names=self.moveit2.joint_names,
                base_link_name=self.reference_frame,
                end_effector_name="Link6",
                group_name=GROUP_NAME,
            )
            temp_moveit2.planner_id = self.planner_id
            temp_moveit2.max_velocity = self.velocity_scaling
            temp_moveit2.max_acceleration = self.acceleration_scaling
            temp_moveit2.cartesian_jump_threshold = self.cartesian_jump_threshold
            temp_moveit2.cartesian_avoid_collisions = self.cartesian_avoid_collisions

            self.get_logger().info("_execute_enforce_motion(): Calling MoveIt2.move_to_pose()...")
            temp_moveit2.move_to_pose(
                position=position,
                quat_xyzw=quaternion,
                cartesian=self.cartesian,
                cartesian_max_step=self.cartesian_max_step,
                cartesian_fraction_threshold=self.cartesian_fraction_threshold,
            )
            temp_moveit2.wait_until_executed()
            
            state = temp_moveit2.query_state()
            if state != MoveIt2State.IDLE:
                self.get_logger().warn(f"_execute_enforce_motion(): Motion ended with state: {state}")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"_execute_enforce_motion(): Error: {e}")
            return False

    def _verify_portafilter_position(self, p_pf_world: np.ndarray, d_rel: np.ndarray, tolerance: float, max_attempts: int = 10) -> bool:
        """Verify that portafilter_link stayed within tolerance."""
        try:
            for attempt in range(max_attempts):
                # Re-read Link6 pose
                current_pose = self._get_link6_pose_with_retries()
                if current_pose is None:
                    return False

                tx2, ty2, tz2, rx2, ry2, rz2 = current_pose
                p6_new = np.array([tx2, ty2, tz2]) * 1e-3
                R6_new = euler_matrix(*np.radians([rx2, ry2, rz2]))[:3, :3]
                p_pf_new = p6_new + R6_new.dot(d_rel)

                error = np.linalg.norm(p_pf_new - p_pf_world)
                if error <= tolerance:
                    self.get_logger().info(f"_verify_portafilter_position(): Position verified (error: {error*1e3:.2f} mm)")
                    return True

                self.get_logger().warn(
                    f"_verify_portafilter_position(): portafilter moved {error*1e3:.2f} mm (>0.5 mm), "
                    f"attempt {attempt + 1}/{max_attempts}"
                )
                
                if attempt < max_attempts - 1:
                    time.sleep(0.2)

            self.get_logger().error("_verify_portafilter_position(): Failed to verify position after all attempts")
            return False
            
        except Exception as e:
            self.get_logger().error(f"_verify_portafilter_position(): Error: {e}")
            return False

    def move_portafilter_arc(self, angle_deg: float) -> bool:
        """
        Rotate the portafilter_link by the specified angle about its local Y axis.
        
        Args:
            angle_deg: Rotation angle in degrees (positive = counterclockwise)
            
        Returns:
            bool: True if successful, False otherwise
        """
        # Input validation
        if not isinstance(angle_deg, (int, float)):
            self.get_logger().error("move_portafilter_arc(): angle_deg must be a number")
            return False
        
        if abs(angle_deg) > 180:
            self.get_logger().warn(f"move_portafilter_arc(): Large angle {angle_deg}° may be unsafe")
        
        self.get_logger().info(f"move_portafilter_arc(): Rotating portafilter_link by {angle_deg}°")

        try:
            # 1) Get current portafilter pose
            current_pose = self._get_portafilter_current_pose()
            if current_pose is None:
                return False

            # 2) Compute new orientation
            new_pose = self._compute_arc_pose(current_pose, angle_deg)
            if new_pose is None:
                return False

            # 3) Execute arc motion
            if not self._execute_arc_motion(new_pose, angle_deg):
                return False

            # 4) Verify motion completion
            if not self._verify_arc_completion():
                return False

            self.get_logger().info("move_portafilter_arc(): Function completed successfully.")
            return True

        except Exception as e:
            self.get_logger().error(f"move_portafilter_arc(): Unexpected error: {e}")
            return False

    def _get_portafilter_current_pose(self) -> tuple | None:
        """Get current portafilter pose from TF."""
        try:
            pf_tf_stamped = self.tf_buffer.lookup_transform(
                self.reference_frame,
                "portafilter_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=5.0)
            )
            current_tf = get_transform_list(pf_tf_stamped)
            
            position = np.array(current_tf[:3])
            current_quat = np.array(current_tf[3:])
            
            return position, current_quat
            
        except Exception as e:
            self.get_logger().error(f"_get_portafilter_current_pose(): Failed to get portafilter_link transform: {e}")
            return None

    def _compute_arc_pose(self, current_pose: tuple, angle_deg: float) -> tuple | None:
        """Compute new pose after rotating about local Y axis."""
        try:
            position, current_quat = current_pose
            
            # Compute new orientation by rotating about local Y axis
            R_current = tf_transformations.quaternion_matrix(current_quat)[0:3, 0:3]
            rotation_axis = R_current[:, 1]  # Local Y axis
            theta = math.radians(angle_deg)
            relative_quat = tf_transformations.quaternion_about_axis(theta, rotation_axis)
            new_quat = tf_transformations.quaternion_multiply(relative_quat, current_quat)
            new_pos = position.tolist()

            self.get_logger().info(
                f"_compute_arc_pose(): New orientation (xyzw): {new_quat}"
            )
            
            return new_pos, new_quat
            
        except Exception as e:
            self.get_logger().error(f"_compute_arc_pose(): Error computing arc pose: {e}")
            return None

    def _execute_arc_motion(self, new_pose: tuple, angle_deg: float) -> bool:
        """Execute the arc motion using MoveIt2."""
        try:
            new_pos, new_quat = new_pose
            
            # Create MoveIt2 instance for portafilter
            pf_moveit2 = MoveIt2(
                node=self,
                joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                base_link_name=self.reference_frame,
                end_effector_name="portafilter_link",
                group_name="portafilter_center",
            )
            pf_moveit2.planner_id = self.planner_id
            pf_moveit2.max_velocity = self.velocity_scaling
            pf_moveit2.max_acceleration = self.acceleration_scaling
            pf_moveit2.cartesian_jump_threshold = self.cartesian_jump_threshold
            pf_moveit2.cartesian_avoid_collisions = self.cartesian_avoid_collisions

            # Execute the arc motion
            pf_moveit2.move_to_pose(
                position=new_pos,
                quat_xyzw=new_quat,
                cartesian=self.cartesian,
                cartesian_max_step=self.cartesian_max_step,
                cartesian_fraction_threshold=self.cartesian_fraction_threshold,
            )
            pf_moveit2.wait_until_executed()
            
            state = pf_moveit2.query_state()
            if state == MoveIt2State.IDLE:
                self.get_logger().info("_execute_arc_motion(): Rotation motion executed successfully.")
                return True
            else:
                self.get_logger().warn(f"_execute_arc_motion(): Rotation motion ended with state: {state}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"_execute_arc_motion(): Error: {e}")
            return False

    def _verify_arc_completion(self, tolerance_deg: float = 1.0, batch_size: int = 10, max_attempts: int = 5) -> bool:
        """Verify that the arc motion has completed with stable joint positions."""
        try:
            sleep_between = 0.1  # seconds

            for overall_attempt in range(max_attempts):
                # Collect one batch of joint samples
                joint_samples = []
                for _ in range(batch_size):
                    js = self.wait_for_joint_state("/joint_states_robot", timeout_sec=1.0)
                    if js and len(js.position) >= 6:
                        angles_deg = [math.degrees(j) for j in js.position[0:6]]
                        joint_samples.append(angles_deg)
                    time.sleep(sleep_between)

                # Compute per-joint spread across this batch
                if not joint_samples:
                    self.get_logger().warn("_verify_arc_completion(): No joint samples collected")
                    continue
                    
                spreads = [max(vals) - min(vals) for vals in zip(*joint_samples)]

                # Check if spread is within tolerance
                if all(spread < tolerance_deg for spread in spreads):
                    self.get_logger().info("_verify_arc_completion(): Joint positions stable")
                    return True

                # Try fallback verification
                if joint_samples:
                    last_sample = joint_samples[-1]
                    if self.verify_joint_positions(last_sample, tolerance_deg):
                        self.get_logger().info("_verify_arc_completion(): Fallback verification passed")
                        return True

                self.get_logger().warn(
                    f"_verify_arc_completion(): Joint spread too large "
                    f"(spreads: {spreads}), attempt {overall_attempt + 1}/{max_attempts}"
                )

            self.get_logger().error("_verify_arc_completion(): Failed to achieve stable joint positions")
            return False
            
        except Exception as e:
            self.get_logger().error(f"_verify_arc_completion(): Error: {e}")
            return False

    def moveEE(
        self,
        offset_x: float,
        offset_y: float,
        offset_z: float,
        offset_rx: float,
        offset_ry: float,
        offset_rz: float,
        EE_link: str = "Link6",
    ) -> bool:
        """
        Perform a linear (Cartesian) move of the specified end effector link
        by translation offsets (mm) and rotation offsets (deg), then verify
        arrival via wait_for_servo_ready().

        Args:
            offset_x, offset_y, offset_z: translation offsets in millimetres
            offset_rx, offset_ry, offset_rz: rotation offsets in degrees
            EE_link: the end‐effector frame to plan for (default "Link6")
            
        Returns:
            bool: True if successful, False otherwise
        """
        # Input validation
        if not isinstance(EE_link, str) or not EE_link:
            self.get_logger().error("moveEE(): EE_link must be a non-empty string")
            return False
        
        # Validate numeric inputs
        try:
            offsets = [float(x) for x in [offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz]]
        except (TypeError, ValueError):
            self.get_logger().error("moveEE(): All offset parameters must be numeric")
            return False
        
        self.get_logger().info(
            f"moveEE(): Starting linear move using '{EE_link}' with translation offsets (mm): "
            f"({offset_x}, {offset_y}, {offset_z}) and rotation offsets (deg): "
            f"({offset_rx}, {offset_ry}, {offset_rz})."
        )

        try:
            # 1) Get current EE transform
            current_tf = self._get_ee_transform_with_retries(EE_link)
            if current_tf is None:
                return False

            # 2) Compute goal pose
            goal_pose = self._compute_ee_goal_pose(current_tf, offsets)
            if goal_pose is None:
                return False

            # 3) Execute motion
            if not self._execute_ee_motion(goal_pose, EE_link):
                return False

            self.get_logger().info("moveEE(): Function completed successfully.")
            return True

        except Exception as e:
            self.get_logger().error(f"moveEE(): Unexpected error: {e}")
            return False

    def _get_ee_transform_with_retries(self, ee_link: str, max_attempts: int = 3) -> list | None:
        """Get current end effector transform with retry logic."""
        from rclpy.time import Time
        from rclpy.duration import Duration
        
        for attempt in range(1, max_attempts + 1):
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    self.reference_frame,
                    ee_link,
                    Time(),
                    timeout=Duration(seconds=5.0)
                )
                current_tf = get_transform_list(tf_stamped)
                self.get_logger().info(f"_get_ee_transform_with_retries(): Current pose obtained: {current_tf}")
                return current_tf
                
            except Exception as e:
                self.get_logger().warn(
                    f"_get_ee_transform_with_retries(): Attempt {attempt}/{max_attempts} failed for '{ee_link}': {e}"
                )
                if attempt < max_attempts:
                    time.sleep(0.1)

        self.get_logger().error(f"_get_ee_transform_with_retries(): Failed to retrieve transform after {max_attempts} attempts for '{ee_link}'")
        return None

    def _compute_ee_goal_pose(self, current_tf: list, offsets: list) -> tuple | None:
        """Compute goal pose by applying translation and rotation offsets."""
        try:
            import numpy as np
            import math
            
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz = offsets
            
            # Convert offsets
            t_offset = np.array([offset_x, offset_y, offset_z]) / 1000.0  # mm to m
            r_offset = np.radians([offset_rx, offset_ry, offset_rz])  # deg to rad

            # Current pose components
            current_translation = np.array(current_tf[:3])
            current_quat = np.array(current_tf[3:])
            R_current = tf_transformations.quaternion_matrix(current_quat)[0:3, 0:3]

            # Compute goal translation
            goal_translation = (current_translation + t_offset).tolist()

            # Compute goal rotation
            R_delta = tf_transformations.euler_matrix(*r_offset)[0:3, 0:3]
            R_goal = R_current.dot(R_delta)
            goal_matrix = np.eye(4)
            goal_matrix[0:3, 0:3] = R_goal
            goal_quat = tf_transformations.quaternion_from_matrix(goal_matrix)

            self.get_logger().info(
                f"_compute_ee_goal_pose(): Computed goal pose:\n"
                f"  Position: {goal_translation}\n"
                f"  Orientation (xyzw): {goal_quat}"
            )
            
            return goal_translation, goal_quat
            
        except Exception as e:
            self.get_logger().error(f"_compute_ee_goal_pose(): Error computing goal pose: {e}")
            return None

    def _execute_ee_motion(self, goal_pose: tuple, ee_link: str) -> bool:
        """Execute the end effector motion using MoveIt2."""
        try:
            goal_translation, goal_quat = goal_pose
            
            # Create temporary MoveIt2 instance
            temp_moveit2 = MoveIt2(
                node=self,
                joint_names=self.moveit2.joint_names,
                base_link_name=self.reference_frame,
                end_effector_name=ee_link,
                group_name=GROUP_NAME,
            )
            temp_moveit2.planner_id = self.planner_id
            temp_moveit2.max_velocity = self.velocity_scaling
            temp_moveit2.max_acceleration = self.acceleration_scaling
            temp_moveit2.cartesian_jump_threshold = self.cartesian_jump_threshold
            temp_moveit2.cartesian_avoid_collisions = self.cartesian_avoid_collisions

            # Execute motion
            temp_moveit2.move_to_pose(
                position=goal_translation,
                quat_xyzw=goal_quat,
                cartesian=self.cartesian,
                cartesian_max_step=self.cartesian_max_step,
                cartesian_fraction_threshold=self.cartesian_fraction_threshold
            )

            # Verify arrival using wait_for_servo_ready()
            temp_moveit2.wait_until_executed()
            timeout = 15.0
            start_time = time.monotonic()
            while not self.wait_for_servo_ready(timeout=timeout):
                if time.monotonic() - start_time > timeout:
                    self.get_logger().error("_execute_ee_motion(): Timeout waiting for servo ready")
                    return False
                self.get_logger().warn("_execute_ee_motion(): not arrived, rechecking...")
                time.sleep(0.2)

            return True
            
        except Exception as e:
            self.get_logger().error(f"_execute_ee_motion(): Error: {e}")
            return False

    def moveJ_deg(
        self,
        angle1: float,
        angle2: float,
        angle3: float,
        angle4: float,
        angle5: float,
        angle6: float,
        velocity_scaling: float = 1.0,
        acceleration_scaling: float = 1.0,
    ) -> bool:
        """
        Move robot joints relatively from current position (degrees), with velocity and acceleration scaling,
        but only if the current joint state is stable. Returns True on success, False on any failure.
        """
        # Input validation
        try:
            rel_angles = [float(x) for x in [angle1, angle2, angle3, angle4, angle5, angle6]]
            velocity_scaling = max(0.1, min(1.0, float(velocity_scaling)))
            acceleration_scaling = max(0.1, min(1.0, float(acceleration_scaling)))
        except (TypeError, ValueError):
            self.get_logger().error("moveJ_deg(): All parameters must be numeric")
            return False

        self.get_logger().info(
            f"moveJ_deg(): Rel angles (deg)={rel_angles}, "
            f"vel_scale={velocity_scaling}, acc_scale={acceleration_scaling}"
        )

        # Backup & apply scalings
        with self._joint_scaling_context(velocity_scaling, acceleration_scaling):
            try:
                # Get stable current joint state
                stable_joints_rad = self._get_stable_joint_state()
                if stable_joints_rad is None:
                    return False

                # Compute goal configuration
                rel_rad = [math.radians(a) for a in rel_angles]
                new_joints_rad = [c + r for c, r in zip(stable_joints_rad, rel_rad)]

                # Validate joint limits
                if not self._validate_joint_limits(new_joints_rad):
                    return False

                # Execute motion
                return self._execute_joint_motion(new_joints_rad)

            except Exception as e:
                self.get_logger().error(f"moveJ_deg(): Unexpected error: {e}")
                return False

    def gotoJ_deg(
        self,
        angle1: float,
        angle2: float,
        angle3: float,
        angle4: float,
        angle5: float,
        angle6: float,
        velocity_scaling: float = 1.0,
        acceleration_scaling: float = 1.0,
    ) -> bool:
        """
        Move robot to absolute joint configuration (degrees), enforcing joint limits,
        then verify arrival via wait_for_servo_ready().

        Args:
            angle1–angle6: target joint angles in degrees (absolute)
            velocity_scaling: velocity scaling factor (0.1 to 1.0)
            acceleration_scaling: acceleration scaling factor (0.1 to 1.0)
        Returns:
            True on success, False on any failure.
        """
        # Input validation
        try:
            target_angles = [float(x) for x in [angle1, angle2, angle3, angle4, angle5, angle6]]
            velocity_scaling = max(0.1, min(1.0, float(velocity_scaling)))
            acceleration_scaling = max(0.1, min(1.0, float(acceleration_scaling)))
        except (TypeError, ValueError):
            self.get_logger().error("gotoJ_deg(): All parameters must be numeric")
            return False

        self.get_logger().info(
            f"gotoJ_deg(): Target absolute joint angles (deg): {target_angles}, "
            f"vel_scale={velocity_scaling}, acc_scale={acceleration_scaling}"
        )

        # Backup & apply scalings
        with self._joint_scaling_context(velocity_scaling, acceleration_scaling):
            try:
                # Convert to radians
                new_joints_rad = [math.radians(a) for a in target_angles]

                # Validate joint limits
                if not self._validate_joint_limits(new_joints_rad):
                    return False

                # Execute motion
                return self._execute_joint_motion(new_joints_rad)

            except Exception as e:
                self.get_logger().error(f"gotoJ_deg(): Unexpected error: {e}")
                return False

    def _joint_scaling_context(self, velocity_scaling: float, acceleration_scaling: float):
        """Context manager for joint scaling backup and restore."""
        from contextlib import contextmanager
        
        @contextmanager
        def scaling_context():
            # Backup original scalings
            orig_vel = self.moveit2.max_velocity
            orig_acc = self.moveit2.max_acceleration
            
            try:
                # Apply new scalings
                self.moveit2.max_velocity = velocity_scaling
                self.moveit2.max_acceleration = acceleration_scaling
                yield
            finally:
                # Restore original scalings
                self.moveit2.max_velocity = orig_vel
                self.moveit2.max_acceleration = orig_acc
        
        return scaling_context()

    def _get_stable_joint_state(self, timeout: float = 5.0, window_size: int = 5, tolerance_deg: float = 1.0) -> list | None:
        """Get stable joint state using sliding window approach."""
        import time
        import math
        
        window = []
        start_time = time.time()
        
        while True:
            js = self.wait_for_joint_state("/joint_states_robot", timeout_sec=0.2)
            if js and len(js.position) >= 6:
                current_rad = list(js.position[:6])
                current_deg = [math.degrees(x) for x in current_rad]
                window.append((current_rad, current_deg))
                
                if len(window) > window_size:
                    window.pop(0)

                if len(window) == window_size:
                    # Check stability across the window
                    spreads = [
                        max(col) - min(col)
                        for col in zip(*(w[1] for w in window))
                    ]
                    if all(s <= tolerance_deg for s in spreads):
                        return window[-1][0]  # Return last stable joint state in radians

            if time.time() - start_time > timeout:
                self.get_logger().error("_get_stable_joint_state(): joint state stability timeout")
                return None

    def _validate_joint_limits(self, joints_rad: list) -> bool:
        """Validate that joint angles are within limits."""
        import math
        
        joint_limits = [
            (-6.2, 6.2),     # joint1
            (-3.14, 3.14),   # joint2
            (-2.79, 0.0),    # joint3
            (-6.28, 6.28),   # joint4
            (-6.28, 6.28),   # joint5
            (-6.28, 6.28),   # joint6
        ]
        
        for idx, (val, (low, high)) in enumerate(zip(joints_rad, joint_limits), start=1):
            if not (low <= val <= high):
                self.get_logger().error(
                    f"_validate_joint_limits(): joint{idx} target {math.degrees(val):.3f}° "
                    f"outside limits [{math.degrees(low):.1f}°, {math.degrees(high):.1f}°]"
                )
                return False
        return True

    def _execute_joint_motion(self, joints_rad: list) -> bool:
        """Execute joint motion and verify completion."""
        try:
            # Execute the motion
            self.moveit2.move_to_configuration(joints_rad)

            # Verify arrival via servo‐ready polling
            self.moveit2.wait_until_executed()
            timeout = 15.0
            start_time = time.monotonic()
            while not self.wait_for_servo_ready(timeout=timeout):
                if time.monotonic() - start_time > timeout:
                    self.get_logger().error("_execute_joint_motion(): Timeout waiting for servo ready")
                    return False
                self.get_logger().warn("_execute_joint_motion(): not arrived, rechecking...")
                time.sleep(0.2)

            self.get_logger().info("_execute_joint_motion(): Motion completed successfully.")
            return True
            
        except Exception as e:
            self.get_logger().error(f"_execute_joint_motion(): Error: {e}")
            return False

    def current_angles(self) -> tuple[float, ...] | None:
        """
        Read the current joint angles immediately and return them in degrees.
        
        This function is designed to be used with run_skill to save/restore robot positions:
        
        Example usage:
            # Save current position
            saved_angles = run_skill("current_angles")
            
            # Do some movements...
            run_skill("moveJ_deg", 10, 0, 0, 0, 0, 0)
            
            # Return to saved position
            if saved_angles:
                run_skill("gotoJ_deg", *saved_angles)
        
        Returns:
            tuple: (angle1, angle2, angle3, angle4, angle5, angle6) in degrees, or None if failed
        """
        try:
            # Wait for servo to be ready (movement complete)
            if not self._wait_for_servo_ready_with_timeout():
                return None
                
            # Read current joint state
            joint_state_msg = self.wait_for_joint_state("/joint_states_robot", timeout_sec=2.0)
            
            if joint_state_msg is None:
                self.get_logger().error("current_angles(): Failed to read joint state")
                return None
                
            if not hasattr(joint_state_msg, 'position') or len(joint_state_msg.position) < 6:
                self.get_logger().error("current_angles(): Insufficient joint data")
                return None
            
            # Convert from radians to degrees - ensure exactly 6 values
            joint_positions = list(joint_state_msg.position[:6])
            angles_deg = tuple(math.degrees(float(angle)) for angle in joint_positions)
            
            self.get_logger().info(f"current_angles(): Current joint angles (deg): {angles_deg}")
            return angles_deg
            
        except (TypeError, ValueError) as e:
            self.get_logger().error(f"current_angles(): Error converting angles: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"current_angles(): Unexpected error: {e}")
            return None

    def _wait_for_servo_ready_with_timeout(self, timeout: float = 15.0) -> bool:
        """Wait for servo to be ready with timeout handling."""
        start_time = time.monotonic()
        while not self.wait_for_servo_ready(timeout=timeout):
            if time.monotonic() - start_time > timeout:
                self.get_logger().error("_wait_for_servo_ready_with_timeout(): Timeout waiting for servo ready")
                return False
            self.get_logger().warn("_wait_for_servo_ready_with_timeout(): Waiting for servo to be ready...")
            time.sleep(0.1)
        return True

    def gotoEE(self, abs_x_mm: float, abs_y_mm: float, abs_z_mm: float,
               abs_rx_deg: float, abs_ry_deg: float, abs_rz_deg: float) -> bool:
        """
        Move the end-effector (Link6) to an **absolute** pose expressed in the
        base_link frame, then verify arrival via wait_for_servo_ready().

        Args:
            abs_x_mm, abs_y_mm, abs_z_mm: absolute position in millimetres
            abs_rx_deg, abs_ry_deg, abs_rz_deg: absolute orientation (XYZ-Euler) in degrees
            
        Returns:
            bool: True if successful, False otherwise
        """
        # Input validation
        try:
            coords = [float(x) for x in [abs_x_mm, abs_y_mm, abs_z_mm, abs_rx_deg, abs_ry_deg, abs_rz_deg]]
        except (TypeError, ValueError):
            self.get_logger().error("gotoEE(): All parameters must be numeric")
            return False
        
        abs_x_mm, abs_y_mm, abs_z_mm, abs_rx_deg, abs_ry_deg, abs_rz_deg = coords
        
        self.get_logger().info(
            f"gotoEE(): Target absolute pose – "
            f"Position (mm): ({abs_x_mm}, {abs_y_mm}, {abs_z_mm}), "
            f"Orientation (deg): ({abs_rx_deg}, {abs_ry_deg}, {abs_rz_deg})"
        )

        try:
            # 1) Build goal pose
            goal_pose = self._compute_absolute_goal_pose(
                abs_x_mm, abs_y_mm, abs_z_mm, abs_rx_deg, abs_ry_deg, abs_rz_deg
            )
            if goal_pose is None:
                return False

            # 2) Execute motion
            if not self._execute_absolute_ee_motion(goal_pose):
                return False

            self.get_logger().info("gotoEE(): Function completed successfully.")
            return True

        except Exception as e:
            self.get_logger().error(f"gotoEE(): Unexpected error: {e}")
            return False

    def _compute_absolute_goal_pose(self, x_mm: float, y_mm: float, z_mm: float,
                                   rx_deg: float, ry_deg: float, rz_deg: float) -> tuple | None:
        """Compute absolute goal pose from millimeters and degrees."""
        try:
            import math
            
            # Build goal pose in metres + quaternion
            goal_translation = [coord / 1000.0 for coord in (x_mm, y_mm, z_mm)]
            r_radians = [math.radians(a) for a in (rx_deg, ry_deg, rz_deg)]
            R_goal = tf_transformations.euler_matrix(*r_radians)
            goal_quat = tf_transformations.quaternion_from_matrix(R_goal)

            self.get_logger().info(f"_compute_absolute_goal_pose(): Goal pose (m, xyzw): {goal_translation}  {goal_quat}")
            return goal_translation, goal_quat
            
        except Exception as e:
            self.get_logger().error(f"_compute_absolute_goal_pose(): Error: {e}")
            return None

    def _execute_absolute_ee_motion(self, goal_pose: tuple) -> bool:
        """Execute absolute end effector motion using MoveIt2."""
        try:
            goal_translation, goal_quat = goal_pose
            
            # Plan & execute via MoveIt2 for Link6
            temp_moveit2 = MoveIt2(
                node=self,
                joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                base_link_name=self.reference_frame,
                end_effector_name="Link6",
                group_name=GROUP_NAME,
            )
            temp_moveit2.planner_id = self.planner_id
            temp_moveit2.max_velocity = self.velocity_scaling
            temp_moveit2.max_acceleration = self.acceleration_scaling
            temp_moveit2.cartesian_jump_threshold = self.cartesian_jump_threshold
            temp_moveit2.cartesian_avoid_collisions = self.cartesian_avoid_collisions

            temp_moveit2.move_to_pose(
                position=goal_translation,
                quat_xyzw=goal_quat,
                cartesian=self.cartesian,
                cartesian_max_step=self.cartesian_max_step,
                cartesian_fraction_threshold=self.cartesian_fraction_threshold
            )

            # Verify arrival via servo‐ready polling
            temp_moveit2.wait_until_executed()
            timeout = 15.0
            start_time = time.monotonic()
            while not self.wait_for_servo_ready(timeout=timeout):
                if time.monotonic() - start_time > timeout:
                    self.get_logger().error("_execute_absolute_ee_motion(): Timeout waiting for servo ready")
                    return False
                self.get_logger().warn("_execute_absolute_ee_motion(): not arrived, rechecking…")
                time.sleep(0.2)

            return True
            
        except Exception as e:
            self.get_logger().error(f"_execute_absolute_ee_motion(): Error: {e}")
            return False

    def toggle_drag_mode(self, max_attempts: int = 10, retry_pause: float = 0.25) -> bool:
        """
        Alternate StartDrag and StopDrag until one succeeds (res == 0).
        Returns True on success, False if both calls fail after max_attempts each.
        """
        from dobot_msgs_v3.srv import StartDrag, StopDrag

        # lazy‐create both clients
        self.start_drag_cli = getattr(
            self, 'start_drag_cli',
            self.create_client(StartDrag, '/dobot_bringup_v3/srv/StartDrag')
        )
        self.stop_drag_cli = getattr(
            self, 'stop_drag_cli',
            self.create_client(StopDrag, '/dobot_bringup_v3/srv/StopDrag')
        )

        # begin by trying to enable drag mode
        use_start = True

        for attempt in range(1, max_attempts + 1):
            cli = self.start_drag_cli if use_start else self.stop_drag_cli
            srv_name = "StartDrag" if use_start else "StopDrag"

            # wait for service
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(
                    f"toggle_drag_mode: {srv_name} service unavailable (attempt {attempt}/{max_attempts})"
                )
                # flip for next attempt
                use_start = not use_start
                time.sleep(retry_pause)
                continue

            # call and wait
            fut = cli.call_async(StartDrag.Request() if use_start else StopDrag.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)

            if fut.done() and fut.result() is not None:
                res = getattr(fut.result(), 'res', None)
                if res == 0:
                    self.get_logger().info(f"toggle_drag_mode: {srv_name} succeeded")
                    return True
                else:
                    self.get_logger().warn(
                        f"toggle_drag_mode: {srv_name} returned res={res}, switching to "
                        f"{'StopDrag' if use_start else 'StartDrag'}"
                    )
            else:
                self.get_logger().warn(
                    f"toggle_drag_mode: {srv_name} call timed out or failed, switching to "
                    f"{'StopDrag' if use_start else 'StartDrag'}"
                )

            # flip for next attempt
            use_start = not use_start
            time.sleep(retry_pause)

        self.get_logger().error(
            "toggle_drag_mode: Exhausted attempts without success"
        )
        return False

    def set_DO(self, index: int, status: int) -> bool:
        """
        Execute a digital output on the Dobot via the DOExecute service.

        Parameters
        ----------
        index : int
            Digital output channel index to toggle.
        status : int
            Desired status (0 = off, 1 = on).

        Returns
        -------
        bool
            True if the service call succeeded (res == 0), False otherwise.
        """
        from dobot_msgs_v3.srv import DOExecute
        import rclpy, time

        # lazy‐create DOExecute client
        self.doexec_cli = getattr(
            self,
            'doexec_cli',
            self.create_client(DOExecute, '/dobot_bringup_v3/srv/DOExecute')
        )

        # build request
        req = DOExecute.Request()
        req.index = index
        req.status = status

        retry_pause = 0.25
        max_attempts = 10

        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(f"set_DO: DOExecute attempt {attempt}/{max_attempts}")

            if not self.doexec_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("set_DO: DOExecute service unavailable")
                return False

            fut = self.doexec_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)

            if not fut.done() or fut.result() is None:
                self.get_logger().error("set_DO: DOExecute call timed out")
                return False  # do not retry on timeout

            resp = fut.result()
            res_code = getattr(resp, 'res', 1)
            if res_code == 0:
                self.get_logger().info("set_DO: DOExecute succeeded")
                break
            else:
                self.get_logger().warn(
                    f"set_DO: DOExecute returned error code {res_code}, retrying…"
                )
                time.sleep(retry_pause)
        else:
            self.get_logger().error("set_DO: DOExecute failed after maximum retries")
            return False

        return True

    # ---------------------------------------------------------------------------
    # 1)  get_machine_position  – sample & save machine pose, then broadcast *_test
    # ---------------------------------------------------------------------------
    def get_machine_position(self,
                            target_tf: str,
                            sample_interval: float = 0.1) -> dict | None:
        """
        Average 30 fresh base_link→target_tf samples, store under 'machines:' in
        pose_data_memory.yaml, and broadcast a static TF '<target_tf>_test'.
        Returns the stored dict on success, otherwise None.
        """
        import yaml, numpy as np, time, os
        from geometry_msgs.msg import TransformStamped
        import tf2_ros
        from ament_index_python.packages import get_package_share_directory

        REQUIRED_SAMPLES, OUTLIER_K = 60, 1.5
        samples = []

        while len(samples) < REQUIRED_SAMPLES:
            tfm, _ = self.get_tf(target_tf, max_retries=0, sleep_time=0.0)
            if tfm:
                if hasattr(tfm, "transform"):                # TransformStamped
                    t, r = tfm.transform.translation, tfm.transform.rotation
                    samples.append([t.x, t.y, t.z, r.x, r.y, r.z, r.w])
                elif len(tfm) == 7:                          # raw [tx,ty,tz,qx,qy,qz,qw]
                    samples.append(list(tfm))
            time.sleep(sample_interval)

        arr    = np.asarray(samples)
        med_t  = np.median(arr[:, 0:3], axis=0)
        dists  = np.linalg.norm(arr[:, 0:3] - med_t, axis=1)
        q1, q3 = np.percentile(dists, [25, 75])
        mask   = (dists >= q1 - OUTLIER_K*(q3-q1)) & (dists <= q3 + OUTLIER_K*(q3-q1))
        kept   = arr[mask]

        tx, ty, tz = kept[:, 0:3].mean(axis=0)

        # quaternion average (Markley / eigen method)
        M = np.zeros((4, 4))
        for row in kept:
            w, x, y, z = row[6], row[3], row[4], row[5]
            v = np.array([w, x, y, z]).reshape(4, 1)
            M += v @ v.T
        vals, vecs = np.linalg.eig(M / kept.shape[0])
        q_avg      = vecs[:, vals.argmax()] / np.linalg.norm(vecs[:, vals.argmax()])
        qw, qx, qy, qz = q_avg.tolist()

        # ── write YAML ──────────────────────────────────────────────────────────
        pkg_share   = get_package_share_directory("pickn_place")
        mem_file    = os.path.join(pkg_share, "machine_pose_data_memory.yaml")

        try:
            with open(mem_file) as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            data = {}

        data.setdefault("machines", {})[target_tf] = {
            "Time":        f"{time.time()}",
            "translation": {"x": float(tx), "y": float(ty), "z": float(tz)},
            "rotation":    {"x": float(qx), "y": float(qy), "z": float(qz), "w": float(qw)},
        }

        with open(mem_file, "w") as f:
            yaml.safe_dump(data, f)

        # ── broadcast <target_tf>_test ──────────────────────────────────────────
        if not hasattr(self, "static_broadcaster"):
            self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        tf_msg = TransformStamped()
        tf_msg.header.stamp    = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "base_link"
        tf_msg.child_frame_id  = f"{target_tf}_test"
        tf_msg.transform.translation.x = tx
        tf_msg.transform.translation.y = ty
        tf_msg.transform.translation.z = tz
        tf_msg.transform.rotation.x    = qx
        tf_msg.transform.rotation.y    = qy
        tf_msg.transform.rotation.z    = qz
        tf_msg.transform.rotation.w    = qw
        self.static_broadcaster.sendTransform(tf_msg)

        self.get_logger().info(f"get_machine_position: stored pose & broadcast '{target_tf}_test'")
        return data["machines"][target_tf]

    # ---------------------------------------------------------------------------
    # 2)  approach_machine  – move to approach pose & broadcast *_approach_test
    # ---------------------------------------------------------------------------
    def approach_machine(self, machine_name: str, point_name: str, cartesian_override: bool = True, motion_type: str = "cartesian") -> bool:
        """
        Move the portafilter_link to the 'approach_pose' of <machine_name>/<point_name>.
        Also broadcasts a static TF '<machine_name>_<point_name>_approach_test'
        for the computed goal pose, then verifies execution via servo readiness.
        
        Args:
            machine_name: Name of the machine
            point_name: Name of the point on the machine
            cartesian_override: Use Cartesian planning (only applies when motion_type="cartesian")
            motion_type: "cartesian" for Cartesian motion (default) or "joint" for joint motion
        """
        try:
            # ── load files ───────────────────────────────────────────────────────
            pkg_share   = get_package_share_directory("pickn_place")
            mem_file    = os.path.join(pkg_share, "machine_pose_data_memory.yaml")
            off_file    = os.path.join(pkg_share, "machine_offset_points.yaml")

            machines = (yaml.safe_load(open(mem_file)) or {}).get("machines", {})
            offsets  = (yaml.safe_load(open(off_file)) or {}).get(machine_name, {})

            if machine_name not in machines:
                self.get_logger().error("approach_machine: machine pose missing.")
                return False
            if "approach_pose" not in offsets.get(point_name, {}):
                self.get_logger().error("approach_machine: approach_pose not found.")
                return False

            # Build base transform
            base_t = machines[machine_name]["translation"]
            base_q = machines[machine_name]["rotation"]
            M_base        = tf_transformations.quaternion_matrix([base_q["x"], base_q["y"],
                                                                base_q["z"], base_q["w"]])
            M_base[0:3,3] = [base_t["x"], base_t["y"], base_t["z"]]

            # Load offset
            ap      = offsets[point_name]["approach_pose"]
            tcp_link = ap.get("TCP", DEFAULT_TCP_LINK)
            off_t   = ap["translation"]
            off_q   = ap["rotation"]
            M_off        = tf_transformations.quaternion_matrix([off_q["x"], off_q["y"],
                                                                off_q["z"], off_q["w"]])
            M_off[0:3,3] = [off_t["x"], off_t["y"], off_t["z"]]

            # Compute goal
            M_goal   = M_base.dot(M_off)
            goal_pos = M_goal[0:3, 3].tolist()
            goal_quat= tf_transformations.quaternion_from_matrix(M_goal)

            # Broadcast test TF
            from geometry_msgs.msg import TransformStamped
            import tf2_ros
            if not hasattr(self, "static_broadcaster"):
                self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
            tf_msg = TransformStamped()
            tf_msg.header.stamp    = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = "base_link"
            tf_msg.child_frame_id  = f"{machine_name}_{point_name}_approach_test"
            tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z = goal_pos
            tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, \
            tf_msg.transform.rotation.z, tf_msg.transform.rotation.w = goal_quat
            self.static_broadcaster.sendTransform(tf_msg)

            # Execute motion based on motion_type
            pf = MoveIt2(
                node=self,
                joint_names=["joint1","joint2","joint3","joint4","joint5","joint6"],
                base_link_name=self.reference_frame,
                end_effector_name=tcp_link,
                group_name="portafilter_center",
            )
            pf.planner_id                 = self.planner_id
            pf.max_velocity               = self.velocity_scaling
            pf.max_acceleration           = self.acceleration_scaling
            pf.cartesian_jump_threshold   = self.cartesian_jump_threshold
            pf.cartesian_avoid_collisions = self.cartesian_avoid_collisions

            if motion_type.lower() == "joint":
                self.get_logger().info("approach_machine: Using JOINT motion planning")
                # Use joint motion - MoveIt2 will use inverse kinematics
                pf.move_to_pose(position=goal_pos, quat_xyzw=goal_quat, cartesian=False)
            else:
                self.get_logger().info("approach_machine: Using CARTESIAN motion planning")
                # Use Cartesian motion (original behavior)
                pf.move_to_pose(position=goal_pos,
                                quat_xyzw=goal_quat,
                                cartesian=cartesian_override,
                                cartesian_max_step=self.cartesian_max_step,
                                cartesian_fraction_threshold=self.cartesian_fraction_threshold)
            
            pf.wait_until_executed()

            # Verify execution via servo readiness
            while not self.wait_for_servo_ready(timeout=15.0):
                self.get_logger().warn("approach_machine: Motion not complete, rechecking servo readiness...")
                time.sleep(0.1)

            self.get_logger().info("approach_machine: Approach pose reached successfully.")
            return True
        except Exception as e:
            self.get_logger().error(f"approach_machine: Exception: {e}")
            return False

    # ---------------------------------------------------------------------------
    # 3)  mount_machine  – move to mount pose & broadcast *_mount_test
    # ---------------------------------------------------------------------------
    def mount_machine(self, machine_name: str, point_name: str, cartesian_override: bool = True, motion_type: str = "cartesian") -> bool:
        """
        Move the portafilter_link to the 'mount_pose' of <machine_name>/<point_name>.
        Also broadcasts a static TF '<machine_name>_<point_name>_mount_test'
        for the computed goal pose, then verifies execution via servo readiness.
        
        Args:
            machine_name: Name of the machine
            point_name: Name of the point on the machine
            cartesian_override: Use Cartesian planning (only applies when motion_type="cartesian")
            motion_type: "cartesian" for Cartesian motion (default) or "joint" for joint motion
        """
        try:
            # ── load files ───────────────────────────────────────────────────────
            pkg_share = get_package_share_directory("pickn_place")
            mem_file  = os.path.join(pkg_share, "machine_pose_data_memory.yaml")
            off_file  = os.path.join(pkg_share, "machine_offset_points.yaml")

            machines = (yaml.safe_load(open(mem_file)) or {}).get("machines", {})
            offsets  = (yaml.safe_load(open(off_file)) or {}).get(machine_name, {})

            if machine_name not in machines:
                self.get_logger().error("mount_machine: machine pose missing.")
                return False
            if "mount_pose" not in offsets.get(point_name, {}):
                self.get_logger().error("mount_machine: mount_pose not found.")
                return False

            # Build base transform
            base_t = machines[machine_name]["translation"]
            base_q = machines[machine_name]["rotation"]
            M_base        = tf_transformations.quaternion_matrix([base_q["x"], base_q["y"],
                                                                base_q["z"], base_q["w"]])
            M_base[0:3, 3] = [base_t["x"], base_t["y"], base_t["z"]]

            # Load offset
            mp       = offsets[point_name]["mount_pose"]
            tcp_link = mp.get("TCP", DEFAULT_TCP_LINK)
            off_t    = mp["translation"]
            off_q    = mp["rotation"]
            M_off        = tf_transformations.quaternion_matrix([off_q["x"], off_q["y"],
                                                                off_q["z"], off_q["w"]])
            M_off[0:3, 3] = [off_t["x"], off_t["y"], off_t["z"]]

            # Compute goal
            M_goal    = M_base.dot(M_off)
            goal_pos  = M_goal[0:3, 3].tolist()
            goal_quat = tf_transformations.quaternion_from_matrix(M_goal)

            # Broadcast test TF
            from geometry_msgs.msg import TransformStamped
            import tf2_ros
            if not hasattr(self, "static_broadcaster"):
                self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
            tf_msg = TransformStamped()
            tf_msg.header.stamp    = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = "base_link"
            tf_msg.child_frame_id  = f"{machine_name}_{point_name}_mount_test"
            tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z = goal_pos
            tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, \
            tf_msg.transform.rotation.z, tf_msg.transform.rotation.w = goal_quat
            self.static_broadcaster.sendTransform(tf_msg)

            # Execute motion based on motion_type
            pf = MoveIt2(
                node=self,
                joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                base_link_name=self.reference_frame,
                end_effector_name=tcp_link,
                group_name="portafilter_center",
            )
            pf.planner_id                 = self.planner_id
            pf.max_velocity               = self.velocity_scaling
            pf.max_acceleration           = self.acceleration_scaling
            pf.cartesian_jump_threshold   = self.cartesian_jump_threshold
            pf.cartesian_avoid_collisions = self.cartesian_avoid_collisions

            if motion_type.lower() == "joint":
                self.get_logger().info("mount_machine: Using JOINT motion planning")
                # Use joint motion - MoveIt2 will use inverse kinematics
                pf.move_to_pose(position=goal_pos, quat_xyzw=goal_quat, cartesian=False)
            else:
                self.get_logger().info("mount_machine: Using CARTESIAN motion planning")
                # Use Cartesian motion (original behavior)
                pf.move_to_pose(position=goal_pos,
                                quat_xyzw=goal_quat,
                                cartesian=cartesian_override,
                                cartesian_max_step=self.cartesian_max_step,
                                cartesian_fraction_threshold=self.cartesian_fraction_threshold)
            
            pf.wait_until_executed()

            # Verify execution via servo readiness
            while not self.wait_for_servo_ready(timeout=15.0):
                self.get_logger().warn("mount_machine: Motion not complete, rechecking servo readiness...")
                time.sleep(0.1)

            self.get_logger().info("mount_machine: Mount pose reached successfully.")
            return True

        except Exception as e:
            self.get_logger().error(f"mount_machine: Exception: {e}")
            return False

import threading
import rclpy
from rclpy.executors import SingleThreadedExecutor

def run_skill(skill_name: str, *skill_args):
    """
    Run one DirectTfMotionNode method in a fresh node/executor
    and return whatever that method returns.
    """
    rclpy.init()
    node = DirectTfMotionNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    result_container = {}            # will hold {"value": ...}

    def _wrapper():
        try:
            result_container["value"] = getattr(node, skill_name)(*skill_args)
        except Exception as e:
            node.get_logger().error(f"{skill_name} threw: {e}")
            result_container["value"] = None          # or False, as you prefer

    th = threading.Thread(target=_wrapper, daemon=True)
    th.start()

    try:
        while th.is_alive():
            try:
                executor.spin_once(timeout_sec=0.1)
            except rclpy._rclpy_pybind11.RCLError as e:
                # Only warn about non-shutdown related errors
                if not ("wait set index" in str(e) and "out of bounds" in str(e)):
                    node.get_logger().warn(f"RCLError: {e}")
    finally:
        # Ensure proper cleanup order
        try:
            executor.shutdown()
            if th.is_alive():
                th.join(timeout=2.0)  # Give thread 2s to finish
            node.destroy_node()
        except Exception as e:
            print(f"Warning: Error during cleanup: {e}")
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass  # rclpy may already be shutdown

    return result_container.get("value") # may be None

# def main():
#     try:
#         run_skill("refresh_position")
#         time.sleep(3.0)

#         ########################################

#         for i in range(1):
#             # run_skill("moveEE", 0, 0, 0, 0, 0, 0, "portafilter_link")
#             # run_skill("gotoEE", 0,-350,250,90,0,0)
#             run_skill("moveJ_deg",-45, 0, 0, 0, 0, 0, 1.0, 0.1)
#             # run_skill("gotoJ_deg", 0, 0, 0, 0, 0, 0)
#             # run_skill("move_to", 'espresso_grinder', 0.15, 0.0, 0.0, 0.0)
#             # run_skill("enforce_rxry")
#             # run_skill("move_portafilter_arc", 45)
            
#     except KeyboardInterrupt:
#         pass

# if __name__ == "__main__":
#     main()

#-------FUNCTIONS--------------------------------------------------------------------------------------------------------------------------------------------------------------
        # run_skill("move_portafilter_arc", 20)
        # run_skill("moveEE", 100,  0, -150, 10, 0, 0) #move end effector from current position. x y z mm, rx ry rz deg
        # run_skill("release_tension") #activate drag mode for 1 sec
        # run_skill("enforce_rxry")
        # run_skill("move_portafilter_arc", 10) #move portafilter in arc degrees -CW +CCW
        # run_skill("moveJ_deg", 0, 0, 0, 0, 0, 0)  #move joints from current position. j1 j2 j3 j4 j5 j6 deg
        # run_skill("gotoEE", 100,  0, -150, 10, 0, 0) # same as move but absolute, no reference, careful! - no motion planning 
        # run_skill("gotoJ_deg", 0, 0, 0, 0, 0, 0) # same as move but absolute, no reference, careful! - no motion planning 
        # run_skill("approach_machine", "three_group_espresso", "group_1")
        # run_skill("mount_machine", "three_group_espresso", "group_1")
        # run_skill("move_to", "three_group_espresso", 0.12)
        # run_skill("get_machine_position", "three_group_espresso")
#-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------