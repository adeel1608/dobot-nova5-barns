#!/usr/bin/env python3
##############################
# CONFIGURATION
##############################

# MOVE FUNCTION
REFERENCE_FRAME = "base_link"               # Reference frame for TF lookup.
PLANNER_ID = "OMPL"                         # Planner ID for MoveIt2.
CARTESIAN = True                            # Use Cartesian planning?
CARTESIAN_MAX_STEP = 0.001                  # Maximum step size for Cartesian motion.
CARTESIAN_FRACTION_THRESHOLD = 0.5          # Fraction threshold for Crtesian planning.
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
        and the last sample’s timestamp is within the freshness threshold.
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
    
    def refresh_position(self):
        """
        Refreshes the MoveIt2 robot state based on a single JointState message.
        """
        joint_state_msg = self.wait_for_joint_state("/joint_states_robot", timeout_sec=5.0)
        if joint_state_msg is None:
            self.get_logger().error("refresh_position: No joint state message received within timeout.")
            return

        joint_angles = [round(angle, 3) for angle in joint_state_msg.position[0:6]]
        self.get_logger().info(f"refresh_position: Refreshing state with joint angles: {joint_angles}")

        # Build the trajectory goal
        from trajectory_msgs.msg import JointTrajectoryPoint
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = 1
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f"refresh_position: Sending trajectory goal: {point.positions}")
        send_goal_future = self.traj_cli.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        if send_goal_future.result() is None:
            self.get_logger().error("refresh_position: Failed to send the trajectory goal.")
            return
        self.get_logger().info("refresh_position: Robot state successfully refreshed.")

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
        Reset the Dobot’s servo-controller from ERROR ➜ READY.

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
        Pulse drag‐mode once to let the Dobot relax, then refresh the robot’s current position.
        Retries StartDrag/StopDrag up to 3 times each, distinguishes timeouts vs driver errors,
        and enforces a minimum 0.5 s wait for each StartDrag attempt.
        Returns True on full success, False on any failure.
        """
        max_attempts   = 10
        timeout_sec    = 1.0
        min_wait_sec   = 0.3
        settling_time  = 0.3

        # 1) Activate drag mode (StartDrag loop)
        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(f"release_tension: Activating drag mode (attempt {attempt}/{max_attempts})")
            start_time = time.monotonic()
            start_fut  = self.start_drag_cli.call_async(StartDrag.Request())
            # wait up to timeout_sec for the response
            rclpy.spin_until_future_complete(self, start_fut, timeout_sec=timeout_sec)

            # Determine outcome
            driver_error = False
            timed_out    = False
            if start_fut.done():
                result = start_fut.result()
                if result is not None and getattr(result, "res", 1) == 0:
                    self.get_logger().info("release_tension: StartDrag succeeded")
                    success = True
                else:
                    driver_error = True
                    success = False
            else:
                timed_out = True
                success   = False

            # Enforce minimum dwell time for StartDrag
            elapsed = time.monotonic() - start_time
            if elapsed < min_wait_sec:
                time.sleep(min_wait_sec - elapsed)

            if success:
                break
            elif timed_out:
                self.get_logger().warn(f"release_tension: StartDrag attempt {attempt} timed out after {timeout_sec}s")
            else:  # driver_error
                code = start_fut.result().res if start_fut.done() and start_fut.result() else None
                self.get_logger().warn(f"release_tension: StartDrag attempt {attempt} returned error code {code}")

        else:
            self.get_logger().error(f"release_tension: StartDrag failed after {max_attempts} attempts")
            return False

        # 2) Settling delay
        self.get_logger().info(f"release_tension: Waiting {settling_time:.2f}s for settling...")
        time.sleep(settling_time)

        # 3) Deactivate drag mode (StopDrag loop)
        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(f"release_tension: Deactivating drag mode (attempt {attempt}/{max_attempts})")
            stop_fut = self.stop_drag_cli.call_async(StopDrag.Request())
            rclpy.spin_until_future_complete(self, stop_fut, timeout_sec=timeout_sec)

            if stop_fut.done() and stop_fut.result() is not None and getattr(stop_fut.result(), "res", 1) == 0:
                self.get_logger().info("release_tension: StopDrag succeeded")
                break
            elif not stop_fut.done():
                self.get_logger().warn(f"release_tension: StopDrag attempt {attempt} timed out after {timeout_sec}s")
            else:
                code = stop_fut.result().res
                self.get_logger().warn(f"release_tension: StopDrag attempt {attempt} returned error code {code}")
        else:
            self.get_logger().error(f"release_tension: StopDrag failed after {max_attempts} attempts")
            return False

        # 4) Refresh robot pose
        self.get_logger().info("release_tension: Refreshing robot position...")
        self.refresh_position()
        time.sleep(1.0)
        self.get_logger().info("release_tension: Completed successfully.")
        return True

###################################### DONE NEW VERIFICATION ###########################################
    def move_to(
        self,
        target_tf: str,
        distance: float,
        offset_x_mm: float = 0.0,
        offset_y_mm: float = 0.0,
        offset_z_mm: float = 0.0,
    ) -> bool:
        """
        Move the end-effector along the vector from a fixed base reference
        to the target frame by `distance` (m), then apply additional
        translation offsets (in millimetres) to the goal pose.

        Args:
            target_tf:       TF frame name to approach
            distance:        Retract distance in metres
            offset_x_mm:     Extra X offset (mm)
            offset_y_mm:     Extra Y offset (mm)
            offset_z_mm:     Extra Z offset (mm)
        """
        self.get_logger().info(
            f"move_to(): frame={target_tf}, distance={distance}m, "
            f"offsets=({offset_x_mm}mm, {offset_y_mm}mm, {offset_z_mm}mm)"
        )
        
        # 1) Get a stable transform
        self.ignore_orientation = True
        stable_tf = self.wait_for_stable_tf(target_tf)
        self.ignore_orientation = False
        if stable_tf is None:
            self.get_logger().error("move_to(): failed to get stable TF.")
            return False

        # 2) Compute base approach position
        target_pos = np.array(stable_tf[:3])
        base_ref   = np.array([0.0, 0.0, target_pos[2] + 0.075])
        approach   = target_pos - base_ref
        norm       = np.linalg.norm(approach)
        if norm < 1e-6:
            self.get_logger().error("move_to(): approach vector too small.")
            return False
        unit_vec = approach / norm
        goal_pos = target_pos - distance * unit_vec

        # 3) Apply mm-to-m offsets
        offsets_m = np.array([offset_x_mm, offset_y_mm, offset_z_mm]) / 1000.0
        goal_pos += offsets_m

        # 4) Compute orientation quaternion
        z_axis = unit_vec
        up     = np.array([0, 0, 1])
        if abs(np.dot(z_axis, up)) > 0.99:
            up = np.array([0, 1, 0])
        x_axis = np.cross(up, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        R      = np.column_stack((x_axis, y_axis, z_axis))
        quat   = tf_transformations.quaternion_from_matrix(
                    np.vstack((np.hstack((R, [[0],[0],[0]])), [0,0,0,1]))
                )

        # 5) Execute Cartesian move
        self.moveit2.move_to_pose(
            position=goal_pos.tolist(),
            quat_xyzw=quat,
            cartesian=self.cartesian,
            cartesian_max_step=self.cartesian_max_step,
            cartesian_fraction_threshold=self.cartesian_fraction_threshold,
        )
        
        # 6) Verify arrival
        self.moveit2.wait_until_executed()
        while not self.wait_for_servo_ready(timeout=15.0):
            self.get_logger().warn("move_to(): not arrived, rechecking...")
            time.sleep(0.2)

        self.get_logger().info("move_to(): completed successfully.")
        return True

    def approach_tool(self, target_tf: str, gripper_position: int = 255):
        """
        Move the end-effector to a pre-defined “approach” offset for the given target_tf,
        then command the gripper to the specified position.

        Args:
            target_tf (str): name of the TF frame to approach
            gripper_position (int): 0–255 gripper position (0=open, 255=closed)
        """
        self.get_logger().info(f"approach(): Using transform for '{target_tf}'")

        # 1) Wait for a stable transform to the target frame
        stable_tf = self.wait_for_stable_tf(target_tf)
        if stable_tf is None:
            self.get_logger().error("approach(): Failed to obtain a stable transform.")
            return

        # 2) Build the approach pose from static offsets in YAML
        t, q = stable_tf[:3], stable_tf[3:]
        input_matrix = tf_transformations.quaternion_matrix(q)
        input_matrix[0:3, 3] = t

        package_share = get_package_share_directory("pickn_place")
        file_path = os.path.join(package_share, "tool_offset_points.yaml")
        try:
            with open(file_path, "r") as f:
                offsets_data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"approach(): Failed to load YAML file: {e}")
            return

        if target_tf not in offsets_data:
            self.get_logger().error("approach(): No offset points for target TF")
            return

        tf_offsets = offsets_data[target_tf]
        if "approach_pose" not in tf_offsets:
            self.get_logger().error(f"approach(): Missing approach_pose offset for {target_tf}")
            return

        off = tf_offsets["approach_pose"]
        off_t = [off["translation"][axis] for axis in ("x","y","z")]
        off_q = [off["rotation"][axis] for axis in ("x","y","z","w")]
        approach_matrix = tf_transformations.quaternion_matrix(off_q)
        approach_matrix[0:3, 3] = off_t

        goal_matrix = np.dot(input_matrix, approach_matrix)
        approach_position = goal_matrix[0:3, 3].tolist()
        approach_quaternion = tf_transformations.quaternion_from_matrix(goal_matrix)

        self.get_logger().info(
            f"approach(): Computed approach pose:\n"
            f"  Position: {approach_position}\n"
            f"  Orientation (xyzw): {approach_quaternion}"
        )

        # 3) Execute the Cartesian move to approach pose
        self.moveit2.move_to_pose(
            position=approach_position,
            quat_xyzw=approach_quaternion,
            cartesian=self.cartesian,
            cartesian_max_step=self.cartesian_max_step,
            cartesian_fraction_threshold=self.cartesian_fraction_threshold,
        )
        self.moveit2.wait_until_executed()
        state = self.moveit2.query_state()
        if state == MoveIt2State.IDLE:
            self.get_logger().info("approach(): Motion reached successfully.")
        else:
            self.get_logger().warn(f"approach(): Motion ended with state: {state}")

        # 4) Verify that we’ve arrived within tolerance
        while not self.verify_goal_pose(expected_goal_pose=approach_position,
                                        tolerance=2.0,
                                        offset=136.55):
            self.get_logger().warn("approach(): Goal pose verification failed, retrying.")
            time.sleep(0.2)

        self.get_logger().info("approach(): Pose reached—now commanding gripper.")

        # 5) Command the gripper to the requested position
        if not self.set_gripper_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("approach(): SetGripperPosition service unavailable.")
        else:
            req = SetGripperPosition.Request()
            req.position = gripper_position
            req.speed    = 255
            req.force    = 255
            future = self.set_gripper_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(
                f"approach(): Gripper commanded to position {gripper_position}"
            )
        time.sleep(0.5) #Gripper settling time

        self.get_logger().info("approach(): Function completed successfully.")

    def grab_tool(
        self,
        target_tf: str,
        verify_min: int = 130,
        verify_max: int = 140,
        close_position: int = 255,
    ) -> bool:
        """
        Close the gripper on the object located at *target_tf* and verify the
        grasp.  On verification failure the gripper is reopened to its original
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
        result = False
        fallback_moved = False
        fallback_pose = None
        open_ref_pos  = None

        try:
            # ───────────────────────────────────────────────────────────────────
            # 0)  Get fresh, stable TF for the object
            # ───────────────────────────────────────────────────────────────────
            stable_tf = self.wait_for_stable_tf(target_tf)
            if stable_tf is None:
                self.get_logger().error("grab_tool: failed to obtain stable TF.")
                return False
            t_dyn, q_dyn = stable_tf[:3], stable_tf[3:]
            M_dyn              = tf_transformations.quaternion_matrix(q_dyn)
            M_dyn[0:3, 3]      = t_dyn

            # ───────────────────────────────────────────────────────────────────
            # 1)  Snapshot current robot pose as fallback
            # ───────────────────────────────────────────────────────────────────
            gp_cli = self.get_pose_cli
            if not gp_cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("grab_tool: GetPose service not available.")
                raise Exception("Early failure: GetPose missing.")

            gp_req    = GetPose.Request()
            gp_req.user = 0
            gp_req.tool = 0
            gp_future = gp_cli.call_async(gp_req)
            rclpy.spin_until_future_complete(self, gp_future, timeout_sec=2.0)

            if gp_future.result() is None:
                self.get_logger().error("grab_tool: GetPose call failed.")
                raise Exception("Early failure: GetPose call failed.")

            pose_vals = [float(v) for v in gp_future.result().pose.strip("{}").split(",")]
            if len(pose_vals) < 6:
                raise Exception("Early failure: malformed GetPose response.")

            tx, ty, tz, rx, ry, rz = pose_vals[:6]
            fallback_translation = [tx / 1000.0, ty / 1000.0, tz / 1000.0]
            R_fb  = tf_transformations.euler_matrix(
                math.radians(rx), math.radians(ry), math.radians(rz)
            )
            fallback_quat = tf_transformations.quaternion_from_matrix(R_fb)
            fallback_pose = fallback_translation + list(fallback_quat)
            self.get_logger().info(f"grab_tool: saved fallback pose: {fallback_pose}")

            # remember current gripper opening
            if self.get_gripper_cli.wait_for_service(timeout_sec=1.0):
                g_req  = GetGripperPosition.Request(); g_req.index = 0
                g_fut  = self.get_gripper_cli.call_async(g_req)
                rclpy.spin_until_future_complete(self, g_fut)
                time.sleep(1.0)
                if g_fut.result():
                    open_ref_pos = g_fut.result().position
                    self.get_logger().info(f"grab_tool: ref gripper pos={open_ref_pos}")

            # ───────────────────────────────────────────────────────────────────
            # 2)  Load grab offset from YAML
            # ───────────────────────────────────────────────────────────────────
            pkg_share  = get_package_share_directory("pickn_place")
            yfile      = os.path.join(pkg_share, "tool_offset_points.yaml")
            tf_offsets = yaml.safe_load(open(yfile, "r")) or {}

            if target_tf not in tf_offsets or "grab_pose" not in tf_offsets[target_tf]:
                self.get_logger().error("grab_tool: grab_pose missing in YAML.")
                raise Exception("YAML load failure.")

            grab_off   = tf_offsets[target_tf]["grab_pose"]
            off_t      = [grab_off["translation"][k] for k in ("x", "y", "z")]
            off_q      = [grab_off["rotation"][k]    for k in ("x", "y", "z", "w")]
            M_off              = tf_transformations.quaternion_matrix(off_q)
            M_off[0:3, 3]      = off_t

            # ───────────────────────────────────────────────────────────────────
            # 3)  Compute absolute goal pose
            # ───────────────────────────────────────────────────────────────────
            M_goal     = M_dyn.dot(M_off)
            goal_pos   = M_goal[0:3, 3].tolist()
            goal_quat  = tf_transformations.quaternion_from_matrix(M_goal)
            self.get_logger().info(
                f"grab_tool: goal pose\n  pos={goal_pos}\n  quat={goal_quat}"
            )

            # ───────────────────────────────────────────────────────────────────
            # 4)  Move to the goal pose
            # ───────────────────────────────────────────────────────────────────
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
                self.get_logger().warn(f"grab_tool: motion finished with state {state}")

            # verify arrival
            wait_elapsed, max_wait = 0.0, 2.0
            while not self.verify_goal_pose(goal_pos, 2.0, 136.55) and wait_elapsed < max_wait:
                self.get_logger().warn("grab_tool: waiting for pose verification …")
                time.sleep(0.2); wait_elapsed += 0.2
            if wait_elapsed >= max_wait:
                self.get_logger().warn("grab_tool: pose not verified, retreating.")
                if fallback_pose:
                    self.moveit2.move_to_pose(
                        position=fallback_pose[:3],
                        quat_xyzw=fallback_pose[3:],
                        cartesian=self.cartesian,
                        cartesian_max_step=self.cartesian_max_step,
                        cartesian_fraction_threshold=self.cartesian_fraction_threshold
                    )
                    self.moveit2.wait_until_executed()
                    fallback_moved = True
                raise Exception("Goal pose verification failed.")

            # ───────────────────────────────────────────────────────────────────
            # 5)  Drag-mode cycle + gripper close
            # ───────────────────────────────────────────────────────────────────
            if not self.start_drag_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("grab_tool: StartDrag unavailable.")
                raise Exception("StartDrag failure.")
            self.start_drag_cli.call_async(StartDrag.Request())
            time.sleep(0.3)

            if not self.set_gripper_cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("grab_tool: SetGripperPosition unavailable.")
                raise Exception("SetGripperPosition failure.")
            s_req = SetGripperPosition.Request()
            s_req.position = close_position
            s_req.speed    = 255
            s_req.force    = 255
            self.set_gripper_cli.call_async(s_req)
            time.sleep(0.5)

            if not self.stop_drag_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("grab_tool: StopDrag unavailable.")
                raise Exception("StopDrag failure.")
            self.stop_drag_cli.call_async(StopDrag.Request())
            time.sleep(0.2)
            self.refresh_position()

            # ───────────────────────────────────────────────────────────────────
            # 6)  Gripper-window verification
            # ───────────────────────────────────────────────────────────────────
            if not self.get_gripper_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("grab_tool: GetGripperPosition unavailable.")
                raise Exception("GetGripperPosition failure.")
            g_req = GetGripperPosition.Request(); g_req.index = 0
            g_fut = self.get_gripper_cli.call_async(g_req)
            rclpy.spin_until_future_complete(self, g_fut, timeout_sec=1.0)
            pos = g_fut.result().position if g_fut.result() else 255
            self.get_logger().info(f"grab_tool: gripper reading = {pos}")

            # ---------- SUCCESS ----------
            if verify_min <= pos <= verify_max:
                self.get_logger().info("grab_tool: grasp verified – success.")
                result = True

            # ---------- FAILURE ----------
            else:
                self.get_logger().warn("grab_tool: verification failed – retreating to fallback.")
                # A) reopen gripper to its original opening
                if open_ref_pos is not None and self.set_gripper_cli.wait_for_service(timeout_sec=1.0):
                    reopen_req            = SetGripperPosition.Request()
                    reopen_req.position   = open_ref_pos
                    reopen_req.speed      = 255
                    reopen_req.force      = 255
                    self.set_gripper_cli.call_async(reopen_req)
                    time.sleep(0.5)

                # B) retreat robot to the saved pose
                if fallback_pose:
                    try:
                        ee6 = MoveIt2(
                            node=self,
                            joint_names=["joint1","joint2","joint3",
                                         "joint4","joint5","joint6"],
                            base_link_name=self.reference_frame,
                            end_effector_name="Link6",
                            group_name=GROUP_NAME,
                        )
                        ee6.planner_id                 = self.planner_id
                        ee6.max_velocity               = self.velocity_scaling
                        ee6.max_acceleration           = self.acceleration_scaling
                        ee6.cartesian_jump_threshold   = self.cartesian_jump_threshold
                        ee6.cartesian_avoid_collisions = self.cartesian_avoid_collisions

                        ee6.move_to_pose(
                            position=fallback_pose[:3],
                            quat_xyzw=fallback_pose[3:],
                            cartesian=True,
                            cartesian_max_step=self.cartesian_max_step,
                            cartesian_fraction_threshold=self.cartesian_fraction_threshold
                        )
                        ee6.wait_until_executed()
                        fallback_moved = True
                    except Exception:
                        self.get_logger().error("grab_tool: failed to retreat to fallback.")
                result = False  # explicit

        # ───────────────────────────────────────────────────────────────────────
        # 7)  Exception & recovery block
        # ───────────────────────────────────────────────────────────────────────
        except Exception as e:
            self.get_logger().error(f"grab_tool: exception {e}")
            if fallback_pose and not fallback_moved:
                self.get_logger().info("grab_tool: returning to fallback pose.")
                try:
                    ee6 = MoveIt2(
                        node=self,
                        joint_names=["joint1","joint2","joint3",
                                     "joint4","joint5","joint6"],
                        base_link_name=self.reference_frame,
                        end_effector_name="Link6",
                        group_name=GROUP_NAME,
                    )
                    ee6.planner_id                 = self.planner_id
                    ee6.max_velocity               = self.velocity_scaling
                    ee6.max_acceleration           = self.acceleration_scaling
                    ee6.cartesian_jump_threshold   = self.cartesian_jump_threshold
                    ee6.cartesian_avoid_collisions = self.cartesian_avoid_collisions

                    ee6.move_to_pose(
                        position=fallback_pose[:3],
                        quat_xyzw=fallback_pose[3:],
                        cartesian=True,
                        cartesian_max_step=self.cartesian_max_step,
                        cartesian_fraction_threshold=self.cartesian_fraction_threshold
                    )
                    ee6.wait_until_executed()
                except Exception:
                    self.get_logger().error("grab_tool: failed to retreat to fallback.")

            result = False

        return result

    # ---------------------------------------------------------------------------
    # ♦ set_gripper_position  – single‐shot gripper command + status read
    # ---------------------------------------------------------------------------
    def set_gripper_position(
            self,
            speed: int = 255,
            position: int = 255,
            force: int = 255,
            settling_time: float = 0.5
    ) -> int | None:
        """
        Command the Dobot gripper to *position* (0 = open … 255 = closed),
        wait for settling, then return the current gripper reading.

        Returns
        -------
        reading : int
            The gripper position read back after settling, or None on failure.
        """
        # ── 0) Clamp inputs ─────────────────────────────────────────────────────
        position = max(0, min(255, int(position)))
        speed    = max(0, min(255, int(speed)))
        force    = max(0, min(255, int(force)))

        # ── 1) Ensure services available ────────────────────────────────────────
        if not self.set_gripper_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("set_gripper_position: SetGripperPosition service unavailable.")
            return None
        if not self.get_gripper_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("set_gripper_position: GetGripperPosition service unavailable.")
            return None

        # ── 2) Send SetGripperPosition ─────────────────────────────────────────
        req = SetGripperPosition.Request()
        req.position = position
        req.speed    = speed
        req.force    = force
        fut = self.set_gripper_cli.call_async(req)

        start = self.get_clock().now().nanoseconds * 1e-9
        # wait up to 2s for the command to be accepted
        while not fut.done() and (self.get_clock().now().nanoseconds * 1e-9 - start) < 2.0:
            rclpy.spin_once(self, timeout_sec=0.01)

        if not fut.done():
            self.get_logger().warn("set_gripper_position: command timed out.")

        # ── 3) Settling delay ───────────────────────────────────────────────────
        time.sleep(settling_time)

        # ── 4) Read back with GetGripperPosition ───────────────────────────────
        get_req = GetGripperPosition.Request()
        get_req.index = 0
        get_fut = self.get_gripper_cli.call_async(get_req)

        start = self.get_clock().now().nanoseconds * 1e-9
        while not get_fut.done() and (self.get_clock().now().nanoseconds * 1e-9 - start) < 2.0:
            rclpy.spin_once(self, timeout_sec=0.01)

        if not get_fut.done() or get_fut.result() is None:
            self.get_logger().error("set_gripper_position: failed to read back gripper position.")
            return None

        reading = get_fut.result().position
        self.get_logger().info(f"set_gripper_position: requested {position}, read back {reading}")
        return reading

    def enforce_rxry(self) -> bool:
        """
        Override Link6’s Rx→90°, Ry→0° (keep current Rz) while freezing
        the world‐space position of portafilter_link to ±0.5 mm.
        Returns True on success.
        """
        import numpy as np
        import time
        from tf_transformations import euler_matrix, quaternion_from_matrix

        # fixed offset from Link6 origin → portafilter_link origin (m)
        d_rel = np.array([0.0, 0.0, 0.276])

        # 1) get current Link6 pose (with retries)
        if not self.get_pose_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("enforce_rxry(): GetPose service unavailable.")
            return False

        req = GetPose.Request()
        req.user = 0
        req.tool = 0

        resp = None
        for attempt in range(1, 4):
            future = self.get_pose_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            resp = future.result()
            if resp is not None and hasattr(resp, "pose"):
                break
            self.get_logger().warn(
                f"enforce_rxry(): GetPose attempt {attempt}/3 failed, retrying in 0.5 s…"
            )
            time.sleep(0.5)

        if resp is None or not hasattr(resp, "pose"):
            self.get_logger().error("enforce_rxry(): Failed to retrieve pose after 3 attempts.")
            return False

        # parse "{tx,ty,tz,rx,ry,rz,…}"
        parts = resp.pose.strip("{}").split(",")
        if len(parts) < 6:
            self.get_logger().error("enforce_rxry(): Invalid pose format.")
            return False
        try:
            tx_mm, ty_mm, tz_mm, rx_curr, ry_curr, rz_curr = [float(p) for p in parts[:6]]
        except Exception as e:
            self.get_logger().error(f"enforce_rxry(): Error parsing pose: {e}")
            return False

        # convert to metres and build current rotation matrix
        p_link6 = np.array([tx_mm, ty_mm, tz_mm]) * 1e-3
        rads = np.radians([rx_curr, ry_curr, rz_curr])
        R6_curr = euler_matrix(*rads)[:3, :3]

        # compute world position of the portafilter_link
        p_pf_world = p_link6 + R6_curr.dot(d_rel)

        # 2) define the *desired* Link6 orientation: Rx=90°, Ry=0°, keep Rz
        rx_t, ry_t, rz_t = 90.0, 0.0, rz_curr
        rads_goal = np.radians([rx_t, ry_t, rz_t])
        M_goal = euler_matrix(*rads_goal)
        R6_goal = M_goal[:3, :3]
        quat_goal = list(quaternion_from_matrix(M_goal))

        # 3) back-solve Link6 goal position so portafilter_link stays put
        p6_goal = p_pf_world - R6_goal.dot(d_rel)
        self.get_logger().info(f"enforce_rxry(): Goal Link6 pos (m): {p6_goal.tolist()}")

        # 4) plan & execute with Link6 as the end effector
        temp = MoveIt2(
            node=self,
            joint_names=self.moveit2.joint_names,
            base_link_name=self.reference_frame,
            end_effector_name="Link6",
            group_name=GROUP_NAME,
        )
        temp.planner_id                 = self.planner_id
        temp.max_velocity               = self.velocity_scaling
        temp.max_acceleration           = self.acceleration_scaling
        temp.cartesian_jump_threshold   = self.cartesian_jump_threshold
        temp.cartesian_avoid_collisions = self.cartesian_avoid_collisions

        self.get_logger().info("enforce_rxry(): Calling MoveIt2.move_to_pose()...")
        temp.move_to_pose(
            position=p6_goal.tolist(),
            quat_xyzw=quat_goal,
            cartesian=self.cartesian,
            cartesian_max_step=self.cartesian_max_step,
            cartesian_fraction_threshold=self.cartesian_fraction_threshold,
        )
        temp.wait_until_executed()
        state = temp.query_state()
        if state != MoveIt2State.IDLE:
            self.get_logger().warn(f"enforce_rxry(): Motion ended with state: {state}")

        # 5) verify portafilter_link stayed within 0.5 mm
        tol = 0.002  # 0.5 mm in metres
        while True:
            # re-read Link6 pose (with retries)
            resp2 = None
            for attempt in range(1, 4):
                future = self.get_pose_cli.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                resp2 = future.result()
                if resp2 is not None and hasattr(resp2, "pose"):
                    break
                self.get_logger().warn(
                    f"enforce_rxry(): Re-GetPose attempt {attempt}/3 failed, retrying in 0.5 s…"
                )
                time.sleep(0.5)

            if resp2 is None or not hasattr(resp2, "pose"):
                self.get_logger().error("enforce_rxry(): Failed to re-read pose after 3 attempts.")
                return False

            parts2 = resp2.pose.strip("{}").split(",")
            tx2, ty2, tz2, rx2, ry2, rz2 = [float(p) for p in parts2[:6]]
            p6_new = np.array([tx2, ty2, tz2]) * 1e-3
            R6_new = euler_matrix(*np.radians([rx2, ry2, rz2]))[:3, :3]
            p_pf_new = p6_new + R6_new.dot(d_rel)

            err = np.linalg.norm(p_pf_new - p_pf_world)
            if err <= tol:
                break

            self.get_logger().warn(
                f"enforce_rxry(): portafilter moved {err*1e3:.2f} mm (>0.5 mm), retrying…"
            )
            time.sleep(0.2)

        self.get_logger().info("enforce_rxry(): Completed with SUCCESS.")
        return True

    def move_portafilter_arc(self, angle_deg: float):
        pf_moveit2 = MoveIt2(
            node=self,
            joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            base_link_name=self.reference_frame,
            end_effector_name="portafilter_link",
            group_name="portafilter_center",
        )
        pf_moveit2.planner_id                 = self.planner_id
        pf_moveit2.max_velocity               = self.velocity_scaling
        pf_moveit2.max_acceleration           = self.acceleration_scaling
        pf_moveit2.cartesian_jump_threshold   = self.cartesian_jump_threshold
        pf_moveit2.cartesian_avoid_collisions = self.cartesian_avoid_collisions

        # 1) Get current portafilter pose
        try:
            pf_tf_stamped = self.tf_buffer.lookup_transform(
                self.reference_frame,
                "portafilter_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=5.0)
            )
            current_tf = get_transform_list(pf_tf_stamped)
        except Exception as e:
            self.get_logger().error(f"move_portafilter_arc: Failed to get portafilter_link transform: {e}")
            return

        position     = np.array(current_tf[:3])
        current_quat = np.array(current_tf[3:])

        # 2) Compute new orientation by rotating about local Y axis
        R_current    = tf_transformations.quaternion_matrix(current_quat)[0:3, 0:3]
        rotation_axis = R_current[:, 1]
        theta         = math.radians(angle_deg)
        relative_quat = tf_transformations.quaternion_about_axis(theta, rotation_axis)
        new_quat      = tf_transformations.quaternion_multiply(relative_quat, current_quat)
        new_pos       = position.tolist()

        self.get_logger().info(
            f"move_portafilter_arc: Rotating portafilter_link by {angle_deg}° "
            f"about its local y axis. New orientation (xyzw): {new_quat}"
        )

        # 3) Execute the arc motion
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
            self.get_logger().info("move_portafilter_arc: Rotation motion executed successfully.")
        else:
            self.get_logger().warn(f"move_portafilter_arc: Rotation motion ended with state: {state}")

        # 5) Joint-state sampling and verification — now blocking forever on failures
        tolerance_deg = 1          # degrees
        batch_size    = 10
        sleep_between = 0.3          # seconds

        while True:
            # a) collect one batch of samples
            joint_samples = []
            for _ in range(batch_size):
                js = self.wait_for_joint_state("/joint_states_robot", timeout_sec=1.0)
                if js:
                    angles_deg = [math.degrees(j) for j in js.position[0:6]]
                    joint_samples.append(angles_deg)
                time.sleep(sleep_between)

            # b) compute per-joint spread across this batch
            spreads = [max(vals) - min(vals) for vals in zip(*joint_samples)] if joint_samples else None

            # c) if we got valid samples and the spread is OK, we can break out
            if spreads is not None and all(spread < tolerance_deg for spread in spreads):
                break

            # otherwise log & retry endlessly
            self.get_logger().warn(
                f"move_portafilter_arc: Joint spread too large "
                f"({spreads or 'no samples'}), retrying…"
            )

            # d) as a fallback, try a single-shot verify; if it passes, we’re done
            if joint_samples:
                last = joint_samples[-1]
                if self.verify_joint_positions(last):
                    break
            # loop again

        self.get_logger().info("move_portafilter_arc: Function completed successfully.")

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
        """
        import time
        self.get_logger().info(
            f"moveEE(): Starting linear move using '{EE_link}' with translation offsets (mm): "
            f"({offset_x}, {offset_y}, {offset_z}) and rotation offsets (deg): "
            f"({offset_rx}, {offset_ry}, {offset_rz})."
        )

        # 1) Lookup current EE transform with retries
        from rclpy.time import Time
        from rclpy.duration import Duration
        attempts = 3
        tf_stamped = None
        for attempt in range(1, attempts + 1):
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    self.reference_frame,
                    EE_link,
                    Time(),
                    timeout=Duration(seconds=5.0)
                )
                break
            except Exception as e:
                self.get_logger().warn(
                    f"moveEE(): Attempt {attempt}/{attempts} failed to retrieve transform for '{EE_link}': {e}"
                )
                if attempt < attempts:
                    time.sleep(0.1)
        if tf_stamped is None:
            self.get_logger().error(
                f"moveEE(): Failed to retrieve transform after {attempts} attempts for '{EE_link}'"
            )
            return False

        current_tf = get_transform_list(tf_stamped)
        self.get_logger().info(f"moveEE(): Current pose obtained: {current_tf}")

        # 2) Compute goal pose: apply translation and rotation deltas
        import numpy as np
        import math
        t_offset = np.array([offset_x, offset_y, offset_z]) / 1000.0
        r_offset = np.radians([offset_rx, offset_ry, offset_rz])

        # Current translation & orientation
        current_translation = np.array(current_tf[:3])
        current_quat        = np.array(current_tf[3:])
        R_current           = tf_transformations.quaternion_matrix(current_quat)[0:3, 0:3]

        # Compute goal translation
        goal_translation = (current_translation + t_offset).tolist()

        # Compute goal rotation
        R_delta     = tf_transformations.euler_matrix(*r_offset)[0:3, 0:3]
        R_goal      = R_current.dot(R_delta)
        goal_matrix = np.eye(4)
        goal_matrix[0:3, 0:3] = R_goal
        goal_quat   = tf_transformations.quaternion_from_matrix(goal_matrix)

        self.get_logger().info(
            f"moveEE(): Computed goal pose:\n"
            f"  Position: {goal_translation}\n"
            f"  Orientation (xyzw): {goal_quat}"
        )

        # 3) Plan & execute via MoveIt2 using the specified EE_link
        temp_moveit2 = MoveIt2(
            node=self,
            joint_names=self.moveit2.joint_names,
            base_link_name=self.reference_frame,
            end_effector_name=EE_link,
            group_name=GROUP_NAME,
        )
        temp_moveit2.planner_id                 = self.planner_id
        temp_moveit2.max_velocity               = self.velocity_scaling
        temp_moveit2.max_acceleration           = self.acceleration_scaling
        temp_moveit2.cartesian_jump_threshold   = self.cartesian_jump_threshold
        temp_moveit2.cartesian_avoid_collisions = self.cartesian_avoid_collisions

        temp_moveit2.move_to_pose(
            position=goal_translation,
            quat_xyzw=goal_quat,
            cartesian=self.cartesian,
            cartesian_max_step=self.cartesian_max_step,
            cartesian_fraction_threshold=self.cartesian_fraction_threshold
        )

        # 4) Verify arrival using wait_for_servo_ready()
        temp_moveit2.wait_until_executed()
        while not self.wait_for_servo_ready(timeout=15.0):
            self.get_logger().warn("moveEE(): not arrived, rechecking...")
            time.sleep(0.2)

        self.get_logger().info("moveEE(): Function completed successfully.")
        return True

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
        import time
        import math

        # 1) Backup & apply scalings
        orig_vel = self.moveit2.max_velocity
        orig_acc = self.moveit2.max_acceleration
        self.moveit2.max_velocity = velocity_scaling
        self.moveit2.max_acceleration = acceleration_scaling

        try:
            # 2) Log the request
            rel_angles = [angle1, angle2, angle3, angle4, angle5, angle6]
            self.get_logger().info(
                f"moveJ_deg(): Rel angles (deg)={rel_angles}, "
                f"vel_scale={velocity_scaling}, acc_scale={acceleration_scaling}"
            )

            # 3) Wait for a stable “current” joint state (5-sample sliding window, ≤1° spread per joint)
            window = []
            start_t = time.time()
            stable_rad = None
            while True:
                js = self.wait_for_joint_state("/joint_states_robot", timeout_sec=0.2)
                if js and len(js.position) >= 6:
                    current_rad = list(js.position[:6])
                    current_deg = [math.degrees(x) for x in current_rad]
                    window.append((current_rad, current_deg))
                    if len(window) > 5:
                        window.pop(0)

                    if len(window) == 5:
                        spreads = [
                            max(col) - min(col)
                            for col in zip(*(w[1] for w in window))
                        ]
                        if all(s <= 1.0 for s in spreads):
                            stable_rad = window[-1][0]
                            break

                if time.time() - start_t > 5.0:
                    self.get_logger().error("moveJ_deg(): joint state stability timeout")
                    return False

            # 4) Compute the goal configuration (radians)
            rel_rad = [math.radians(a) for a in rel_angles]
            new_joints_rad = [c + r for c, r in zip(stable_rad, rel_rad)]

            # 5) Joint-limit check
            joint_limits = [
                (-6.2, 6.2),     # joint1
                (-3.14, 3.14),   # joint2
                (-2.79, 0.0),    # joint3
                (-6.28, 6.28),   # joint4
                (-6.28, 6.28),   # joint5
                (-6.28, 6.28),   # joint6
            ]
            for idx, (val, (low, high)) in enumerate(zip(new_joints_rad, joint_limits), start=1):
                if not (low <= val <= high):
                    self.get_logger().error(
                        f"moveJ_deg(): joint{idx} target {math.degrees(val):.3f}° "
                        f"outside limits [{math.degrees(low):.1f}°, {math.degrees(high):.1f}°]"
                    )
                    return False

            # 6) Execute the motion
            self.moveit2.move_to_configuration(new_joints_rad)

            # 8) Verify arrival via servo‐ready polling
            self.moveit2.wait_until_executed()
            while not self.wait_for_servo_ready(timeout=15.0):
                self.get_logger().warn("moveJ_deg(): not arrived, rechecking...")
                time.sleep(0.2)

            self.get_logger().info("moveJ_deg(): Function completed successfully.")
            return True

        finally:
            # 9) Restore original scalings
            self.moveit2.max_velocity = orig_vel
            self.moveit2.max_acceleration = orig_acc

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
            velocity_scaling: velocity scaling factor (0.0 to 1.0)
            acceleration_scaling: acceleration scaling factor (0.0 to 1.0)
        Returns:
            True on success, False on any failure.
        """

        # 1) Backup & apply scalings
        orig_vel = self.moveit2.max_velocity
        orig_acc = self.moveit2.max_acceleration
        self.moveit2.max_velocity = velocity_scaling
        self.moveit2.max_acceleration = acceleration_scaling

        try:
            # 2) Log the incoming command
            target_angles = [angle1, angle2, angle3, angle4, angle5, angle6]
            self.get_logger().info(
                f"gotoJ_deg(): Target absolute joint angles (deg): {target_angles}, "
                f"vel_scale={velocity_scaling}, acc_scale={acceleration_scaling}"
            )

            # 3) Convert to radians
            new_joints_rad = [math.radians(a) for a in target_angles]

            # 4) Joint-limit check
            joint_limits = [
                (-6.2, 6.2),    # joint1
                (-3.14, 3.14),  # joint2
                (-2.79, 0.0),   # joint3
                (-6.28, 6.28),  # joint4
                (-6.28, 6.28),  # joint5
                (-6.28, 6.28),  # joint6
            ]
            for idx, (rad, (low, high)) in enumerate(zip(new_joints_rad, joint_limits), start=1):
                if not (low <= rad <= high):
                    self.get_logger().error(
                        f"gotoJ_deg(): joint{idx} target {math.degrees(rad):.3f}° "
                        f"outside limits [{math.degrees(low):.1f}°, {math.degrees(high):.1f}°]"
                    )
                    return False

            # 5) Execute the motion
            self.moveit2.move_to_configuration(new_joints_rad)

            # 6) Verify arrival via servo‐ready polling
            self.moveit2.wait_until_executed()
            while not self.wait_for_servo_ready(timeout=15.0):
                self.get_logger().warn("gotoJ_deg(): not arrived, rechecking...")
                time.sleep(0.2)

            self.get_logger().info("gotoJ_deg(): Function completed successfully.")
            return True

        finally:
            # 7) Restore original scalings
            self.moveit2.max_velocity = orig_vel
            self.moveit2.max_acceleration = orig_acc


    def gotoEE(self, abs_x_mm: float, abs_y_mm: float, abs_z_mm: float,
               abs_rx_deg: float, abs_ry_deg: float, abs_rz_deg: float):
        """
        Move the end-effector (Link6) to an **absolute** pose expressed in the
        base_link frame, then verify arrival via wait_for_servo_ready().

        Args:
            abs_x_mm, abs_y_mm, abs_z_mm : absolute position in millimetres
            abs_rx_deg, abs_ry_deg, abs_rz_deg : absolute orientation (XYZ-Euler) in degrees
        """
        self.get_logger().info(
            f"gotoEE(): Target absolute pose – "
            f"Position (mm): ({abs_x_mm}, {abs_y_mm}, {abs_z_mm}), "
            f"Orientation (deg): ({abs_rx_deg}, {abs_ry_deg}, {abs_rz_deg})"
        )

        # 1) Build goal pose in metres + quaternion
        goal_translation = [coord / 1000.0 for coord in (abs_x_mm, abs_y_mm, abs_z_mm)]
        r_radians = [math.radians(a) for a in (abs_rx_deg, abs_ry_deg, abs_rz_deg)]
        R_goal = tf_transformations.euler_matrix(*r_radians)
        goal_quat = tf_transformations.quaternion_from_matrix(R_goal)

        self.get_logger().info(f"gotoEE(): Goal pose (m, xyzw): {goal_translation}  {goal_quat}")

        # 2) Plan & execute via MoveIt2 for Link6
        temp_moveit2 = MoveIt2(
            node=self,
            joint_names=["joint1","joint2","joint3","joint4","joint5","joint6"],
            base_link_name=self.reference_frame,
            end_effector_name="Link6",
            group_name=GROUP_NAME,
        )
        temp_moveit2.planner_id                 = self.planner_id
        temp_moveit2.max_velocity               = self.velocity_scaling
        temp_moveit2.max_acceleration           = self.acceleration_scaling
        temp_moveit2.cartesian_jump_threshold   = self.cartesian_jump_threshold
        temp_moveit2.cartesian_avoid_collisions = self.cartesian_avoid_collisions

        temp_moveit2.move_to_pose(
            position=goal_translation,
            quat_xyzw=goal_quat,
            cartesian=self.cartesian,
            cartesian_max_step=self.cartesian_max_step,
            cartesian_fraction_threshold=self.cartesian_fraction_threshold
        )

        # 3) Verify arrival via servo‐ready polling
        temp_moveit2.wait_until_executed()
        while not self.wait_for_servo_ready(timeout=15.0):
            self.get_logger().warn("gotoEE(): not arrived, rechecking…")
            time.sleep(0.2)

        self.get_logger().info("gotoEE(): Function completed successfully.")

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
    def approach_machine(self, machine_name: str, point_name: str, cartesian_override: bool = True) -> bool:
        """
        Move the portafilter_link to the 'approach_pose' of <machine_name>/<point_name>.
        Also broadcasts a static TF '<machine_name>_<point_name>_approach_test'
        for the computed goal pose, then verifies execution via servo readiness.
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

            # Execute motion
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
    def mount_machine(self, machine_name: str, point_name: str, cartesian_override: bool = True) -> bool:
        """
        Move the portafilter_link to the 'mount_pose' of <machine_name>/<point_name>.
        Also broadcasts a static TF '<machine_name>_<point_name>_mount_test'
        for the computed goal pose, then verifies execution via servo readiness.
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

            # Execute motion
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
        executor.remove_node(node)
        th.join(timeout=1.0)  # Give thread 1s to finish
        node.destroy_node()
        rclpy.shutdown()

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