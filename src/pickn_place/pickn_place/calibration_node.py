#!/usr/bin/env python3

import os
import sys
import math
import threading
import subprocess
import time
import numpy as np
import cv2
import yaml
from datetime import datetime

from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Transform
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformListener

# Import for package install directory
from ament_index_python.packages import get_package_share_directory

# =============================================================================
# Global Configuration Variables
# =============================================================================
# Sampling and stability settings
SAMPLES = 51                        # Total number of samples to collect
NUM_CONSECUTIVE_TF = 10              # Number of consecutive transforms for marker stability check (reduced for faster capture)
TRANSLATION_THRESHOLD = 0.0005       # Translation threshold in meters (0.4mm - relaxed for robot vibrations)
ROTATION_THRESHOLD = 1.0             # Rotation threshold in degrees (relaxed for real-world noise)
LOOKUP_TIME_OFFSET = 2.0            # Seconds offset for transform lookup
MIN_SAMPLES_FOR_CALIBRATION = 10    # Minimum samples before starting incremental validation

# Frame names
DEFAULT_TRACKING_BASE_FRAME = "camera_depth_optical_frame"
DEFAULT_TRACKING_MARKER_FRAME = "calibration_tag"
DEFAULT_ROBOT_BASE_FRAME = "base_link"
DEFAULT_ROBOT_EFFECTOR_FRAME = "Link6"

# File paths for saving calibration data
CALIBRATION_FILEPATH = os.path.expanduser(
    "~/barns_ws/src/pickn_place/share/axab_calibration.yaml"
)
RAW_DATA_FILEPATH = os.path.expanduser(
    "~/barns_ws/src/pickn_place/share/axab_calibration_raw.yaml"
)

# New file path for package install folder
PACKAGE_INSTALL_FILEPATH = os.path.join(get_package_share_directory('pickn_place'), "axab_calibration.yaml")

# Calibration solver option (choose one from: "Tsai-Lenz", "Park", "Horaud", "Andreff", "Daniilidis")
CALIBRATION_ALGORITHM = "Tsai-Lenz"
# =============================================================================

# -----------------------------
# Calibration Backend
# -----------------------------
class CalibrationBackend:
    """
    Uses OpenCV's hand-eye calibration. We compute only once at the end.
    """
    AVAILABLE_ALGORITHMS = {
        'Tsai-Lenz': cv2.CALIB_HAND_EYE_TSAI,
        'Park': cv2.CALIB_HAND_EYE_PARK,
        'Horaud': cv2.CALIB_HAND_EYE_HORAUD,
        'Andreff': cv2.CALIB_HAND_EYE_ANDREFF,
        'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    @staticmethod
    def list_to_opencv(transform):
        # transform: [tx, ty, tz, qx, qy, qz, qw] (meters + quaternion)
        translation = np.array(transform[:3])
        rotation_matrix = Rot.from_quat(transform[3:]).as_matrix()
        return rotation_matrix, translation

    @staticmethod
    def get_opencv_samples(samples_robot, samples_tracking):
        # "robot_samples": transforms from base link to end effector.
        # "tracking_samples": transforms from camera -> tag (consistent usage is all that matters).
        hand_base_rot, hand_base_tr = [], []
        marker_camera_rot, marker_camera_tr = [], []

        for robot_tf, tracking_tf in zip(samples_robot, samples_tracking):
            rM, tM = CalibrationBackend.list_to_opencv(tracking_tf)
            marker_camera_rot.append(rM)
            marker_camera_tr.append(tM)
            rR, tR = CalibrationBackend.list_to_opencv(robot_tf)
            hand_base_rot.append(rR)
            hand_base_tr.append(tR)

        return (hand_base_rot, hand_base_tr), (marker_camera_rot, marker_camera_tr)

    @staticmethod
    def compute_calibration(samples_robot, samples_tracking, algorithm="Tsai-Lenz"):
        # Convert samples to OpenCV's format
        (hand_world_rot, hand_world_tr), (marker_camera_rot, marker_camera_tr) = \
            CalibrationBackend.get_opencv_samples(samples_robot, samples_tracking)
        method = CalibrationBackend.AVAILABLE_ALGORITHMS[algorithm]
        # Perform the calibration
        hand_camera_rot, hand_camera_tr = cv2.calibrateHandEye(
            hand_world_rot, hand_world_tr,
            marker_camera_rot, marker_camera_tr,
            method=method
        )
        # Convert result from rotation matrix to quaternion
        quat = Rot.from_matrix(hand_camera_rot).as_quat()
        qx, qy, qz, qw = [float(val) for val in quat]
        tx, ty, tz = [float(val) for val in hand_camera_tr]

        # Return a 7-element list: translation and quaternion.
        return [tx, ty, tz, qx, qy, qz, qw]

# -----------------------------
# Helper Functions
# -----------------------------
def get_transform(tf_msg: Transform):
    """
    Convert geometry_msgs/Transform to a list [tx, ty, tz, qx, qy, qz, qw].
    """
    t = tf_msg.translation
    q = tf_msg.rotation
    return [float(t.x), float(t.y), float(t.z), float(q.x), float(q.y), float(q.z), float(q.w)]

def compute_translation_rotation_diff(tf1, tf2):
    """
    Compare two transforms in [tx, ty, tz, qx, qy, qz, qw].
    Returns (translation_diff_meters, rotation_diff_degrees).
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
    Compute the maximum translation & rotation difference among a list of transforms.
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
    Compute the average quaternion using the Markley method.
    quaternions: np.array of shape (N, 4), in [qx, qy, qz, qw] format.
    Returns a shape (4,) representing the averaged quaternion.
    """
    # Align quaternions to the first quaternion to avoid double-cover issues.
    aligned = []
    for q in quaternions:
        if np.dot(q, quaternions[0]) < 0:
            aligned.append(-q)
        else:
            aligned.append(q)
    aligned = np.array(aligned)
    
    # Build the symmetric accumulator matrix.
    M = np.zeros((4, 4))
    for q in aligned:
        M += np.outer(q, q)
    M = M / aligned.shape[0]
    
    # Compute eigenvalues and eigenvectors.
    eigenvalues, eigenvectors = np.linalg.eig(M)
    max_index = np.argmax(eigenvalues)
    avg_q = eigenvectors[:, max_index]
    
    # Normalize the quaternion.
    norm = np.linalg.norm(avg_q)
    if norm < 1e-8:
        avg_q = np.array([0., 0., 0., 1.], dtype=np.float32)
    else:
        avg_q = avg_q / norm
    
    return avg_q.astype(np.float32)

def average_transforms(transforms):
    """
    Average a list of transforms in [tx, ty, tz, qx, qy, qz, qw].
    Returns a single transform that is the average of translations and quaternions.
    """
    translations = np.array([t[:3] for t in transforms])
    avg_translation = np.mean(translations, axis=0)
    quaternions = np.array([t[3:] for t in transforms])
    avg_quat = average_quaternions(quaternions)
    return list(avg_translation) + list(avg_quat)

# -----------------------------
# Main Node
# -----------------------------
class HandEyeCalibrationNode(Node):
    def __init__(self):
        super().__init__("hand_eye_calibration_node")

        # Declare and get parameters for tracking and robot frames.
        self.declare_parameter('tracking_base_frame', DEFAULT_TRACKING_BASE_FRAME)
        self.declare_parameter('tracking_marker_frame', DEFAULT_TRACKING_MARKER_FRAME)
        self.declare_parameter('robot_base_frame', DEFAULT_ROBOT_BASE_FRAME)
        self.declare_parameter('robot_effector_frame', DEFAULT_ROBOT_EFFECTOR_FRAME)

        self.tracking_base_frame = self.get_parameter('tracking_base_frame').value
        self.tracking_marker_frame = self.get_parameter('tracking_marker_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.robot_effector_frame = self.get_parameter('robot_effector_frame').value

        # New parameter to control initialization
        self.declare_parameter('initialized', False)
        self.INITIALIZED = self.get_parameter('initialized').value

        # Remove any old raw data file so that no old data is used.
        if os.path.exists(RAW_DATA_FILEPATH):
            try:
                os.remove(RAW_DATA_FILEPATH)
                self.get_logger().info(f"Removed old file: {RAW_DATA_FILEPATH}")
            except Exception as e:
                self.get_logger().warn(f"Failed to remove old file {RAW_DATA_FILEPATH}: {e}")

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create services for capturing data and saving final calibration
        self.capture_service = self.create_service(
            Trigger,
            'capture_point',
            self.capture_callback
        )
        
        self.save_calibration_service = self.create_service(
            Trigger,
            'save_calibration',
            self.save_calibration_callback
        )

        # Store samples in memory (cleared after each calibration cycle).
        self.robot_samples = []
        self.tracking_samples = []
        
        # Store calibration history for incremental validation
        self.calibration_history = []  # List of calibration transforms
        self.current_calibration = None  # Current best calibration [tx, ty, tz, qx, qy, qz, qw]
        self.calibration_error_history = []  # List of errors between consecutive calibrations

        self.get_logger().info("Hand-Eye Calibration Node Started.")

        # Create an independent timer for the support warning message (once at startup)
        self.support_warning_shown = False
        self.create_timer(5.0, self.support_warning_callback)

        # Add a parameter callback to update the 'initialized' parameter.
        self.add_on_set_parameters_callback(self.parameter_callback)

    def support_warning_callback(self):
        # This independent timer logs the support warning once at startup if not initialized.
        if not self.INITIALIZED and not self.support_warning_shown:
            self.get_logger().warn("this is support node for axxb_calibration, run 'ros2 run pickn_place axxb_calibration' instead.")
            self.support_warning_shown = True

    def save_calibration_callback(self, req, resp):
        """
        Service to save current calibration and raw data on demand.
        Called when user types 'done'.
        """
        current_count = len(self.robot_samples)
        
        if current_count < MIN_SAMPLES_FOR_CALIBRATION:
            resp.success = False
            resp.message = f"Not enough samples ({current_count}/{MIN_SAMPLES_FOR_CALIBRATION})"
            return resp
        
        if self.current_calibration is None:
            resp.success = False
            resp.message = "No calibration computed yet"
            return resp
        
        self.get_logger().info("="*70)
        self.get_logger().info(f"💾 SAVING FINAL CALIBRATION")
        self.get_logger().info("="*70)
        self.get_logger().info(f"✅ Total samples used: {current_count}")
        self.get_logger().info(f"✅ Final calibration error: {self.calibration_error_history[-1]:.4f}mm")
        
        # Write raw data
        self.write_raw_data_file()
        
        # Write final calibration
        self.write_final_calibration_file(self.current_calibration)
        
        self.get_logger().info("="*70)
        self.get_logger().info("🎉 CALIBRATION SAVED SUCCESSFULLY!")
        self.get_logger().info("="*70)
        
        resp.success = True
        resp.message = f"Calibration saved with {current_count} samples"
        return resp
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'initialized':
                if param.type_ == Parameter.Type.BOOL:
                    self.INITIALIZED = param.value
        return SetParametersResult(successful=True)

    def capture_callback(self, req, resp):
        """
        Called each time the user triggers a sample capture.
        1) Collect consecutive tracker transforms (for marker stability check).
        2) Check if the marker is stable.
        3) Collect the robot transform (base link → end effector).
        4) Store the calibration point.
        5) Once enough samples are gathered, save raw data to file, compute, and save final calibration.
        """
        # 1) Collect consecutive tracker transforms to verify stability
        tracker_samples = []
        for i in range(NUM_CONSECUTIVE_TF):
            now_loop = self.get_clock().now()
            lookup_time = (now_loop - Duration(seconds=LOOKUP_TIME_OFFSET)).to_msg()
            try:
                # camera_depth_optical_frame -> calibration_tag
                tf_stamped = self.tf_buffer.lookup_transform(
                    self.tracking_base_frame,
                    self.tracking_marker_frame,
                    lookup_time,
                    timeout=Duration(seconds=3.0)
                )
            except Exception as ex:
                error_msg = f"Failed to get tracker transform {i+1}: {ex}"
                self.get_logger().warn(error_msg)
                resp.success = False
                resp.message = error_msg
                return resp

            tracker_samples.append(get_transform(tf_stamped.transform))
            time.sleep(0.1)  # small delay between consecutive checks

        # 2) Check stability
        max_tdiff, max_rdiff = compute_max_spread(tracker_samples)
        if max_tdiff > TRANSLATION_THRESHOLD or max_rdiff > ROTATION_THRESHOLD:
            msg = (f"⚠️ Marker not stable. Max diff: {max_tdiff:.6f}m, {max_rdiff:.6f}°. "
                   f"Thresholds: {TRANSLATION_THRESHOLD}m, {ROTATION_THRESHOLD}°")
            self.get_logger().warn(msg)
            resp.success = False
            resp.message = msg
            return resp

        # Average the consecutive tracker samples
        final_tracker = average_transforms(tracker_samples)

        # 3) Collect the robot transform
        now_robot = self.get_clock().now()
        lookup_time_robot = (now_robot - Duration(seconds=LOOKUP_TIME_OFFSET)).to_msg()

        try:
            # base_link -> Link6
            robot_tf_stamped = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                self.robot_effector_frame,
                lookup_time_robot,
                timeout=Duration(seconds=3.0)
            )
        except Exception as ex:
            error_msg = f"Failed to get robot transform: {ex}"
            self.get_logger().warn(error_msg)
            resp.success = False
            resp.message = error_msg
            return resp

        # 4) TENTATIVELY store the sample
        temp_robot_sample = get_transform(robot_tf_stamped.transform)
        temp_tracking_sample = final_tracker
        
        current_count = len(self.robot_samples)
        
        # 5) Incremental calibration validation (after MIN_SAMPLES_FOR_CALIBRATION samples)
        
        if current_count < MIN_SAMPLES_FOR_CALIBRATION:
            # Just accept the sample (not enough for calibration yet)
            self.robot_samples.append(temp_robot_sample)
            self.tracking_samples.append(temp_tracking_sample)
            current_count = len(self.robot_samples)
            
            progress_bar = "█" * current_count + "░" * (MIN_SAMPLES_FOR_CALIBRATION - current_count)
            self.get_logger().info(f"✓ Sample {current_count}/{MIN_SAMPLES_FOR_CALIBRATION} [{progress_bar}] {int(100*current_count/MIN_SAMPLES_FOR_CALIBRATION)}% (building initial set)")
            
            resp.success = True
            resp.message = f"Sample {current_count} captured. Need {MIN_SAMPLES_FOR_CALIBRATION} for initial calibration."
            return resp
        
        elif current_count == MIN_SAMPLES_FOR_CALIBRATION:
            # First calibration computation (10 samples)
            self.robot_samples.append(temp_robot_sample)
            self.tracking_samples.append(temp_tracking_sample)
            current_count = len(self.robot_samples)
            
            self.get_logger().info("="*70)
            self.get_logger().info(f"🎯 INITIAL CALIBRATION (10 samples)")
            self.get_logger().info("="*70)
            
            new_calibration = CalibrationBackend.compute_calibration(
                self.robot_samples,
                self.tracking_samples,
                algorithm=CALIBRATION_ALGORITHM
            )
            
            self.current_calibration = new_calibration
            self.calibration_history.append(new_calibration)
            
            self.get_logger().info(f"✅ Initial calibration computed!")
            self.get_logger().info(f"   Translation: [{new_calibration[0]:.4f}, {new_calibration[1]:.4f}, {new_calibration[2]:.4f}]m")
            self.get_logger().info(f"   Rotation: [{new_calibration[3]:.4f}, {new_calibration[4]:.4f}, {new_calibration[5]:.4f}, {new_calibration[6]:.4f}]")
            self.get_logger().info("="*70)
            
            resp.success = True
            resp.message = f"Sample {current_count} - Initial calibration complete! Continue for refinement."
            return resp
        
        else:
            # Incremental validation (11+ samples)
            # Temporarily add sample and recompute
            temp_robot_samples = self.robot_samples + [temp_robot_sample]
            temp_tracking_samples = self.tracking_samples + [temp_tracking_sample]
            
            new_calibration = CalibrationBackend.compute_calibration(
                temp_robot_samples,
                temp_tracking_samples,
                algorithm=CALIBRATION_ALGORITHM
            )
            
            # Compute error between new and current calibration
            trans_error = np.linalg.norm(np.array(new_calibration[:3]) - np.array(self.current_calibration[:3]))
            
            # Quaternion distance
            q1 = np.array(new_calibration[3:])
            q2 = np.array(self.current_calibration[3:])
            dot_product = abs(np.dot(q1, q2))
            dot_product = min(1.0, dot_product)  # Clamp to [0, 1]
            rot_error = math.degrees(2 * math.acos(dot_product))
            
            total_error = trans_error * 1000  # Convert to mm for readability
            
            # Decide whether to accept the sample
            if len(self.calibration_error_history) == 0:
                # First refinement sample - accept it as baseline
                accept_sample = True
                comparison = "BASELINE"
            else:
                # Compare to trend
                previous_error = self.calibration_error_history[-1]
                if total_error <= previous_error * 1.5:  # Allow 50% tolerance for noise
                    accept_sample = True
                    if total_error < previous_error:
                        comparison = "IMPROVED ✓"
                    else:
                        comparison = "SIMILAR ≈"
                else:
                    accept_sample = False
                    comparison = "WORSE ✗"
            
            if accept_sample:
                # Accept sample
                self.robot_samples.append(temp_robot_sample)
                self.tracking_samples.append(temp_tracking_sample)
                self.current_calibration = new_calibration
                self.calibration_history.append(new_calibration)
                self.calibration_error_history.append(total_error)
                
                current_count = len(self.robot_samples)
                
                self.get_logger().info("─"*70)
                self.get_logger().info(f"✅ Sample {current_count} ACCEPTED - {comparison}")
                self.get_logger().info(f"   Δ Translation: {trans_error*1000:.4f}mm, Δ Rotation: {rot_error:.4f}°")
                self.get_logger().info(f"   Total Error: {total_error:.4f}mm")
                if len(self.calibration_error_history) > 1:
                    self.get_logger().info(f"   Previous Error: {self.calibration_error_history[-2]:.4f}mm")
                self.get_logger().info("─"*70)
                
                resp.success = True
                resp.message = f"Sample {current_count} accepted - {comparison}"
            else:
                # Reject sample
                prev_error = self.calibration_error_history[-1]
                self.get_logger().warn("─"*70)
                self.get_logger().warn(f"⚠️ Sample REJECTED - {comparison}")
                self.get_logger().warn(f"   New Error: {total_error:.4f}mm vs Previous: {prev_error:.4f}mm")
                self.get_logger().warn(f"   Δ Translation: {trans_error*1000:.4f}mm, Δ Rotation: {rot_error:.4f}°")
                self.get_logger().warn(f"   Sample discarded - current count remains {len(self.robot_samples)}")
                self.get_logger().warn("─"*70)
                
                resp.success = True
                resp.message = f"Sample rejected (error increased). Try a different pose. Current: {len(self.robot_samples)} samples"
            
            return resp

    def reset_calibration_cycle(self):
        """
        Resets the calibration cycle to its initial state.
        This simulates the node being freshly launched.
        """
        # Remove any existing raw data file.
        if os.path.exists(RAW_DATA_FILEPATH):
            try:
                os.remove(RAW_DATA_FILEPATH)
                self.get_logger().info(f"Removed old file: {RAW_DATA_FILEPATH}")
            except Exception as e:
                self.get_logger().warn(f"Failed to remove old file {RAW_DATA_FILEPATH}: {e}")
        # Clear stored samples and history
        self.robot_samples.clear()
        self.tracking_samples.clear()
        self.calibration_history.clear()
        self.calibration_error_history.clear()
        self.current_calibration = None
        self.get_logger().info("Calibration cycle reset. Ready for new samples.")

    def write_raw_data_file(self):
        """
        Write the raw robot and tracking samples to a YAML file in a descriptive format:
          Sample1:
            TF base_link to Link6: {...}
            TF camera_depth_optical_frame to calibration_tag: {...}
          ...
        Ensure each numeric value is converted to regular Python floats.
        """
        data = {"timestamp": datetime.now().isoformat()}

        for i, (robot_sample, tracking_sample) in enumerate(zip(self.robot_samples, self.tracking_samples)):
            sample_key = f"Sample{i+1}"

            # Explicitly convert to regular Python floats (not numpy types):
            rx  = float(round(robot_sample[0], 6))
            ry  = float(round(robot_sample[1], 6))
            rz  = float(round(robot_sample[2], 6))
            rqx = float(round(robot_sample[3], 6))
            rqy = float(round(robot_sample[4], 6))
            rqz = float(round(robot_sample[5], 6))
            rqw = float(round(robot_sample[6], 6))

            tx  = float(round(tracking_sample[0], 6))
            ty  = float(round(tracking_sample[1], 6))
            tz  = float(round(tracking_sample[2], 6))
            tqx = float(round(tracking_sample[3], 6))
            tqy = float(round(tracking_sample[4], 6))
            tqz = float(round(tracking_sample[5], 6))
            tqw = float(round(tracking_sample[6], 6))

            data[sample_key] = {
                f"TF {self.robot_base_frame} to {self.robot_effector_frame}": {
                    "translation": {
                        "x": rx,
                        "y": ry,
                        "z": rz
                    },
                    "rotation": {
                        "x": rqx,
                        "y": rqy,
                        "z": rqz,
                        "w": rqw
                    }
                },
                f"TF {self.tracking_base_frame} to {self.tracking_marker_frame}": {
                    "translation": {
                        "x": tx,
                        "y": ty,
                        "z": tz
                    },
                    "rotation": {
                        "x": tqx,
                        "y": tqy,
                        "z": tqz,
                        "w": tqw
                    }
                }
            }

        try:
            with open(RAW_DATA_FILEPATH, 'w') as f:
                yaml.dump(data, f)
            self.get_logger().info(f"Raw data saved to {RAW_DATA_FILEPATH}")
        except Exception as e:
            self.get_logger().warn(f"Failed to write raw data: {e}")

    def read_raw_data_file(self):
        """
        Read raw data back from the descriptive YAML format and convert
        it into 'raw_robot_samples' and 'raw_tracking_samples', each a list of 7-element transforms.
        """
        try:
            with open(RAW_DATA_FILEPATH, 'r') as f:
                data = yaml.safe_load(f)
            self.get_logger().info(f"Raw data loaded from {RAW_DATA_FILEPATH}")

            raw_robot_samples = []
            raw_tracking_samples = []

            # "timestamp" is top-level, so skip that key. Only parse "Sample1", "Sample2", ...
            for key in data.keys():
                if key.startswith("Sample"):
                    sample_info = data[key]
                    # Robot transform
                    robot_key = f"TF {self.robot_base_frame} to {self.robot_effector_frame}"
                    # Tracking transform
                    tracking_key = f"TF {self.tracking_base_frame} to {self.tracking_marker_frame}"

                    if robot_key not in sample_info or tracking_key not in sample_info:
                        continue

                    robot_tf = sample_info[robot_key]
                    tracking_tf = sample_info[tracking_key]

                    # Robot
                    rx = robot_tf["translation"]["x"]
                    ry = robot_tf["translation"]["y"]
                    rz = robot_tf["translation"]["z"]
                    rqx = robot_tf["rotation"]["x"]
                    rqy = robot_tf["rotation"]["y"]
                    rqz = robot_tf["rotation"]["z"]
                    rqw = robot_tf["rotation"]["w"]
                    raw_robot_samples.append([rx, ry, rz, rqx, rqy, rqz, rqw])

                    # Tracking
                    tx = tracking_tf["translation"]["x"]
                    ty = tracking_tf["translation"]["y"]
                    tz = tracking_tf["translation"]["z"]
                    tqx = tracking_tf["rotation"]["x"]
                    tqy = tracking_tf["rotation"]["y"]
                    tqz = tracking_tf["rotation"]["z"]
                    tqw = tracking_tf["rotation"]["w"]
                    raw_tracking_samples.append([tx, ty, tz, tqx, tqy, tqz, tqw])

            return {
                "raw_robot_samples": raw_robot_samples,
                "raw_tracking_samples": raw_tracking_samples
            }
        except Exception as e:
            self.get_logger().warn(f"Failed to read raw data: {e}")
            return None

    def write_final_calibration_file(self, final_cal_quat):
        """
        Write final calibration to a YAML file.
        New format:
          calibration_transform:
            rotation:         # Quaternion (as computed by the calibration)
              w: <float>
              x: <float>
              y: <float>
              z: <float>
            translation:      # Translation (in meters)
              x: <float>
              y: <float>
              z: <float>
            rotation_degrees: # Euler angles in degrees (derived from the quaternion)
              roll: <float>
              pitch: <float>
              yaw: <float>
          metadata:
            camera_frame: <frame>
            method: ax=xb calibration (hand-eye)
            robot_frame: <frame>
            units:
              rotation: quaternion/degrees
              translation: meter
          timestamp: <ISO timestamp>
        """
        # final_cal_quat: [tx, ty, tz, qx, qy, qz, qw]
        tx, ty, tz, qx, qy, qz, qw = [float(v) for v in final_cal_quat]
        quat = [qx, qy, qz, qw]
        # Convert quaternion to Euler angles (in radians)
        roll, pitch, yaw = Rot.from_quat(quat).as_euler('xyz')
        quat_dict = {
            "w": round(qw, 6),
            "x": round(qx, 6),
            "y": round(qy, 6),
            "z": round(qz, 6)
        }
        translation_dict = {
            "x": round(tx, 6),
            "y": round(ty, 6),
            "z": round(tz, 6)
        }
        rotation_degrees = {
            "roll": round(math.degrees(roll), 6),
            "pitch": round(math.degrees(pitch), 6),
            "yaw": round(math.degrees(yaw), 6)
        }
        data = {
            "calibration_transform": {
                "rotation": quat_dict,
                "translation": translation_dict,
                "rotation_degrees": rotation_degrees
            },
            "metadata": {
                "camera_frame": self.tracking_base_frame,
                "method": "ax=xb calibration (hand-eye)",
                "robot_frame": self.robot_base_frame,
                "units": {"rotation": "quaternion/degrees", "translation": "meter"}
            },
            "timestamp": datetime.now().isoformat()
        }
        try:
            with open(CALIBRATION_FILEPATH, 'w') as f:
                yaml.dump(data, f)
            self.get_logger().info(f"Final calibration saved to {CALIBRATION_FILEPATH}")
        except Exception as e:
            self.get_logger().warn(f"Failed to write final calibration: {e}")
        
        # Save a second copy to the package install folder
        try:
            with open(PACKAGE_INSTALL_FILEPATH, 'w') as f:
                yaml.dump(data, f)
            self.get_logger().info(f"Final calibration also saved to {PACKAGE_INSTALL_FILEPATH}")
        except Exception as e:
            self.get_logger().warn(f"Failed to write final calibration to install folder: {e}")

# -----------------------------
# Keyboard Listener Function
# -----------------------------
def keyboard_listener(node: HandEyeCalibrationNode):
    """
    Allows user to press 'c' to capture.
    """
    client = node.create_client(Trigger, 'capture_point')
    while rclpy.ok():
        user_input = sys.stdin.readline().strip().lower()
        if not user_input:
            continue
        if user_input == 'c':
            if client.wait_for_service(timeout_sec=1.0):
                req = Trigger.Request()
                client.call_async(req)
            else:
                node.get_logger().warn("Service [/capture_point] not available!")

# -----------------------------
# Main Function
# -----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrationNode()

    # Start keyboard listener in a separate thread
    kb_thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    kb_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Already shutdown, ignore

    # Close any OpenCV windows if used
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
