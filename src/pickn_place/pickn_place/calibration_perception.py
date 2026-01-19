#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_transformations
from tf_transformations import (
    quaternion_from_matrix,
    euler_from_quaternion,
    quaternion_from_euler,
)
import numpy as np
import math
from time import time
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

# ================================================================================
# Global Configuration Variables
# ================================================================================
# ArUco detection settings:
DEFAULT_MARKER_SIZE_MM = 60.0  # Marker size in millimeters for the board markers.
DEFAULT_VISUALIZE = True       # Whether to visualize the detection.

# Calibration pose offsets (if all zero, calibration_pose will overlap calibration_tag).
DEFAULT_CALIBRATION_POSE_X = 0.0
DEFAULT_CALIBRATION_POSE_Y = 0.05
DEFAULT_CALIBRATION_POSE_Z = 0.35
DEFAULT_CALIBRATION_POSE_ROLL_DEG = 0.0
DEFAULT_CALIBRATION_POSE_PITCH_DEG = 0.0
DEFAULT_CALIBRATION_POSE_YAW_DEG = 0.0

# Other settings
REPROJECTION_ERROR_THRESHOLD = 0.5
IMAGE_TOPIC = '/camera/color/image_raw'
CAMERA_INFO_TOPIC = '/camera/color/camera_info'
DEPTH_INFO_TOPIC = '/camera/depth/camera_info'
DEPTH_IMAGE_TOPIC = '/camera/depth/image_raw'

# Logging settings
LOG_INTERVAL = 3.0

# ArUco board configuration (for 2x2 markers)
VALID_MARKER_IDS = {1, 2, 3, 4}

# Distance overlay tolerance
MIN_DISTANCE_TOLERANCE = 0.83
MAX_DISTANCE_TOLERANCE = 0.87

# ================================================================================

def average_quaternions(quaternions):
    """
    Compute the average quaternion using the Markley method.
    quaternions: numpy array of shape (N, 4), in [x, y, z, w] format.
    Returns a numpy array of shape (4,) representing the averaged quaternion.
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

class ArucoPerceptionNode(Node):
    def __init__(self):
        super().__init__('aruco_perception_node')

        # ---------------------------
        # 1. Declare ROS parameters
        # ---------------------------
        # Marker size for board markers.
        self.declare_parameter('marker_size_mm', DEFAULT_MARKER_SIZE_MM)
        self.MARKER_SIZE_MM = self.get_parameter('marker_size_mm').get_parameter_value().double_value

        self.declare_parameter('visualize', DEFAULT_VISUALIZE)
        self.VISUALIZE = self.get_parameter('visualize').get_parameter_value().bool_value

        # Calibration pose parameters.
        self.declare_parameter('calibration_pose_x', DEFAULT_CALIBRATION_POSE_X)
        self.declare_parameter('calibration_pose_y', DEFAULT_CALIBRATION_POSE_Y)
        self.declare_parameter('calibration_pose_z', DEFAULT_CALIBRATION_POSE_Z)
        self.declare_parameter('calibration_pose_roll_deg', DEFAULT_CALIBRATION_POSE_ROLL_DEG)
        self.declare_parameter('calibration_pose_pitch_deg', DEFAULT_CALIBRATION_POSE_PITCH_DEG)
        self.declare_parameter('calibration_pose_yaw_deg', DEFAULT_CALIBRATION_POSE_YAW_DEG)
        
        # Toggle parameter for support node initialization.
        self.declare_parameter('initialized', False)
        self.INITIALIZED = self.get_parameter('initialized').get_parameter_value().bool_value

        # -----------------------------------
        # 2. Basic settings and placeholders
        # -----------------------------------
        self.REPROJECTION_ERROR_THRESHOLD = REPROJECTION_ERROR_THRESHOLD
        self.IMAGE_TOPIC = IMAGE_TOPIC
        self.CAMERA_INFO_TOPIC = CAMERA_INFO_TOPIC
        self.DEPTH_INFO_TOPIC = DEPTH_INFO_TOPIC
        self.DEPTH_IMAGE_TOPIC = DEPTH_IMAGE_TOPIC

        self.last_log_time = 0
        self.LOG_INTERVAL = LOG_INTERVAL
        self.bridge = CvBridge()

        # ArUco detection parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 11
        self.aruco_params.adaptiveThreshWinSizeStep = 2
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 5
        self.aruco_params.cornerRefinementMinAccuracy = 0.1
        self.aruco_solver = cv2.SOLVEPNP_ITERATIVE

        # Color camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_width = None
        self.image_height = None
        self.camera_info_received = False
        self.image_received = False

        # Depth camera intrinsics
        self.depth_camera_matrix = None
        self.depth_dist_coeffs = None
        self.depth_width = None
        self.depth_height = None
        self.depth_info_received = False
        self.depth_image_received = False
        self.latest_depth_image = None

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # -------------------------------
        # TF buffer and listener for lookup
        # -------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Storage for last valid transforms for continuous broadcasting.
        self.last_calibration_tag = None
        self.last_calibration_pose = None

        # Timer to broadcast TF continuously (10 Hz)
        self.create_timer(0.1, self.publish_tf)

        # In this example, we expect 4 markers arranged in a 2x2 board.
        self.VALID_MARKER_IDS = VALID_MARKER_IDS

        # Precompute board (object) corner coordinates for each marker.
        half_marker = (self.MARKER_SIZE_MM / 1000.0) / 2.0  # e.g., 0.03 m
        self.board_corners = {
            1: np.array([
                    [-half_marker*2,  half_marker*2, 0.0],
                    [0.0,            half_marker*2, 0.0],
                    [0.0,            0.0,           0.0],
                    [-half_marker*2,  0.0,           0.0],
                ], dtype=np.float32),
            2: np.array([
                    [0.0,            half_marker*2, 0.0],
                    [half_marker*2,  half_marker*2, 0.0],
                    [half_marker*2,  0.0,           0.0],
                    [0.0,            0.0,           0.0],
                ], dtype=np.float32),
            3: np.array([
                    [-half_marker*2,  0.0,           0.0],
                    [0.0,            0.0,           0.0],
                    [0.0,           -half_marker*2, 0.0],
                    [-half_marker*2, -half_marker*2, 0.0],
                ], dtype=np.float32),
            4: np.array([
                    [0.0,            0.0,           0.0],
                    [half_marker*2,  0.0,           0.0],
                    [half_marker*2, -half_marker*2, 0.0],
                    [0.0,           -half_marker*2, 0.0],
                ], dtype=np.float32),
        }
        # Define the overall board outline (the outer border of the 2x2 grid)
        self.board_outline = np.array([
            [-half_marker*2,  half_marker*2, 0.0],
            [ half_marker*2,  half_marker*2, 0.0],
            [ half_marker*2, -half_marker*2, 0.0],
            [-half_marker*2, -half_marker*2, 0.0],
        ], dtype=np.float32)

        # Subscriptions
        self.create_subscription(CameraInfo, self.CAMERA_INFO_TOPIC, self.camera_info_callback, 10)
        self.create_subscription(Image, self.IMAGE_TOPIC, self.image_callback, 10)
        self.create_subscription(CameraInfo, self.DEPTH_INFO_TOPIC, self.depth_info_callback, 10)
        self.create_subscription(Image, self.DEPTH_IMAGE_TOPIC, self.depth_callback, 10)

        self.create_timer(3.0, self.check_input_topics)
        # Create an independent timer for the support warning message (once at startup)
        self.support_warning_shown = False
        self.create_timer(5.0, self.support_warning_callback)

        self.add_on_set_parameters_callback(self.parameter_callback)

        # Create visualization window with initial size 1000x900.
        if self.VISUALIZE:
            cv2.namedWindow("Aruco Board Detection - RGB (top) and Depth (bottom)", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Aruco Board Detection - RGB (top) and Depth (bottom)", 1000, 900)

    def publish_tf(self):
        if self.last_calibration_tag is not None:
            self.tf_broadcaster.sendTransform(self.last_calibration_tag)
        if self.last_calibration_pose is not None:
            self.tf_broadcaster.sendTransform(self.last_calibration_pose)

    def throttled_log(self, message, level="info"):
        current_time = time()
        if current_time - self.last_log_time >= self.LOG_INTERVAL:
            if level == "info":
                self.get_logger().info(message)
            elif level == "warn":
                self.get_logger().warn(message)
            elif level == "error":
                self.get_logger().error(message)
            self.last_log_time = current_time

    def camera_info_callback(self, msg):
        if self.camera_matrix is None and self.dist_coeffs is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.image_width = msg.width
            self.image_height = msg.height
            self.camera_info_received = True
            self.throttled_log("Camera intrinsics and resolution set (color).", "info")

    def depth_info_callback(self, msg):
        if self.depth_camera_matrix is None and self.depth_dist_coeffs is None:
            self.depth_camera_matrix = np.array(msg.k).reshape(3, 3)
            self.depth_dist_coeffs = np.array(msg.d)
            self.depth_width = msg.width
            self.depth_height = msg.height
            self.depth_info_received = True
            self.throttled_log("Depth intrinsics and resolution set (depth).", "info")

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_image = depth_image
            self.depth_image_received = True
        except cv2.error as e:
            self.throttled_log(f"Depth callback error: {e}", "error")

    def check_input_topics(self):
        if not self.camera_info_received:
            self.throttled_log("No color camera info received. Ensure /camera/color/camera_info is publishing.", "warn")
        if not self.image_received:
            self.throttled_log("No color image received. Ensure /camera/color/image_raw is publishing.", "warn")
        if not self.depth_info_received:
            self.throttled_log("No depth camera info received. Ensure /camera/depth/camera_info is publishing.", "warn")
        if not self.depth_image_received:
            self.throttled_log("No depth image received. Ensure /camera/depth/image_raw is publishing.", "warn")

    def support_warning_callback(self):
        # This independent timer logs the support warning once at startup if not initialized.
        if not self.INITIALIZED and not self.support_warning_shown:
            self.get_logger().warn("this is support node for axxb_calibration, run 'ros2 run pickn_place axxb_calibration' instead.")
            self.support_warning_shown = True

    def crop_center(self, frame):
        if self.image_width is None or self.image_height is None:
            return frame

        h, w, _ = frame.shape
        crop_x = min(self.image_width, w)
        crop_y = min(self.image_height, h)

        start_x = (w - crop_x) // 2
        start_y = (h - crop_y) // 2

        return frame[start_y:start_y + crop_y, start_x:start_x + crop_x]

    # ------------------------------------
    # 3D-3D alignment (Horn’s method)
    # ------------------------------------
    def estimate_pose_3D_3D(self, object_points, camera_points):
        """
        object_points: Nx3 in the board's object frame
        camera_points: Nx3 in the camera frame (from depth)
        Returns: R (3x3), t (3,) such that camera_points ≈ R @ object_points + t
        or (None, None) if it fails.
        """
        if object_points.shape[0] < 3:
            return None, None

        # 1) Compute centroids.
        centroid_obj = np.mean(object_points, axis=0)
        centroid_cam = np.mean(camera_points, axis=0)

        # 2) Subtract centroids.
        obj_centered = object_points - centroid_obj
        cam_centered = camera_points - centroid_cam

        # 3) Compute cross-covariance matrix.
        H = obj_centered.T @ cam_centered

        # 4) SVD decomposition.
        U, S, Vt = np.linalg.svd(H)
        R_ = Vt.T @ U.T

        # Correct reflection if needed.
        if np.linalg.det(R_) < 0:
            Vt[2, :] *= -1
            R_ = Vt.T @ U.T

        # 5) Compute translation.
        t_ = centroid_cam - R_ @ centroid_obj
        return R_, t_

    def image_callback(self, msg):
        self.image_received = True
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.throttled_log("Waiting for color camera intrinsics...", "warn")
            return

        # Retrieve the RGB frame and crop if needed.
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = self.crop_center(frame)

        # Prepare a depth visualization image.
        if self.latest_depth_image is not None:
            depth_vis = cv2.normalize(self.latest_depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_vis = np.uint8(depth_vis)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            depth_vis = cv2.resize(depth_vis, (frame.shape[1], frame.shape[0]))
        else:
            depth_vis = np.zeros((frame.shape[0], frame.shape[1], 3), dtype=np.uint8)

        # Detect markers in the RGB frame.
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        # Prepare lists for accumulating 3D–3D correspondences.
        object_points_all = []
        camera_points_all = []
        detected_marker_ids = set()

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in self.VALID_MARKER_IDS:
                    continue

                detected_marker_ids.add(marker_id)
                corners_2d = corners[i][0]  # shape: (4,2)
                
                # --- Overlays for each marker: red circles and green outline ---
                for corner in corners_2d:
                    cv2.circle(frame, (int(round(corner[0])), int(round(corner[1]))), 5, (0, 0, 255), -1)
                cv2.polylines(frame, [np.int32(corners_2d)], True, (0, 255, 0), 2)
                if self.latest_depth_image is not None:
                    for corner in corners_2d:
                        cv2.circle(depth_vis, (int(round(corner[0])), int(round(corner[1]))), 5, (0, 0, 255), -1)
                    cv2.polylines(depth_vis, [np.int32(corners_2d)], True, (0, 255, 0), 2)
                # --- End overlays ---

                if self.latest_depth_image is None:
                    continue

                fx = self.depth_camera_matrix[0, 0]
                fy = self.depth_camera_matrix[1, 1]
                cx = self.depth_camera_matrix[0, 2]
                cy = self.depth_camera_matrix[1, 2]

                camera_3d = []
                valid = True
                for corner in corners_2d:
                    u = int(round(corner[0]))
                    v = int(round(corner[1]))
                    if u < 0 or u >= self.depth_width or v < 0 or v >= self.depth_height:
                        valid = False
                        break
                    raw_depth = self.latest_depth_image[v, u]
                    if raw_depth <= 0:
                        valid = False
                        break
                    Z = float(raw_depth) / 1000.0
                    X = (u - cx) * Z / fx
                    Y = (v - cy) * Z / fy
                    camera_3d.append([X, Y, Z])
                if not valid or len(camera_3d) < 4:
                    continue

                camera_3d = np.array(camera_3d, dtype=np.float32)

                # Get the precomputed board object points for this marker.
                if marker_id in self.board_corners:
                    object_points = self.board_corners[marker_id]
                else:
                    continue

                # Accumulate the 4 correspondences from this marker.
                object_points_all.append(object_points)
                camera_points_all.append(camera_3d)

        # If all 4 markers are detected, perform board pose estimation.
        if detected_marker_ids == self.VALID_MARKER_IDS and len(object_points_all) == 4:
            object_points_total = np.vstack(object_points_all)  # (16,3)
            camera_points_total = np.vstack(camera_points_all)    # (16,3)

            R, t = self.estimate_pose_3D_3D(object_points_total, camera_points_total)
            if R is not None:
                transform_matrix = np.eye(4, dtype=np.float32)
                transform_matrix[:3, :3] = R
                quat = quaternion_from_matrix(transform_matrix)  # [x, y, z, w]
                raw_quat = np.array([-quat[2], quat[1], -quat[0], quat[3]], dtype=np.float32)

                # Create the board transform message as calibration_tag.
                t_msg = TransformStamped()
                t_msg.header.stamp = self.get_clock().now().to_msg()
                # Using 'camera_depth_optical_frame' as the parent frame.
                t_msg.header.frame_id = 'camera_depth_optical_frame'
                t_msg.child_frame_id = 'calibration_tag'
                t_msg.transform.translation.x = float(t[0])
                t_msg.transform.translation.y = float(t[1])
                t_msg.transform.translation.z = float(t[2])
                t_msg.transform.rotation.x = float(raw_quat[0])
                t_msg.transform.rotation.y = float(raw_quat[1])
                t_msg.transform.rotation.z = float(raw_quat[2])
                t_msg.transform.rotation.w = float(raw_quat[3])

                # Compute calibration_pose offset.
                offset_x = self.get_parameter('calibration_pose_x').get_parameter_value().double_value
                offset_y = self.get_parameter('calibration_pose_y').get_parameter_value().double_value
                offset_z = self.get_parameter('calibration_pose_z').get_parameter_value().double_value
                offset_roll_deg  = self.get_parameter('calibration_pose_roll_deg').get_parameter_value().double_value
                offset_pitch_deg = self.get_parameter('calibration_pose_pitch_deg').get_parameter_value().double_value
                offset_yaw_deg   = self.get_parameter('calibration_pose_yaw_deg').get_parameter_value().double_value
                offset_roll  = math.radians(offset_roll_deg)
                offset_pitch = math.radians(offset_pitch_deg)
                offset_yaw   = math.radians(offset_yaw_deg)
                offset_quat = quaternion_from_euler(offset_roll, offset_pitch, offset_yaw)
                t_pose = TransformStamped()
                t_pose.header.stamp = self.get_clock().now().to_msg()
                t_pose.header.frame_id = 'calibration_tag'
                t_pose.child_frame_id = 'calibration_pose'
                t_pose.transform.translation.x = -offset_x
                t_pose.transform.translation.y = offset_y
                t_pose.transform.translation.z = offset_z
                t_pose.transform.rotation.x = float(offset_quat[0])
                t_pose.transform.rotation.y = float(offset_quat[1])
                t_pose.transform.rotation.z = float(offset_quat[2])
                t_pose.transform.rotation.w = float(offset_quat[3])

                self.last_calibration_tag = t_msg
                self.last_calibration_pose = t_pose

                # For visualization: draw the overall board outline and a coordinate axis.
                rvec, _ = cv2.Rodrigues(R)
                board_outline_pts, _ = cv2.projectPoints(self.board_outline, rvec, t, self.camera_matrix, self.dist_coeffs)
                board_outline_pts = board_outline_pts.reshape(-1, 2).astype(int)
                axis_length = 0.05  # 50 mm
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, t, axis_length)
                cv2.drawFrameAxes(depth_vis, self.camera_matrix, self.dist_coeffs, rvec, t, axis_length)

                # -----------------------------------------------------
                # Lookup TF from base_link to calibration_tag and overlay the distance
                # -----------------------------------------------------
                try:
                    # Look up the transform between base_link and calibration_tag.
                    trans = self.tf_buffer.lookup_transform("base_link", "calibration_tag", rclpy.time.Time())
                    dx = trans.transform.translation.x
                    dy = trans.transform.translation.y
                    dz = trans.transform.translation.z
                    distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                    distance_text = f"Distance: {distance:.2f} m"
                    # Overlay the text on the RGB frame.
                    cv2.putText(frame, distance_text, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                    # Decide ROI color based on distance tolerance
                    if MIN_DISTANCE_TOLERANCE <= distance <= MAX_DISTANCE_TOLERANCE:
                        roi_color = (0, 255, 0)  # Green if within tolerance
                    else:
                        roi_color = (0, 0, 255)  # Red if out of tolerance

                except Exception as e:
                    self.throttled_log(f"TF lookup failed: {e}", "warn")
                    roi_color = (0, 0, 255)  # Red ROI color if no distance

            else:
                self.throttled_log("Not all 4 markers detected for board pose estimation.", "warn")
                roi_color = (0, 0, 255)
        else:
            self.throttled_log("Not all 4 markers detected for board pose estimation.", "warn")
            roi_color = (0, 0, 255)

        # --- Composite visualization ---
        if self.VISUALIZE:
            roi_thickness = 3   # Adjust thickness here
            roi_size = 400      # Adjust size here

            center_x = frame.shape[1] // 2
            center_y = frame.shape[0] // 2
            top_left = (center_x - roi_size // 2, center_y - roi_size // 2)
            bottom_right = (center_x + roi_size // 2, center_y + roi_size // 2)

            center_x_d = depth_vis.shape[1] // 2
            center_y_d = depth_vis.shape[0] // 2
            top_left_d = (center_x_d - roi_size // 2, center_y_d - roi_size // 2)
            bottom_right_d = (center_x_d + roi_size // 2, center_y_d + roi_size // 2)

            cv2.rectangle(frame, top_left, bottom_right, roi_color, roi_thickness)
            cv2.rectangle(depth_vis, top_left_d, bottom_right_d, roi_color, roi_thickness)

            # Create the composite image.
            if depth_vis.shape[1] != frame.shape[1]:
                depth_vis = cv2.resize(depth_vis, (frame.shape[1], depth_vis.shape[0]))
            composite = np.vstack((frame, depth_vis))

            cv2.imshow("Aruco Board Detection - RGB (top) and Depth (bottom)", composite)
            cv2.waitKey(1)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'visualize':
                if param.type_ == Parameter.Type.BOOL:
                    self.VISUALIZE = param.value
            elif param.name in [
                'calibration_pose_x',
                'calibration_pose_y',
                'calibration_pose_z',
                'calibration_pose_roll_deg',
                'calibration_pose_pitch_deg',
                'calibration_pose_yaw_deg',
            ]:
                pass
            elif param.name == 'initialized':
                if param.type_ == Parameter.Type.BOOL:
                    self.INITIALIZED = param.value 
        return SetParametersResult(successful=True)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
