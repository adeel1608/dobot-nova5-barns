#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from transformations import (
    quaternion_from_matrix,
    euler_from_quaternion,
    quaternion_from_euler,
)
import numpy as np
import math
from time import time
import tf2_ros
from rcl_interfaces.msg import SetParametersResult
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger  # <-- Imported Trigger service
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# =============================================================================
# Global Configuration Variables
# =============================================================================
# General visualization and logging parameters
DEFAULT_VISUALIZE = True
DEFAULT_LOG_INTERVAL = 5
CAM_NAME = os.environ.get('CAM_NAME', 'camera')
# Camera and topic settings
DEFAULT_REPROJECTION_ERROR_THRESHOLD = 0.5
DEFAULT_CAMERA_FRAME = 'camera_depth_optical_frame'
DEFAULT_IMAGE_TOPIC = f"/{CAM_NAME}/color/image_raw"
DEFAULT_CAMERA_INFO_TOPIC = f"/{CAM_NAME}/color/camera_info"
DEFAULT_DEPTH_INFO_TOPIC = f"/{CAM_NAME}/depth/camera_info"
DEFAULT_DEPTH_IMAGE_TOPIC = f"/{CAM_NAME}/depth/image_raw"
DEFAULT_EXTRINSICS_TOPIC = f"/{CAM_NAME}/depth_to_color"

# ArUco detection parameters
DEFAULT_ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
DEFAULT_ARUCO_PARAMS = cv2.aruco.DetectorParameters()
DEFAULT_ARUCO_PARAMS.adaptiveThreshWinSizeMin = 3
DEFAULT_ARUCO_PARAMS.adaptiveThreshWinSizeMax = 11
DEFAULT_ARUCO_PARAMS.adaptiveThreshWinSizeStep = 2
DEFAULT_ARUCO_PARAMS.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
DEFAULT_ARUCO_PARAMS.cornerRefinementWinSize = 5
DEFAULT_ARUCO_PARAMS.cornerRefinementMinAccuracy = 0.1
DEFAULT_ARUCO_SOLVER = cv2.SOLVEPNP_ITERATIVE

# Timer intervals (seconds)
DEFAULT_TF_BROADCAST_INTERVAL = 0.1
DEFAULT_CALIBRATION_TF_INTERVAL = 0.1
DEFAULT_CALIBRATION_FILE_CHECK_INTERVAL = 2.0
DEFAULT_POSE_SAVE_INTERVAL = 0.035

# QoS profile for sensor data (BEST_EFFORT reliability to match camera)
SENSOR_DATA_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

# Calibration file and configuration settings
PACKAGE_NAME = 'pickn_place'
DEFAULT_CALIBRATION_FILENAME = 'axab_calibration.yaml'
DEFAULT_ARUCO_SIZE_CONFIG_FILE = 'aruco_size_config.yaml'
DEFAULT_ARUCO_ID_NAME_CONFIG_FILE = 'arucoID_name_config.yaml'
DEFAULT_POSE_DATA_FILENAME = 'pose_data_memory.yaml'
# =============================================================================

def average_quaternions(quaternions):
    """
    Compute the average quaternion using the Markley method.
    quaternions: numpy array of shape (N, 4), in [x, y, z, w] format.
    Returns a numpy array of shape (4,) representing the averaged quaternion.
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
    M = M / aligned.shape[0]
    
    eigenvalues, eigenvectors = np.linalg.eig(M)
    max_index = np.argmax(eigenvalues)
    avg_q = eigenvectors[:, max_index]
    
    norm = np.linalg.norm(avg_q)
    if norm < 1e-8:
        avg_q = np.array([0., 0., 0., 1.], dtype=np.float32)
    else:
        avg_q = avg_q / norm
    
    return avg_q.astype(np.float32)


def quaternion_to_matrix(q):
    """
    Convert a quaternion (x, y, z, w) into a 3x3 rotation matrix.
    """
    x, y, z, w = q
    xx = x * x; yy = y * y; zz = z * z
    xy = x * y; xz = x * z; yz = y * z
    wx = w * x; wy = w * y; wz = w * z

    return np.array([
        [1 - 2 * (yy + zz),     2 * (xy - wz),       2 * (xz + wy)],
        [2 * (xy + wz),         1 - 2 * (xx + zz),   2 * (yz - wx)],
        [2 * (xz - wy),         2 * (yz + wx),       1 - 2 * (xx + yy)]
    ], dtype=np.float32)


class ArucoPerceptionNode(Node):
    def __init__(self):
        robot_id = os.environ.get('ROBOT_ID', '1')
        super().__init__(f'aruco_perception_node_robot_{robot_id}')
        self.get_logger().info(f"Starting ArUco Perception for Robot {robot_id}")

        # ---------------------------
        # 1. Declare ROS parameters
        # ---------------------------
        self.declare_parameter('visualize', DEFAULT_VISUALIZE)
        self.VISUALIZE = self.get_parameter('visualize').get_parameter_value().bool_value

        # -----------------------------------
        # 2. Basic settings and placeholders
        # -----------------------------------
        self.REPROJECTION_ERROR_THRESHOLD = DEFAULT_REPROJECTION_ERROR_THRESHOLD
        self.CAMERA_FRAME = DEFAULT_CAMERA_FRAME

        self.IMAGE_TOPIC = DEFAULT_IMAGE_TOPIC
        self.CAMERA_INFO_TOPIC = DEFAULT_CAMERA_INFO_TOPIC
        self.DEPTH_INFO_TOPIC = DEFAULT_DEPTH_INFO_TOPIC
        self.DEPTH_IMAGE_TOPIC = DEFAULT_DEPTH_IMAGE_TOPIC
        self.EXTRINSICS_TOPIC = DEFAULT_EXTRINSICS_TOPIC

        self.last_log_time = 0
        self.LOG_INTERVAL = DEFAULT_LOG_INTERVAL
        self.bridge = CvBridge()

        # ArUco detection parameters
        self.aruco_dict = DEFAULT_ARUCO_DICT
        self.aruco_params = DEFAULT_ARUCO_PARAMS
        self.aruco_solver = DEFAULT_ARUCO_SOLVER

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

        # Extrinsics from depth -> color
        self.depth_to_color_R = np.eye(3, dtype=np.float32)
        self.depth_to_color_t = np.zeros((3, 1), dtype=np.float32)

        # Create a TF broadcaster (for calibration and other TFs)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timers
        self.create_timer(DEFAULT_TF_BROADCAST_INTERVAL, self.publish_tf)
        self.create_timer(DEFAULT_CALIBRATION_TF_INTERVAL, self.publish_calibration_tf)
        self.create_timer(DEFAULT_CALIBRATION_FILE_CHECK_INTERVAL, self.check_calibration_file)

        # Load calibration file.
        self.calib_loaded = True
        try:
            package_share_dir = get_package_share_directory(PACKAGE_NAME)
            calib_file = os.path.join(package_share_dir, DEFAULT_CALIBRATION_FILENAME)
            with open(calib_file, 'r') as f:
                self.calib_data = yaml.safe_load(f)
            calib_transform_data = self.calib_data.get("calibration_transform", {})
            self.calib_quat = np.array([
                calib_transform_data["rotation"]["x"],
                calib_transform_data["rotation"]["y"],
                calib_transform_data["rotation"]["z"],
                calib_transform_data["rotation"]["w"]
            ], dtype=np.float32)
            self.calib_trans = np.array([
                calib_transform_data["translation"]["x"],
                calib_transform_data["translation"]["y"],
                calib_transform_data["translation"]["z"]
            ], dtype=np.float32)
            self.T_calib = np.eye(4, dtype=np.float32)
            self.T_calib[:3, :3] = quaternion_to_matrix(self.calib_quat)
            self.T_calib[:3, 3] = self.calib_trans
        except Exception as e:
            self.get_logger().error(f"Error loading calibration file: {e}")
            self.calib_loaded = False
            self.calib_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
            self.calib_trans = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        # Load marker configuration files from package share.
        try:
            package_share_dir = get_package_share_directory(PACKAGE_NAME)
            size_config_file = os.path.join(package_share_dir, DEFAULT_ARUCO_SIZE_CONFIG_FILE)
            id_name_config_file = os.path.join(package_share_dir, DEFAULT_ARUCO_ID_NAME_CONFIG_FILE)
            with open(size_config_file, 'r') as f:
                self.size_config = yaml.safe_load(f)
            with open(id_name_config_file, 'r') as f:
                self.id_name_config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Error loading marker configuration files: {e}")
            self.size_config = {"aruco_markers": []}
            self.id_name_config = {"aruco_id": []}

        # Build marker name mapping from configuration.
        self.marker_name_mapping = {}
        for item in self.id_name_config.get("aruco_id", []):
            self.marker_name_mapping[item["id"]] = item["name"]

        # Subscriptions with sensor_data QoS profile
        self.create_subscription(CameraInfo, self.CAMERA_INFO_TOPIC, self.camera_info_callback, SENSOR_DATA_QOS)
        self.create_subscription(Image, self.IMAGE_TOPIC, self.image_callback, SENSOR_DATA_QOS)
        self.create_subscription(CameraInfo, self.DEPTH_INFO_TOPIC, self.depth_info_callback, SENSOR_DATA_QOS)
        self.create_subscription(Image, self.DEPTH_IMAGE_TOPIC, self.depth_callback, SENSOR_DATA_QOS)
        self.create_subscription(TransformStamped, self.EXTRINSICS_TOPIC, self.extrinsics_callback, SENSOR_DATA_QOS)

        self.create_timer(5.0, self.check_input_topics)

        # Storage for last valid marker transforms for continuous broadcasting.
        self.last_marker_transforms = {}

        # File path for saving marker pose data (unique per robot)
        package_share_dir = get_package_share_directory(PACKAGE_NAME)
        pose_filename = f"pose_data_memory_robot_{robot_id}.yaml"
        self.pose_data_file = os.path.join(package_share_dir, pose_filename)

        # Throttling parameters for saving pose data.
        self.pose_save_interval = DEFAULT_POSE_SAVE_INTERVAL
        self.last_pose_save_time = 0.0

        # Toggle for writing data (default True)
        self.save_data = True

        # Create a service to toggle the data saving.
        self.toggle_service = self.create_service(Trigger, 'toggle_TF', self.toggle_tf_callback)

        # Create and resize the OpenCV window only once upon startup.
        if self.VISUALIZE:
            try:
                cv2.namedWindow("Aruco Detection - RGB (top) and Depth (bottom)", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("Aruco Detection - RGB (top) and Depth (bottom)", 900, 1000)
            except Exception as e:
                self.get_logger().warn(f"Could not create OpenCV window (headless mode?): {e}")
                self.VISUALIZE = False

    def toggle_tf_callback(self, request, response):
        """
        Toggles the saving of marker pose data to the memory file.
        """
        self.save_data = not self.save_data
        response.success = True
        response.message = f"Data saving toggled to {self.save_data}"
        return response

    def publish_tf(self):
        # Broadcast all stored marker transforms.
        for transform in self.last_marker_transforms.values():
            self.tf_broadcaster.sendTransform(transform)

    def publish_calibration_tf(self):
        t_msg = TransformStamped()
        t_msg.header.stamp = self.get_clock().now().to_msg()
        t_msg.header.frame_id = "Link6"
        t_msg.child_frame_id = "calibrated_camera_link"
        t_msg.transform.translation.x = float(self.calib_trans[0])
        t_msg.transform.translation.y = float(self.calib_trans[1])
        t_msg.transform.translation.z = float(self.calib_trans[2])
        t_msg.transform.rotation.x = float(self.calib_quat[0])
        t_msg.transform.rotation.y = float(self.calib_quat[1])
        t_msg.transform.rotation.z = float(self.calib_quat[2])
        t_msg.transform.rotation.w = float(self.calib_quat[3])
        self.tf_broadcaster.sendTransform(t_msg)

    def check_calibration_file(self):
        if not self.calib_loaded:
            self.get_logger().error("Calibration file not found")

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

    def extrinsics_callback(self, msg):
        tx = msg.transform.translation.x
        ty = msg.transform.translation.y
        tz = msg.transform.translation.z

        qx = msg.transform.rotation.x
        qy = msg.transform.rotation.y
        qz = msg.transform.rotation.z
        qw = msg.transform.rotation.w

        self.depth_to_color_t = np.array([[tx], [ty], [tz]], dtype=np.float32)
        self.get_logger().info(
            f"Depth->Color extrinsics updated: "
            f"R=[qx={qx},qy={qy},qz={qz},qw={qw}] T=({tx},{ty},{tz})"
        )

    def check_input_topics(self):
        if not self.camera_info_received:
            self.throttled_log(f"No color camera info received. Ensure {self.CAMERA_INFO_TOPIC} is publishing.", "warn")
        if not self.image_received:
            self.throttled_log(f"No color image received. Ensure {self.IMAGE_TOPIC} is publishing.", "warn")
        if not self.depth_info_received:
            self.throttled_log(f"No depth camera info received. Ensure {self.DEPTH_INFO_TOPIC} is publishing.", "warn")
        if not self.depth_image_received:
            self.throttled_log(f"No depth image received. Ensure {self.DEPTH_IMAGE_TOPIC} is publishing.", "warn")

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
    # 3D-3D alignment (Horn's method)
    # ------------------------------------
    def estimate_pose_3D_3D(self, object_points, camera_points):
        if object_points.shape[0] < 3:
            return None, None

        centroid_obj = np.mean(object_points, axis=0)
        centroid_cam = np.mean(camera_points, axis=0)

        obj_centered = object_points - centroid_obj
        cam_centered = camera_points - centroid_cam

        H = obj_centered.T @ cam_centered

        U, S, Vt = np.linalg.svd(H)
        R_ = Vt.T @ U.T

        if np.linalg.det(R_) < 0:
            Vt[2, :] *= -1
            R_ = Vt.T @ U.T

        t_ = centroid_cam - R_ @ centroid_obj
        return R_, t_

    def save_pose_data(self, new_pose_data):
        # Load full YAML, defaulting to empty dict
        full_data = {}
        if os.path.exists(self.pose_data_file):
            try:
                with open(self.pose_data_file, 'r') as f:
                    full_data = yaml.safe_load(f) or {}
            except Exception as e:
                self.get_logger().error(f"Error reading existing pose data: {e}")

        # Merge only under the `poses:` key
        poses = full_data.get('poses', {})
        poses.update(new_pose_data)
        full_data['poses'] = poses

        # Write back out
        try:
            with open(self.pose_data_file, 'w') as f:
                yaml.dump(full_data, f, default_flow_style=False)
        except Exception as e:
            self.get_logger().error(f"Failed to save pose data: {e}")

    def image_callback(self, msg):
        self.image_received = True
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.throttled_log("Waiting for color camera intrinsics...", "warn")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = self.crop_center(frame)

        if self.latest_depth_image is not None:
            depth_normalized = cv2.normalize(self.latest_depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_vis = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            depth_vis = cv2.resize(depth_vis, (frame.shape[1], frame.shape[0]))
        else:
            depth_vis = np.zeros((frame.shape[0], frame.shape[1], 3), dtype=np.uint8)

        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        valid_marker_transforms = {}

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                marker_id = int(marker_id)
                marker_size_mm = None
                for entry in self.size_config.get("aruco_markers", []):
                    low, high = entry["id_range"]
                    if low <= marker_id <= high:
                        marker_size_mm = entry["size_mm"]
                        break
                if marker_size_mm is None:
                    marker_size_mm = 100
                marker_size = marker_size_mm / 1000.0

                corners_2d = corners[i][0]

                if self.latest_depth_image is None:
                    continue

                fx = self.depth_camera_matrix[0, 0]
                fy = self.depth_camera_matrix[1, 1]
                cx = self.depth_camera_matrix[0, 2]
                cy = self.depth_camera_matrix[1, 2]

                local_3d = np.array([
                    [-marker_size/2,  marker_size/2, 0.0],
                    [ marker_size/2,  marker_size/2, 0.0],
                    [ marker_size/2, -marker_size/2, 0.0],
                    [-marker_size/2, -marker_size/2, 0.0],
                ], dtype=np.float32)

                camera_3d = []
                valid_3d_corners = True

                for corner_2d in corners_2d:
                    u = int(round(corner_2d[0]))
                    v = int(round(corner_2d[1]))
                    if u < 0 or u >= self.depth_width or v < 0 or v >= self.depth_height:
                        valid_3d_corners = False
                        break
                    raw_depth = self.latest_depth_image[v, u]
                    if raw_depth <= 0:
                        valid_3d_corners = False
                        break
                    Z = float(raw_depth) / 1000.0
                    X = (u - cx) * Z / fx
                    Y = (v - cy) * Z / fy
                    camera_3d.append([X, Y, Z])

                if not valid_3d_corners or len(camera_3d) < 4:
                    continue

                camera_3d = np.array(camera_3d, dtype=np.float32)
                R, t = self.estimate_pose_3D_3D(local_3d, camera_3d)
                if R is None:
                    continue

                transform_matrix = np.eye(4, dtype=np.float32)
                transform_matrix[:3, :3] = R
                quat = quaternion_from_matrix(transform_matrix)  # [x, y, z, w]
                raw_quat = np.array([-quat[2], quat[1], -quat[0], quat[3]], dtype=np.float32)

                if marker_id in self.marker_name_mapping:
                    if marker_id not in valid_marker_transforms:
                        valid_marker_transforms[marker_id] = {"positions": [], "orientations": []}
                    valid_marker_transforms[marker_id]["positions"].append(t)
                    valid_marker_transforms[marker_id]["orientations"].append(raw_quat)

                for corner in corners_2d:
                    cv2.circle(frame, (int(round(corner[0])), int(round(corner[1]))),
                               5, (0, 0, 255), -1)
                cv2.polylines(frame, [np.int32(corners_2d)], True, (0, 255, 0), 2)
                rvec, _ = cv2.Rodrigues(R)
                tvec = t.reshape((3, 1))
                axis_length = marker_size * 0.5
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, axis_length)
                center_2d = np.mean(corners_2d, axis=0).astype(int)
                raw_depth_center = 0
                if (0 <= center_2d[0] < self.depth_width) and (0 <= center_2d[1] < self.depth_height):
                    raw_depth_center = self.latest_depth_image[center_2d[1], center_2d[0]]
                marker_name = self.marker_name_mapping.get(marker_id, f"ID {marker_id}")
                text = f"[{marker_name} (ID {marker_id})] Dist: {raw_depth_center:.1f} mm"
                cv2.putText(frame, text, (center_2d[0], center_2d[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                for corner in corners_2d:
                    cv2.circle(depth_vis, (int(round(corner[0])), int(round(corner[1]))),
                               5, (0, 0, 255), -1)
                cv2.polylines(depth_vis, [np.int32(corners_2d)], True, (0, 255, 0), 2)
                cv2.drawFrameAxes(depth_vis, self.camera_matrix, self.dist_coeffs, rvec, tvec, axis_length)
                cv2.putText(depth_vis, text, (center_2d[0], center_2d[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if valid_marker_transforms and self.save_data:
            pose_data = {}
            timestamp = str(time())
            for m_id, data in valid_marker_transforms.items():
                avg_t = np.mean(data["positions"], axis=0)
                avg_q = average_quaternions(np.array(data["orientations"]))
                pose_data[self.marker_name_mapping[m_id]] = {
                    'Time': timestamp,
                    'rotation': {
                        'w': float(avg_q[3]),
                        'x': float(avg_q[0]),
                        'y': float(avg_q[1]),
                        'z': float(avg_q[2]),
                    },
                    'translation': {
                        'x': float(avg_t[0]),
                        'y': float(avg_t[1]),
                        'z': float(avg_t[2]),
                    }
                }
            current_time = time()
            if current_time - self.last_pose_save_time >= self.pose_save_interval:
                self.save_pose_data(pose_data)
                self.last_pose_save_time = current_time

        if self.VISUALIZE:
            roi_thickness = 3
            roi_size = 400

            center_x = frame.shape[1] // 2
            center_y = frame.shape[0] // 2
            top_left = (center_x - roi_size // 2, center_y - roi_size // 2)
            bottom_right = (center_x + roi_size // 2, center_y + roi_size // 2)
            cv2.rectangle(frame, top_left, bottom_right, (255, 0, 0), roi_thickness)

            center_x_d = depth_vis.shape[1] // 2
            center_y_d = depth_vis.shape[0] // 2
            top_left_d = (center_x_d - roi_size // 2, center_y_d - roi_size // 2)
            bottom_right_d = (center_x_d + roi_size // 2, center_y_d + roi_size // 2)
            cv2.rectangle(depth_vis, top_left_d, bottom_right_d, (255, 0, 0), roi_thickness)

            if depth_vis.shape[1] != frame.shape[1]:
                depth_vis = cv2.resize(depth_vis, (frame.shape[1], depth_vis.shape[0]))
            composite = np.vstack((frame, depth_vis))
            cv2.imshow("Aruco Detection - RGB (top) and Depth (bottom)", composite)
            cv2.waitKey(1)

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
