#!/usr/bin/env python3
"""
Aruco perception node with sliding-window averaging and immediate TF publish:
• Configurable sample window size (default 5)
• Use deque(maxlen=N) per marker for sliding-buffer
• On each new detection, append; if buffer full, average & broadcast immediately
• Calibration TF still broadcast at 100 Hz
"""
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import numpy as np
from time import time
import tf2_ros
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from transformations import quaternion_from_matrix
from collections import deque

# ─── Constants ────────────────────────────────────────────────────────────────
DEFAULT_VISUALIZE       = True
DEFAULT_SAMPLE_WINDOW   = 6
DEFAULT_LOG_INTERVAL    = 5.0

DEFAULT_IMAGE_TOPIC         = '/camera/color/image_raw'
DEFAULT_CAMERA_INFO_TOPIC   = '/camera/color/camera_info'
DEFAULT_DEPTH_IMAGE_TOPIC   = '/camera/depth/image_raw'
DEFAULT_DEPTH_INFO_TOPIC    = '/camera/depth/camera_info'
DEFAULT_EXTRINSICS_TOPIC    = '/camera/depth_to_color'

ARUCO_DICT   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
ARUCO_PARAMS.adaptiveThreshWinSizeMin    = 3
ARUCO_PARAMS.adaptiveThreshWinSizeMax    = 11
ARUCO_PARAMS.adaptiveThreshWinSizeStep   = 2
ARUCO_PARAMS.cornerRefinementMethod      = cv2.aruco.CORNER_REFINE_SUBPIX
ARUCO_PARAMS.cornerRefinementWinSize     = 5
ARUCO_PARAMS.cornerRefinementMinAccuracy = 0.1

CALIBRATION_TF_INT    = 0.01  # seconds → 100 Hz
PACKAGE_NAME          = 'pickn_place'
CALIB_FILE            = 'axab_calibration.yaml'
ID_NAME_CONFIG_FILE   = 'arucoID_name_config.yaml'
MARKER_SIZE_MM        = 100   # fixed model size for compatibility

# ─── Helpers ────────────────────────────────────────────────────────────────
def quaternion_to_matrix(q):
    x, y, z, w = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1-2*(yy+zz),   2*(xy-wz),     2*(xz+wy)],
        [2*(xy+wz),     1-2*(xx+zz),   2*(yz-wx)],
        [2*(xz-wy),     2*(yz+wx),     1-2*(xx+yy)]
    ], dtype=np.float32)

def average_quaternions(qs):
    M = np.zeros((4,4), dtype=np.float64)
    for q in qs:
        if np.dot(qs[0], q) < 0:
            q = -q
        M += np.outer(q, q)
    M /= qs.shape[0]
    vals, vecs = np.linalg.eig(M)
    q_avg = vecs[:, np.argmax(vals)]
    return (q_avg / np.linalg.norm(q_avg)).astype(np.float32)

# ─── Node ─────────────────────────────────────────────────────────────────
class ArucoPerceptionNode(Node):
    def __init__(self):
        super().__init__('aruco_perception_node')

        # Params
        self.declare_parameter('visualize', DEFAULT_VISUALIZE)
        self.declare_parameter('sample_window_size', DEFAULT_SAMPLE_WINDOW)
        self.VISUALIZE     = self.get_parameter('visualize').get_parameter_value().bool_value
        self.sample_window = self.get_parameter('sample_window_size').get_parameter_value().integer_value

        # Logging throttle
        self.last_log_time = 0.0

        # Bridge
        self.bridge = CvBridge()

        # Intrinsics flags
        self.camera_info_received = False
        self.depth_info_received  = False
        self.image_received       = False
        self.depth_image_received = False

        # Latest frames
        self.latest_depth_image = None

        # TF broadcaster & buffers
        self.tf_broadcaster        = tf2_ros.TransformBroadcaster(self)
        self.last_marker_transforms = {}  # id → TransformStamped
        self.marker_samples         = {}  # id → {'positions': deque, 'orientations': deque}

        # Timers
        self.create_timer(CALIBRATION_TF_INT, self.publish_calibration_tf)
        self.create_timer(DEFAULT_LOG_INTERVAL, self.check_input_topics)

        # Load static calibration
        pkg_dir = get_package_share_directory(PACKAGE_NAME)
        try:
            with open(os.path.join(pkg_dir, CALIB_FILE), 'r') as f:
                ct = yaml.safe_load(f)['calibration_transform']
            self.calib_quat  = np.array([ct['rotation'][k] for k in ('x','y','z','w')], dtype=np.float32)
            self.calib_trans = np.array([ct['translation'][k] for k in ('x','y','z')], dtype=np.float32)
            self.calib_loaded = True
        except Exception as e:
            self.get_logger().error(f"Calibration load error: {e}")
            self.calib_quat  = np.array([0,0,0,1], dtype=np.float32)
            self.calib_trans = np.zeros(3, dtype=np.float32)
            self.calib_loaded = False

        # Load ID→name mapping
        try:
            with open(os.path.join(pkg_dir, ID_NAME_CONFIG_FILE), 'r') as f:
                items = yaml.safe_load(f).get('aruco_id', [])
        except Exception as e:
            self.get_logger().error(f"ID→name load error: {e}")
            items = []
        self.marker_name_mapping = {it['id']: it['name'] for it in items}

        # Subscriptions
        self.create_subscription(CameraInfo, DEFAULT_CAMERA_INFO_TOPIC, self.camera_info_callback, 10)
        self.create_subscription(Image,      DEFAULT_IMAGE_TOPIC,      self.image_callback,      10)
        self.create_subscription(CameraInfo, DEFAULT_DEPTH_INFO_TOPIC, self.depth_info_callback,10)
        self.create_subscription(Image,      DEFAULT_DEPTH_IMAGE_TOPIC,self.depth_callback,     10)
        self.create_subscription(TransformStamped, DEFAULT_EXTRINSICS_TOPIC, self.extrinsics_callback,10)

        # Visualization
        if self.VISUALIZE:
            cv2.namedWindow("Aruco Detection - RGB (top) and Depth (bottom)", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Aruco Detection - RGB (top) and Depth (bottom)", 900, 1000)

    # ─── Logging helper ─────────────────────────────────────────────────────
    def throttled_log(self, msg, level="info"):
        if time() - self.last_log_time >= DEFAULT_LOG_INTERVAL:
            getattr(self.get_logger(), level)(msg)
            self.last_log_time = time()

    # ─── Callbacks ─────────────────────────────────────────────────────────
    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3,3)
            self.dist_coeffs   = np.array(msg.d)
            self.image_width, self.image_height = msg.width, msg.height
            self.camera_info_received = True
            self.throttled_log("Color camera intrinsics received.")

    def depth_info_callback(self, msg: CameraInfo):
        if not self.depth_info_received:
            self.depth_camera_matrix = np.array(msg.k).reshape(3,3)
            self.depth_dist_coeffs   = np.array(msg.d)
            self.depth_width, self.depth_height = msg.width, msg.height
            self.depth_info_received = True
            self.throttled_log("Depth camera intrinsics received.")

    def depth_callback(self, msg: Image):
        self.depth_image_received = True
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except cv2.error as e:
            self.throttled_log(f"Depth callback error: {e}", "error")

    def extrinsics_callback(self, msg: TransformStamped):
        t = msg.transform.translation
        self.depth_to_color_t = np.array([[t.x],[t.y],[t.z]], dtype=np.float32)
        self.get_logger().info("Depth→Color extrinsics updated.")

    def check_input_topics(self):
        if not self.camera_info_received:
            self.throttled_log("No color camera info received.", "warn")
        if not self.image_received:
            self.throttled_log("No color image received.", "warn")
        if not self.depth_info_received:
            self.throttled_log("No depth camera info received.", "warn")
        if not self.depth_image_received:
            self.throttled_log("No depth image received.", "warn")

    # ─── Main image callback ─────────────────────────────────────────────────
    def image_callback(self, msg: Image):
        self.image_received = True

        # Guards
        if not self.camera_info_received:
            self.throttled_log("Waiting for color camera intrinsics...", "warn")
            return
        if not self.depth_info_received:
            self.throttled_log("Waiting for depth camera intrinsics...", "warn")
            return
        if self.latest_depth_image is None:
            self.throttled_log("Waiting for first depth image...", "warn")
            return

        # Convert & crop
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = self.crop_center(frame)

        # Depth visualization
        depth_vis = cv2.normalize(self.latest_depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_vis = cv2.applyColorMap(depth_vis.astype(np.uint8), cv2.COLORMAP_JET)
        depth_vis = cv2.resize(depth_vis, (frame.shape[1], frame.shape[0]))

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(frame, ARUCO_DICT, parameters=ARUCO_PARAMS)
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                mid = int(mid)

                # Sliding-window buffers
                buf = self.marker_samples.setdefault(
                    mid,
                    {
                        'positions': deque(maxlen=self.sample_window),
                        'orientations': deque(maxlen=self.sample_window)
                    }
                )

                # Build object & camera point sets
                s = MARKER_SIZE_MM / 1000.0
                obj_pts = np.array([
                    [-s/2, s/2, 0], [ s/2, s/2, 0],
                    [ s/2,-s/2, 0], [-s/2,-s/2, 0]
                ], dtype=np.float32)
                cam_pts = []
                valid = True
                for (uf, vf) in corners[i][0]:
                    u, v = int(round(uf)), int(round(vf))
                    if not (0 <= u < self.depth_width and 0 <= v < self.depth_height):
                        valid = False; break
                    d = float(self.latest_depth_image[v, u]) / 1000.0
                    if d <= 0:
                        valid = False; break
                    fx, fy = self.depth_camera_matrix[0,0], self.depth_camera_matrix[1,1]
                    cx, cy = self.depth_camera_matrix[0,2], self.depth_camera_matrix[1,2]
                    X = (u - cx) * d / fx
                    Y = (v - cy) * d / fy
                    cam_pts.append([X, Y, d])
                if not valid or len(cam_pts) < 4:
                    continue
                cam_pts = np.array(cam_pts, dtype=np.float32)

                # Horn’s method for pose
                co, cc = obj_pts.mean(axis=0), cam_pts.mean(axis=0)
                H = (obj_pts - co).T @ (cam_pts - cc)
                U,_, Vt = np.linalg.svd(H)
                R = Vt.T @ U.T
                if np.linalg.det(R) < 0:
                    Vt[2,:] *= -1
                    R = Vt.T @ U.T
                t = cc - R @ co

                # Quaternion from rotation matrix
                T4 = np.eye(4, dtype=np.float32)
                T4[:3,:3] = R
                q = quaternion_from_matrix(T4)
                rq = np.array([-q[2], q[1], -q[0], q[3]], dtype=np.float32)

                # Append to sliding window
                buf['positions'].append(t)
                buf['orientations'].append(rq)

                # If buffer full, average & broadcast immediately
                if len(buf['orientations']) == self.sample_window:
                    avg_t = np.mean(np.vstack(buf['positions']), axis=0)
                    avg_q = average_quaternions(np.vstack(buf['orientations']))
                    tfm = TransformStamped()
                    tfm.header.stamp    = self.get_clock().now().to_msg()
                    tfm.header.frame_id = "calibrated_camera_link"
                    tfm.child_frame_id  = self.marker_name_mapping.get(mid, f"ID_{mid}")
                    tfm.transform.translation.x = float(avg_t[0])
                    tfm.transform.translation.y = float(avg_t[1])
                    tfm.transform.translation.z = float(avg_t[2])
                    tfm.transform.rotation.x    = float(avg_q[0])
                    tfm.transform.rotation.y    = float(avg_q[1])
                    tfm.transform.rotation.z    = float(avg_q[2])
                    tfm.transform.rotation.w    = float(avg_q[3])
                    # Immediate publish
                    self.tf_broadcaster.sendTransform(tfm)
                    self.last_marker_transforms[mid] = tfm

                # Visualization drawing (same as original)
                pts2d = corners[i][0].astype(int)
                cv2.polylines(frame, [pts2d], True, (0,255,0), 2)
                cv2.polylines(depth_vis, [pts2d], True, (0,255,0), 2)
                rvec, _ = cv2.Rodrigues(R)
                tvec = t.reshape((3,1))
                axlen = s * 0.2
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, axlen)
                cv2.drawFrameAxes(depth_vis, self.camera_matrix, self.dist_coeffs, rvec, tvec, axlen)
                c = pts2d.mean(axis=0).astype(int)
                dv = self.latest_depth_image[c[1], c[0]]
                label = f"{self.marker_name_mapping.get(mid, mid)} {dv:.1f}mm"
                cv2.putText(frame, label, (c[0],c[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                cv2.putText(depth_vis, label, (c[0],c[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        # Show combined view
        if self.VISUALIZE:
            h, w = frame.shape[:2]
            if depth_vis.shape[1] != w:
                depth_vis = cv2.resize(depth_vis, (w, depth_vis.shape[0]))
            vis = np.vstack((frame, depth_vis))
            cv2.imshow("Aruco Detection - RGB (top) and Depth (bottom)", vis)
            cv2.waitKey(1)

    def crop_center(self, frame):
        if not getattr(self, 'image_width', None):
            return frame
        h, w = frame.shape[:2]
        cx, cy = (w - self.image_width)//2, (h - self.image_height)//2
        return frame[cy:cy+self.image_height, cx:cx+self.image_width]

    # Calibration TF at 100 Hz
    def publish_calibration_tf(self):
        if not self.calib_loaded:
            return
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = "Link6"
        t.child_frame_id  = "calibrated_camera_link"
        t.transform.translation.x = float(self.calib_trans[0])
        t.transform.translation.y = float(self.calib_trans[1])
        t.transform.translation.z = float(self.calib_trans[2])
        t.transform.rotation.x    = float(self.calib_quat[0])
        t.transform.rotation.y    = float(self.calib_quat[1])
        t.transform.rotation.z    = float(self.calib_quat[2])
        t.transform.rotation.w    = float(self.calib_quat[3])
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        if self.VISUALIZE:
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
        try:
            rclpy.shutdown()
        except Exception:
            pass
