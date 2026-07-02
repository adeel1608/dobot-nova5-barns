#!/usr/bin/env python3
"""
ArUco perception node with sliding-window averaging and immediate TF publish.

Detection / TF behaviour
------------------------
- Configurable sample window size (default 6).
- Use deque(maxlen=N) per marker for sliding-buffer.
- On each new detection, append and broadcast the running average immediately.
- Calibration TF still broadcast at 100 Hz.

Visualization (control-plane / data-plane split)
------------------------------------------------
The previous version popped up a local OpenCV window (cv2.imshow) which was
useless headless and ran 24/7 even when nobody was watching. The headless,
on-demand replacement is:

  Control plane:
    - std_srvs/srv/SetBool service at  ~/set_visualization
    - std_msgs/Bool status topic at    ~/visualization/active   (TRANSIENT_LOCAL)

  Data plane:
    - sensor_msgs/CompressedImage at   ~/visualization/compressed (BEST_EFFORT)

While the visualization is disabled (the default), the node skips every
render-only operation: depth normalization / colormap, polylines, axis draw,
label text, vstack and JPEG encoding. It keeps detecting markers and
broadcasting TFs as before. When SetBool(true) is received, the node renders
each subsequent frame, JPEG-encodes it, and publishes a CompressedImage.

A companion perception_streamer node subscribes to the CompressedImage topic
and serves it as MJPEG to the video-stream backend / dashboard. That keeps the
heavy data path entirely inside the robot pod (intra-host shared-memory DDS
transport) and avoids piping raw frames through any non-ROS bridges.
"""
import os
from collections import deque
from time import time

import cv2
import numpy as np
import rclpy
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from transformations import quaternion_from_matrix

# ---- Constants ---------------------------------------------------------------
DEFAULT_VISUALIZE       = False
DEFAULT_SAMPLE_WINDOW   = 6
DEFAULT_LOG_INTERVAL    = 5.0
DEPTH_MEDIAN_HALF       = 1      # 3x3 median patch for depth sampling
DEFAULT_VIZ_JPEG_QUALITY = 70
DEFAULT_VIZ_FRAME_SKIP   = 1     # publish every N+1-th frame (1 -> ~half rate)

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
ARUCO_PARAMS.cornerRefinementMinAccuracy = 0.05

CALIBRATION_TF_INT    = 0.01  # seconds -> 100 Hz
PACKAGE_NAME          = 'pickn_place'
CALIB_FILE            = 'axab_calibration.yaml'
ID_NAME_CONFIG_FILE   = 'arucoID_name_config.yaml'
MARKER_SIZE_MM        = 100   # fixed model size for compatibility

_s = MARKER_SIZE_MM / 1000.0
OBJ_PTS = np.array([
    [-_s/2, _s/2, 0], [ _s/2, _s/2, 0],
    [ _s/2,-_s/2, 0], [-_s/2,-_s/2, 0]
], dtype=np.float32)

# ---- Helpers -----------------------------------------------------------------
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

# ---- Node -------------------------------------------------------------------
class ArucoPerceptionNode(Node):
    def __init__(self):
        super().__init__('aruco_perception_node')
        self.get_logger().info("Initializing ArucoPerceptionNode...")

        # --- Parameters ------------------------------------------------------
        # Topic names are exposed so robot1/robot2 (or tests) can rewire them
        # without touching code. Defaults preserve the historical behaviour.
        self.declare_parameter('visualize',          DEFAULT_VISUALIZE)
        self.declare_parameter('sample_window_size', DEFAULT_SAMPLE_WINDOW)
        self.declare_parameter('image_topic',         DEFAULT_IMAGE_TOPIC)
        self.declare_parameter('camera_info_topic',   DEFAULT_CAMERA_INFO_TOPIC)
        self.declare_parameter('depth_image_topic',   DEFAULT_DEPTH_IMAGE_TOPIC)
        self.declare_parameter('depth_info_topic',    DEFAULT_DEPTH_INFO_TOPIC)
        self.declare_parameter('extrinsics_topic',    DEFAULT_EXTRINSICS_TOPIC)
        self.declare_parameter('viz_jpeg_quality',    DEFAULT_VIZ_JPEG_QUALITY)
        self.declare_parameter('viz_frame_skip',      DEFAULT_VIZ_FRAME_SKIP)

        self._viz_enabled = bool(self.get_parameter('visualize').value)
        self.sample_window = int(self.get_parameter('sample_window_size').value)
        self._viz_quality = int(self.get_parameter('viz_jpeg_quality').value)
        self._viz_skip_n = max(0, int(self.get_parameter('viz_frame_skip').value))
        self._viz_skip_counter = 0

        image_topic       = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        depth_image_topic = self.get_parameter('depth_image_topic').value
        depth_info_topic  = self.get_parameter('depth_info_topic').value
        extrinsics_topic  = self.get_parameter('extrinsics_topic').value

        self.get_logger().info(
            f"Visualization initial state: {'Enabled' if self._viz_enabled else 'Disabled'} "
            f"(toggle with /aruco_perception_node/set_visualization)"
        )
        self.get_logger().info(f"Sample window size: {self.sample_window}")
        self.get_logger().info(
            f"Topics: color={image_topic}, color_info={camera_info_topic}, "
            f"depth={depth_image_topic}, depth_info={depth_info_topic}, "
            f"extrinsics={extrinsics_topic}"
        )

        # Logging throttle
        self.last_log_time = 0.0

        # Bridge
        self.bridge = CvBridge()
        self.get_logger().info("CvBridge initialized.")

        # Intrinsics flags
        self.camera_info_received = False
        self.depth_info_received  = False
        self.image_received       = False
        self.depth_image_received = False

        # Latest frames
        self.latest_depth_image = None

        # TF broadcaster & buffers
        self.tf_broadcaster        = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info("TF broadcaster initialized.")
        self.last_marker_transforms = {}  # id -> TransformStamped
        self.marker_samples         = {}  # id -> {'positions': deque, 'orientations': deque}

        # --- Visualization control plane / data plane ------------------------
        # Created once; cheap to keep around even when disabled. Subscribers
        # only start receiving frames once SetBool(true) has been received.
        viz_pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        viz_status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._viz_pub = self.create_publisher(
            CompressedImage, '~/visualization/compressed', viz_pub_qos
        )
        self._viz_status_pub = self.create_publisher(
            Bool, '~/visualization/active', viz_status_qos
        )
        self._publish_viz_status()

        self._set_viz_srv = self.create_service(
            SetBool, '~/set_visualization', self._on_set_visualization
        )
        self.get_logger().info(
            "Visualization control plane ready: service "
            "~/set_visualization (std_srvs/SetBool), publisher "
            "~/visualization/compressed (sensor_msgs/CompressedImage)."
        )

        # Timers
        self.create_timer(CALIBRATION_TF_INT, self.publish_calibration_tf)
        self.create_timer(DEFAULT_LOG_INTERVAL, self.check_input_topics)
        self.get_logger().info("Timers created for calibration TF publishing and topic checks.")

        # Load static calibration
        pkg_dir = get_package_share_directory(PACKAGE_NAME)
        self.get_logger().info(f"Loading calibration from '{CALIB_FILE}'...")
        try:
            with open(os.path.join(pkg_dir, CALIB_FILE), 'r') as f:
                ct = yaml.safe_load(f)['calibration_transform']
            self.calib_quat  = np.array([ct['rotation'][k] for k in ('x','y','z','w')], dtype=np.float32)
            self.calib_trans = np.array([ct['translation'][k] for k in ('x','y','z')], dtype=np.float32)
            self.calib_loaded = True
            self.get_logger().info("Successfully loaded static calibration transform.")
        except Exception as e:
            self.get_logger().error(f"Calibration load error: {e}")
            self.get_logger().warn("Using default identity calibration.")
            self.calib_quat  = np.array([0,0,0,1], dtype=np.float32)
            self.calib_trans = np.zeros(3, dtype=np.float32)
            self.calib_loaded = False

        # Load ID->name mapping
        self.get_logger().info(f"Loading ArUco ID-to-name mapping from '{ID_NAME_CONFIG_FILE}'...")
        try:
            with open(os.path.join(pkg_dir, ID_NAME_CONFIG_FILE), 'r') as f:
                items = yaml.safe_load(f).get('aruco_id', [])
            self.marker_name_mapping = {it['id']: it['name'] for it in items}
            self.get_logger().info(f"Loaded {len(self.marker_name_mapping)} ArUco ID mappings.")
        except Exception as e:
            self.get_logger().error(f"ID->name load error: {e}")
            items = []
            self.marker_name_mapping = {}

        # Subscriptions
        self.get_logger().info("Creating subscriptions...")
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image,      image_topic,       self.image_callback,       10)
        self.create_subscription(CameraInfo, depth_info_topic,  self.depth_info_callback,  10)
        self.create_subscription(Image,      depth_image_topic, self.depth_callback,       10)
        self.create_subscription(TransformStamped, extrinsics_topic, self.extrinsics_callback, 10)
        self.get_logger().info("Subscriptions created.")

        self.get_logger().info("ArucoPerceptionNode initialized successfully.")

    # ---- Visualization service -----------------------------------------------
    def _on_set_visualization(self, request: SetBool.Request, response: SetBool.Response):
        new_state = bool(request.data)
        if new_state == self._viz_enabled:
            response.success = True
            response.message = f"Visualization already {'enabled' if new_state else 'disabled'}"
            return response

        self._viz_enabled = new_state
        self._viz_skip_counter = 0
        self._publish_viz_status()
        self.get_logger().info(
            f"Visualization {'enabled' if new_state else 'disabled'} via SetBool"
        )
        response.success = True
        response.message = f"Visualization {'enabled' if new_state else 'disabled'}"
        return response

    def _publish_viz_status(self):
        msg = Bool()
        msg.data = bool(self._viz_enabled)
        self._viz_status_pub.publish(msg)

    # ---- Logging helper ------------------------------------------------------
    def throttled_log(self, msg, level="info"):
        if time() - self.last_log_time >= DEFAULT_LOG_INTERVAL:
            if level == "debug":
                self.get_logger().debug(msg)
            elif level == "info":
                self.get_logger().info(msg)
            elif level == "warn" or level == "warning":
                self.get_logger().warn(msg)
            elif level == "error":
                self.get_logger().error(msg)
            elif level == "fatal":
                self.get_logger().fatal(msg)
            else:
                self.get_logger().info(msg)
            self.last_log_time = time()

    # ---- Callbacks -----------------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3,3)
            self.dist_coeffs   = np.array(msg.d)
            self.image_width, self.image_height = msg.width, msg.height
            self.camera_info_received = True
            self.get_logger().info("Color camera intrinsics received and processed.")

    def depth_info_callback(self, msg: CameraInfo):
        if not self.depth_info_received:
            self.depth_camera_matrix = np.array(msg.k).reshape(3,3)
            self.depth_dist_coeffs   = np.array(msg.d)
            self.depth_width, self.depth_height = msg.width, msg.height
            self.depth_info_received = True
            self.get_logger().info("Depth camera intrinsics received and processed.")

    def depth_callback(self, msg: Image):
        if not self.depth_image_received:
            self.throttled_log("First depth image received.")
        self.depth_image_received = True
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except cv2.error as e:
            self.get_logger().error(f"Depth callback cv2 error: {e}")

    def extrinsics_callback(self, msg: TransformStamped):
        t = msg.transform.translation
        self.depth_to_color_t = np.array([[t.x],[t.y],[t.z]], dtype=np.float32)
        self.get_logger().info(f"Depth->Color extrinsics updated: T=({t.x:.4f}, {t.y:.4f}, {t.z:.4f})")

    def check_input_topics(self):
        if not self.camera_info_received:
            self.throttled_log("No color camera info received.", "warn")
        if not self.image_received:
            self.throttled_log("No color image received.", "warn")
        if not self.depth_info_received:
            self.throttled_log("No depth camera info received.", "warn")
        if not self.depth_image_received:
            self.throttled_log("No depth image received.", "warn")

    # ---- Main image callback -------------------------------------------------
    def image_callback(self, msg: Image):
        if not self.image_received:
            self.get_logger().info("First color image received.")
        self.image_received = True

        if not self.camera_info_received:
            self.throttled_log("Waiting for color camera intrinsics...", "warn")
            return
        if not self.depth_info_received:
            self.throttled_log("Waiting for depth camera intrinsics...", "warn")
            return
        if self.latest_depth_image is None:
            self.throttled_log("Waiting for first depth image...", "warn")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        frame = self.crop_center(frame)

        # Visualization-only render buffers; never produced when disabled so
        # that idle CPU stays at "detection-only" levels.
        viz_active = self._viz_enabled
        depth_vis = None
        if viz_active:
            depth_vis = cv2.normalize(self.latest_depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_vis = cv2.applyColorMap(depth_vis.astype(np.uint8), cv2.COLORMAP_JET)
            depth_vis = cv2.resize(depth_vis, (frame.shape[1], frame.shape[0]))

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(frame, ARUCO_DICT, parameters=ARUCO_PARAMS)
        if ids is None:
            self.throttled_log("No ArUco markers detected in the current frame.", "info")

        if ids is not None:
            self.throttled_log(f"Detected {len(ids)} marker(s): {ids.flatten().tolist()}", "info")

            scale_u = self.depth_width / float(self.image_width) if self.image_width else 1.0
            scale_v = self.depth_height / float(self.image_height) if self.image_height else 1.0
            fx, fy = self.depth_camera_matrix[0,0], self.depth_camera_matrix[1,1]
            cx, cy = self.depth_camera_matrix[0,2], self.depth_camera_matrix[1,2]
            dh_img, dw_img = self.depth_height, self.depth_width
            k = DEPTH_MEDIAN_HALF

            for i, mid in enumerate(ids.flatten()):
                mid = int(mid)

                buf = self.marker_samples.setdefault(
                    mid,
                    {
                        'positions': deque(maxlen=self.sample_window),
                        'orientations': deque(maxlen=self.sample_window)
                    }
                )

                cam_pts = []
                valid = True
                for (uf, vf) in corners[i][0]:
                    u_d = int(round(uf * scale_u))
                    v_d = int(round(vf * scale_v))
                    if not (k <= u_d < dw_img - k and k <= v_d < dh_img - k):
                        valid = False; break
                    patch = self.latest_depth_image[v_d-k:v_d+k+1, u_d-k:u_d+k+1]
                    d = float(np.median(patch)) / 1000.0
                    if d <= 0:
                        valid = False; break
                    X = (u_d - cx) * d / fx
                    Y = (v_d - cy) * d / fy
                    cam_pts.append([X, Y, d])
                if not valid or len(cam_pts) < 4:
                    continue
                cam_pts = np.array(cam_pts, dtype=np.float32)

                co, cc = OBJ_PTS.mean(axis=0), cam_pts.mean(axis=0)
                H_mat = (OBJ_PTS - co).T @ (cam_pts - cc)
                U, _, Vt = np.linalg.svd(H_mat)
                R = Vt.T @ U.T
                if np.linalg.det(R) < 0:
                    Vt[2,:] *= -1
                    R = Vt.T @ U.T
                t = cc - R @ co

                T4 = np.eye(4, dtype=np.float32)
                T4[:3,:3] = R
                q = quaternion_from_matrix(T4)
                rq = np.array([-q[2], q[1], -q[0], q[3]], dtype=np.float32)

                buf['positions'].append(t)
                buf['orientations'].append(rq)

                if len(buf['orientations']) == self.sample_window:
                    avg_t = np.mean(np.vstack(buf['positions']), axis=0)
                    avg_q = average_quaternions(np.vstack(buf['orientations']))
                    marker_name = self.marker_name_mapping.get(mid, f"ID_{mid}")
                    tfm = TransformStamped()
                    tfm.header.stamp    = self.get_clock().now().to_msg()
                    tfm.header.frame_id = "calibrated_camera_link"
                    tfm.child_frame_id  = marker_name
                    tfm.transform.translation.x = float(avg_t[0])
                    tfm.transform.translation.y = float(avg_t[1])
                    tfm.transform.translation.z = float(avg_t[2])
                    tfm.transform.rotation.x    = float(avg_q[0])
                    tfm.transform.rotation.y    = float(avg_q[1])
                    tfm.transform.rotation.z    = float(avg_q[2])
                    tfm.transform.rotation.w    = float(avg_q[3])
                    self.tf_broadcaster.sendTransform(tfm)
                    self.last_marker_transforms[mid] = tfm

                if viz_active:
                    pts2d = corners[i][0].astype(int)
                    cv2.polylines(frame, [pts2d], True, (0,255,0), 2)
                    cv2.polylines(depth_vis, [pts2d], True, (0,255,0), 2)
                    rvec, _ = cv2.Rodrigues(R)
                    tvec = t.reshape((3,1))
                    axlen = _s * 0.2
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, axlen)
                    cv2.drawFrameAxes(depth_vis, self.camera_matrix, self.dist_coeffs, rvec, tvec, axlen)
                    c = pts2d.mean(axis=0).astype(int)
                    dv = self.latest_depth_image[c[1], c[0]]
                    label = f"{self.marker_name_mapping.get(mid, mid)} {dv:.1f}mm"
                    cv2.putText(frame, label, (c[0],c[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                    cv2.putText(depth_vis, label, (c[0],c[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        # Compose + publish the visualization frame only if (a) viz is enabled
        # and (b) we are on a frame the skip-counter wants to publish. This is
        # the headless replacement for the legacy cv2.imshow window.
        if viz_active and depth_vis is not None:
            if self._viz_skip_n > 0 and self._viz_skip_counter < self._viz_skip_n:
                self._viz_skip_counter += 1
            else:
                self._viz_skip_counter = 0
                h, w = frame.shape[:2]
                if depth_vis.shape[1] != w:
                    depth_vis = cv2.resize(depth_vis, (w, depth_vis.shape[0]))
                vis = np.vstack((frame, depth_vis))
                ok, jpg = cv2.imencode(
                    '.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, self._viz_quality]
                )
                if ok:
                    out = CompressedImage()
                    out.header.stamp = self.get_clock().now().to_msg()
                    out.header.frame_id = 'aruco_visualization'
                    out.format = 'jpeg'
                    out.data = jpg.tobytes()
                    self._viz_pub.publish(out)

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
        self.get_logger().info("Shutting down ArucoPerceptionNode...")
        # No GUI windows are owned by this process anymore; nothing extra to
        # release. The publishers/services are cleaned up by Node.destroy_node.
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
