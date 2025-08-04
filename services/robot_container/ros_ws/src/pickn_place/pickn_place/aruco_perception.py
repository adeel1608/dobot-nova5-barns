#!/usr/bin/env python3
"""
Aruco perception node using interior-plane RANSAC + ray-plane intersection.

Pipeline (per detection):
1) Detect marker quad in RGB.
2) Sample an m×n grid of interior pixels inside the quad (exclude a small margin).
3) Bilinear-sample aligned depth at those pixels; back-project to camera-frame 3D using *color* intrinsics.
4) RANSAC plane fit -> inliers -> least-squares plane refit.
5) Intersect each *corner* ray (from camera center) with that plane to obtain 4 clean 3D points.
6) Solve pose (Horn SVD). If plane fit fails, fall back to solvePnPRansac.
7) Sliding-window smoothing (translation mean + quaternion eigen-avg).
8) Publish TF and visualize.

Assumptions:
- The depth image is *already aligned* to the color frame (e.g., Orbbec Gemini 335 alignment enabled).
- Depth units are millimeters (divide by 1000 to get meters). Adjust DEPTH_SCALE if needed.
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
DEFAULT_EXTRINSICS_TOPIC    = '/camera/depth_to_color'  # optional (not required when aligned)

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
MARKER_SIZE_MM        = 100   # physical side length of the printed marker (mm)

# Plane-fit / sampling params
GRID_M, GRID_N        = 8, 8       # interior sampling grid
GRID_MARGIN_FRAC      = 0.10       # exclude this fraction from each quad edge
RANSAC_THRESH_M       = 0.010      # 1 cm inlier threshold (meters)
RANSAC_MAX_ITERS      = 120
MIN_PLANE_INLIERS     = 20         # minimum interior points to accept a plane
DEPTH_SCALE           = 1.0/1000.0 # mm -> m; set to 0.001 if needed

# ─── Math helpers ────────────────────────────────────────────────────────────
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
    """Eigen-avg quaternions; handle antipodal symmetry."""
    M = np.zeros((4,4), dtype=np.float64)
    for q in qs:
        if np.dot(qs[0], q) < 0:
            q = -q
        M += np.outer(q, q)
    M /= qs.shape[0]
    vals, vecs = np.linalg.eig(M)
    q_avg = vecs[:, np.argmax(vals)]
    return (q_avg / np.linalg.norm(q_avg)).astype(np.float32)

def bilinear_depth(depth_img, u, v):
    """Bilinear sample depth at subpixel (u,v). Returns meters (float) or 0 if invalid."""
    h, w = depth_img.shape[:2]
    if u < 0 or v < 0 or u >= w-1 or v >= h-1:
        return 0.0
    u0, v0 = int(np.floor(u)), int(np.floor(v))
    du, dv = u - u0, v - v0
    w00 = (1-du)*(1-dv); w10 = du*(1-dv); w01 = (1-du)*dv; w11 = du*dv
    d00 = float(depth_img[v0,   u0  ])
    d10 = float(depth_img[v0,   u0+1])
    d01 = float(depth_img[v0+1, u0  ])
    d11 = float(depth_img[v0+1, u0+1])
    d = w00*d00 + w10*d10 + w01*d01 + w11*d11
    if d <= 0.0 or np.isnan(d):
        return 0.0
    return d * DEPTH_SCALE

def make_perspective_warp(quad):
    """
    quad: (4,2) float32 in image pixels in order:
          [TopLeft, TopRight, BottomRight, BottomLeft] as returned by ArUco.
    Returns 3x3 H mapping unit square (0..1)×(0..1) -> quad.
    """
    unit = np.array([[0,0],[1,0],[1,1],[0,1]], dtype=np.float32)
    H = cv2.getPerspectiveTransform(unit, quad.astype(np.float32))
    return H

def sample_grid_inside_quad(quad, m=8, n=8, margin=0.10):
    """
    Returns (m*n, 2) float array of (u,v) points inside quad, excluding a 'margin' fraction.
    """
    H = make_perspective_warp(quad)
    us = np.linspace(margin, 1.0 - margin, n, dtype=np.float32)
    vs = np.linspace(margin, 1.0 - margin, m, dtype=np.float32)
    grid = np.stack(np.meshgrid(us, vs), axis=-1).reshape(-1,2)  # (m*n,2) in unit square
    homog = np.concatenate([grid, np.ones((grid.shape[0],1), dtype=np.float32)], axis=1)  # (N,3)
    mapped = (H @ homog.T).T
    mapped = mapped[:, :2] / mapped[:, 2:3]
    return mapped  # (N,2)

def fit_plane_svd(points_m):
    """
    Least-squares plane fit via SVD on demeaned points.
    Returns (n, d), with ||n||=1, plane eq: n^T x + d = 0
    """
    P = np.asarray(points_m, dtype=np.float64)
    c = P.mean(axis=0)
    U, S, Vt = np.linalg.svd(P - c, full_matrices=False)
    n = Vt[-1, :]
    n = n / np.linalg.norm(n)
    d = -np.dot(n, c)
    return n.astype(np.float32), float(d)

def ransac_plane(points_m, thresh=0.01, max_iters=120, min_inliers=20):
    """
    RANSAC plane: random 3-point samples; returns (n, d, inlier_mask) or (None,...)
    """
    P = np.asarray(points_m, dtype=np.float32)
    N = P.shape[0]
    if N < 3:
        return None, None, None
    best_inliers = None
    best_count = -1

    rng = np.random.default_rng()
    for _ in range(max_iters):
        ids = rng.choice(N, size=3, replace=False)
        p1, p2, p3 = P[ids]
        # Compute normal
        v1 = p2 - p1
        v2 = p3 - p1
        n = np.cross(v1, v2)
        norm = np.linalg.norm(n)
        if norm < 1e-8:
            continue
        n = n / norm
        d = -np.dot(n, p1)
        # Distances
        dist = np.abs(P @ n + d)
        inliers = dist < thresh
        count = int(np.count_nonzero(inliers))
        if count > best_count:
            best_count, best_inliers = count, inliers
            if best_count >= max(min_inliers, int(0.8*N)):  # early exit
                break

    if best_inliers is None or best_count < min_inliers:
        return None, None, None

    # Refit plane with all inliers for accuracy
    n_refit, d_refit = fit_plane_svd(P[best_inliers])
    return n_refit, d_refit, best_inliers

def intersect_ray_plane(u, v, K, n, d):
    """
    Ray from camera center through pixel (u,v):
      vdir = inv(K) @ [u, v, 1]^T
      t = -d / (n^T vdir)
      P = t * vdir
    Returns 3D point in camera frame (float32) or None if invalid (parallel or t<=0).
    """
    vdir = np.linalg.inv(K) @ np.array([u, v, 1.0], dtype=np.float64)
    denom = float(np.dot(n, vdir))
    if abs(denom) < 1e-8:
        return None
    t = -float(d) / denom
    if t <= 0:
        return None
    P = (t * vdir).astype(np.float32)
    return P

# ─── Node ─────────────────────────────────────────────────────────────────
class ArucoPerceptionPlaneNode(Node):
    def __init__(self):
        super().__init__('aruco_perception_plane_node')

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

        # Latest frames
        self.latest_depth_image = None

        # TF broadcaster & buffers
        self.tf_broadcaster         = tf2_ros.TransformBroadcaster(self)
        self.last_marker_transforms = {}  # id → TransformStamped
        self.marker_samples         = {}  # id → {'positions': deque, 'orientations': deque}

        # Timers
        self.create_timer(CALIBRATION_TF_INT, self.publish_calibration_tf)
        self.create_timer(DEFAULT_LOG_INTERVAL, self.check_input_topics)

        # Load static calibration TF (robot_link -> calibrated_camera_link)
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
        self.create_subscription(CameraInfo, DEFAULT_DEPTH_INFO_TOPIC, self.depth_info_callback, 10)
        self.create_subscription(Image,      DEFAULT_DEPTH_IMAGE_TOPIC,self.depth_callback,      10)
        # Optional extrinsics (not used for aligned depth sampling)
        self.create_subscription(TransformStamped, DEFAULT_EXTRINSICS_TOPIC, self.extrinsics_callback, 10)

        # Visualization
        if self.VISUALIZE:
            cv2.namedWindow("Aruco PlaneFit - RGB (top) and Depth (bottom)", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Aruco PlaneFit - RGB (top) and Depth (bottom)", 900, 1000)

    # ─── Logging helper ─────────────────────────────────────────────────────
    def throttled_log(self, msg, level="info"):
        if time() - self.last_log_time >= DEFAULT_LOG_INTERVAL:
            getattr(self.get_logger(), level)(msg)
            self.last_log_time = time()

    # ─── Callbacks ─────────────────────────────────────────────────────────
    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3,3).astype(np.float64)
            self.dist_coeffs   = np.array(msg.d, dtype=np.float64)
            self.image_width, self.image_height = msg.width, msg.height
            self.camera_info_received = True
            self.throttled_log("Color camera intrinsics received.")

    def depth_info_callback(self, msg: CameraInfo):
        if not self.depth_info_received:
            # We don't need depth intrinsics for aligned sampling, but keep for debug.
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
        # Not required for aligned sampling; kept for visibility.
        t = msg.transform.translation
        self.depth_to_color_t = np.array([[t.x],[t.y],[t.z]], dtype=np.float32)
        self.get_logger().info("Depth→Color extrinsics updated (unused for aligned sampling).")

    def check_input_topics(self):
        if not self.camera_info_received:
            self.throttled_log("No color camera info received.", "warn")
        if not getattr(self, 'image_received', False):
            self.throttled_log("No color image received.", "warn")
        if not self.depth_info_received:
            self.throttled_log("No depth camera info received.", "warn")
        if not getattr(self, 'depth_image_received', False):
            self.throttled_log("No depth image received.", "warn")

    # ─── Main image callback ─────────────────────────────────────────────────
    def image_callback(self, msg: Image):
        self.image_received = True

        # Guards
        if not self.camera_info_received:
            self.throttled_log("Waiting for color camera intrinsics...", "warn"); return
        if self.latest_depth_image is None:
            self.throttled_log("Waiting for first depth image...", "warn"); return

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

                # Marker object points (meters)
                s = MARKER_SIZE_MM / 1000.0
                obj_pts = np.array([
                    [-s/2, s/2, 0], [ s/2, s/2, 0],
                    [ s/2,-s/2, 0], [-s/2,-s/2, 0]
                ], dtype=np.float32)

                # --- Plane fit from interior depth samples ---
                pts2d = corners[i][0].astype(np.float32)  # (4,2)
                # Subpixel refine in RGB for better 2D corner accuracy (optional but helpful)
                # Note: cv2.cornerSubPix expects grayscale + initial pts
                try:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
                    corners_refined = pts2d.copy().reshape(-1,1,2)
                    cv2.cornerSubPix(gray, corners_refined, (5,5), (-1,-1), criteria)
                    pts2d = corners_refined.reshape(-1,2)
                except Exception:
                    pass  # if subpix fails, keep original

                # Interior sampling (aligned depth; use color intrinsics)
                grid_uv = sample_grid_inside_quad(pts2d, m=GRID_M, n=GRID_N, margin=GRID_MARGIN_FRAC)

                # Back-project valid samples to 3D
                K = self.camera_matrix
                fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
                pts3d = []
                for (u, v) in grid_uv:
                    d = bilinear_depth(self.latest_depth_image, float(u), float(v))
                    if d <= 0.0:
                        continue
                    X = (u - cx) * d / fx
                    Y = (v - cy) * d / fy
                    pts3d.append([X, Y, d])

                used_plane = False
                cam_pts = None

                if len(pts3d) >= MIN_PLANE_INLIERS:
                    n, d, inliers = ransac_plane(pts3d, thresh=RANSAC_THRESH_M,
                                                 max_iters=RANSAC_MAX_ITERS,
                                                 min_inliers=MIN_PLANE_INLIERS)
                    if n is not None:
                        used_plane = True
                        cam_pts = []
                        for (u, v) in pts2d:
                            P = intersect_ray_plane(float(u), float(v), K, n, d)
                            if P is None:
                                used_plane = False
                                break
                            cam_pts.append(P)
                        if used_plane:
                            cam_pts = np.array(cam_pts, dtype=np.float32)

                # --- Fallback: RGB-only solvePnPRansac (depth-free) ---
                if cam_pts is None or not used_plane:
                    # Use PnP with 2D corners
                    retval, rvec, tvec, inliers_pnp = cv2.solvePnPRansac(
                        obj_pts, pts2d, K, self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE,  # robust for square planar targets
                        iterationsCount=100, reprojectionError=3.0, confidence=0.99
                    )
                    if not retval:
                        continue
                    R, _ = cv2.Rodrigues(rvec)
                    t = tvec.reshape(3)
                else:
                    # Pose from cam_pts (Horn SVD)
                    obj_mean = obj_pts.mean(axis=0)
                    cam_mean = cam_pts.mean(axis=0)
                    H = (obj_pts - obj_mean).T @ (cam_pts - cam_mean)
                    U, _, Vt = np.linalg.svd(H)
                    R = Vt.T @ U.T
                    if np.linalg.det(R) < 0:
                        Vt[2,:] *= -1
                        R = Vt.T @ U.T
                    t = cam_mean - R @ obj_mean

                # Quaternion from rotation matrix
                T4 = np.eye(4, dtype=np.float32); T4[:3,:3] = R.astype(np.float32)
                q = quaternion_from_matrix(T4)
                # Axis re-map to match your original ROS convention
                rq = np.array([-q[2], q[1], -q[0], q[3]], dtype=np.float32)

                # Append to sliding window
                buf['positions'].append(t.astype(np.float32))
                buf['orientations'].append(rq)

                # If buffer is full, average & broadcast immediately
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
                    self.tf_broadcaster.sendTransform(tfm)
                    self.last_marker_transforms[mid] = tfm

                # Visualization
                pts2d_int = pts2d.astype(int)
                cv2.polylines(frame, [pts2d_int], True, (0,255,0), 2)
                cv2.polylines(depth_vis, [pts2d_int], True, (0,255,0), 2)
                rvec_vis, _ = cv2.Rodrigues(R.astype(np.float64))
                tvec_vis = t.reshape((3,1)).astype(np.float64)
                axlen = s * 0.2
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec_vis, tvec_vis, axlen)
                cv2.drawFrameAxes(depth_vis, self.camera_matrix, self.dist_coeffs, rvec_vis, tvec_vis, axlen)
                c = pts2d.mean(axis=0).astype(int)
                dv = bilinear_depth(self.latest_depth_image, float(c[0]), float(c[1])) / (DEPTH_SCALE if DEPTH_SCALE!=0 else 1.0)
                label = f"{self.marker_name_mapping.get(mid, mid)}"
                cv2.putText(frame, label, (c[0], c[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                cv2.putText(depth_vis, label, (c[0], c[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        # Show combined view
        if self.VISUALIZE:
            h, w = frame.shape[:2]
            if depth_vis.shape[1] != w:
                depth_vis = cv2.resize(depth_vis, (w, depth_vis.shape[0]))
            vis = np.vstack((frame, depth_vis))
            cv2.imshow("Aruco PlaneFit - RGB (top) and Depth (bottom)", vis)
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
    node = ArucoPerceptionPlaneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
