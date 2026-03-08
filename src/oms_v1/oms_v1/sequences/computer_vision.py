# ******************************************************************************
#   Copyright (c) 2024 Orbbec 3D Technology, Inc
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http:# www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
# ******************************************************************************
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

PRINT_INTERVAL = 0.2  # seconds
MIN_DEPTH = 0       # 20mm (actual physical min depth)
MAX_DEPTH = 500   # 10000mm (Orbbec cameras typically range up to 10m)

# IMPORTANT: Define the invalid depth threshold (in millimeters).
# Pixels below this value (after scaling) will be considered 'invalid' or 'no depth'.
# 0 mm is truly no depth, but sometimes very close objects might register a small value.
# Adjust this based on your camera's noise and how transparent objects appear.
INVALID_DEPTH_THRESHOLD_MM = 180 # Pixels with depth <= 180mm will be considered invalid for detection

# --- Cup Detection Parameters ---
# These values will likely need to be adjusted based on your gripper, camera, and cup size.
# They define a rectangular Region of Interest (ROI) in the depth image where you expect
# the transparent cup's "hole" to appear when held by the gripper.

# ROI relative to image width/height (normalized 0-1)
# Example: 0.4 means 40% from the left/top edge.
CUP_ROI_X_NORM_MIN = 0.45
CUP_ROI_X_NORM_MAX = 0.65
CUP_ROI_Y_NORM_MIN = 0.50
CUP_ROI_Y_NORM_MAX = 0.80

# Threshold for detecting a cup:
# If the percentage of invalid depth pixels in the ROI exceeds this, a cup is detected.
# This value needs careful calibration with an empty gripper vs. a gripper holding a cup.
CUP_DETECTION_INVALID_PERCENTAGE_THRESHOLD = 40 # Example: 70% of ROI pixels are invalid

# --- End Cup Detection Parameters ---

class TemporalFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.previous_frame = None

    def process(self, frame):
        if self.previous_frame is None:
            result = frame
        else:
            # Use np.array for operations to ensure type consistency
            result = cv2.addWeighted(frame, self.alpha, self.previous_frame, 1 - self.alpha, 0)
        self.previous_frame = result
        return result


class DepthSubscriberNode(Node):
    """ROS2 node to subscribe to depth image topics."""
    
    def __init__(self):
        super().__init__('cup_detection_node')
        self.bridge = CvBridge()
        self.latest_depth_image = None
        self.depth_scale = 0.001  # Default: 1mm per unit (typical for 16-bit depth)
        self.depth_info_received = False
        
        # Subscribers
        self.depth_image_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_image_callback,
            10
        )
        
        self.depth_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.depth_info_callback,
            10
        )
        
        self.get_logger().info("Subscribed to /camera/depth/image_raw and /camera/depth/camera_info")
    
    def depth_image_callback(self, msg):
        try:
            # Convert ROS2 Image message to OpenCV format (16-bit depth)
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")
    
    def depth_info_callback(self, msg):
        # Extract depth scale from camera info if available
        # For Orbbec cameras, depth is typically in mm, so scale is 0.001 (1mm = 0.001m)
        # But we work in mm, so we use 1.0 as scale (raw values are already in mm)
        # Actually, 16-bit depth images from Orbbec are typically in mm directly
        self.depth_scale = 1.0  # Assume raw values are in mm
        self.depth_info_received = True


def detect_cup_gripper(**params):
    """
    Detect if a transparent cup is in the gripper using depth camera via ROS2 topics.
    Runs headless (no GUI) and returns True if cup detected, False otherwise.
    
    Args:
        **params: Optional parameters (for compatibility with sequence interface)
    
    Returns:
        bool: True if cup detected, False otherwise
    """
    # Initialize ROS2 if not already initialized
    if not rclpy.ok():
        rclpy.init()
    
    # Create node and subscribe to topics
    node = DepthSubscriberNode()
    temporal_filter = TemporalFilter(alpha=0.5)
    
    last_print_time = time.time()
    cup_detected = False
    frames_processed = 0
    max_frames = 5
    timeout_seconds = 10.0
    start_time = time.time()
    
    try:
        # Wait for first depth image and camera info
        print("[CV] Waiting for depth camera topics...")
        while (node.latest_depth_image is None or not node.depth_info_received) and \
              (time.time() - start_time) < timeout_seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
        
        if node.latest_depth_image is None:
            print("[CV] ERROR: No depth image received within timeout")
            node.destroy_node()
            return False
        
        if not node.depth_info_received:
            print("[CV] WARNING: No camera info received, using default scale")
        
        print(f"[CV] Depth image received: {node.latest_depth_image.shape}")
        
        # Capture and process frames
        while frames_processed < max_frames:
            # Spin to get latest frame
            rclpy.spin_once(node, timeout_sec=0.1)
            
            if node.latest_depth_image is None:
                time.sleep(0.1)
                continue
            
            depth_image = node.latest_depth_image.copy()
            
            # Get image dimensions
            height, width = depth_image.shape[:2]
            
            # Depth image from ROS2 is typically uint16 in millimeters
            # Convert to numpy array if not already
            if not isinstance(depth_image, np.ndarray):
                depth_data_raw = np.array(depth_image, dtype=np.uint16)
            else:
                depth_data_raw = depth_image.astype(np.uint16)
            
            # Ensure 2D array
            if len(depth_data_raw.shape) > 2:
                depth_data_raw = depth_data_raw[:, :, 0]
            
            # Apply temporal filtering
            filtered_depth_data_raw = temporal_filter.process(depth_data_raw)
            
            # Convert to float32 and apply depth scale to get depth in millimeters
            # For Orbbec cameras via ROS2, values are typically already in mm
            depth_data_mm = filtered_depth_data_raw.astype(np.float32) * node.depth_scale
            
            # Apply depth range filter (MIN_DEPTH to MAX_DEPTH)
            # Pixels outside this range are set to 3000 mm (background)
            depth_data_filtered = np.where((depth_data_mm >= MIN_DEPTH) & (depth_data_mm <= MAX_DEPTH),
                                            depth_data_mm, 3000)
            
            # --- Cup Detection Logic ---
            # Calculate ROI pixel coordinates based on the current frame's resolution
            roi_x_min = int(width * CUP_ROI_X_NORM_MIN)
            roi_x_max = int(width * CUP_ROI_X_NORM_MAX)
            roi_y_min = int(height * CUP_ROI_Y_NORM_MIN)
            roi_y_max = int(height * CUP_ROI_Y_NORM_MAX)
            
            # Extract the ROI from the depth data
            roi_depth_data = depth_data_filtered[roi_y_min:roi_y_max, roi_x_min:roi_x_max]
            
            # Create a binary mask where True indicates an "invalid" depth pixel
            # An "invalid" pixel is one that is very close to 0 mm, indicating light passed through.
            invalid_roi_mask = (roi_depth_data <= INVALID_DEPTH_THRESHOLD_MM)
            
            # Count the number of invalid pixels within the ROI
            num_invalid_pixels = np.sum(invalid_roi_mask)
            total_roi_pixels = roi_depth_data.size
            
            if total_roi_pixels > 0:  # Avoid division by zero if ROI is empty
                percentage_invalid = (num_invalid_pixels / total_roi_pixels) * 100
                
                if percentage_invalid >= CUP_DETECTION_INVALID_PERCENTAGE_THRESHOLD:
                    cup_detected = True
                    detection_status_text = f"CUP DETECTED! Invalid: {percentage_invalid:.1f}%"
                else:
                    detection_status_text = f"No cup. Invalid: {percentage_invalid:.1f}%"
            else:
                detection_status_text = "ROI is empty, cannot detect cup."
            
            # --- End Cup Detection Logic ---
            
            # Optional: Print center distance for general depth checking
            center_y = int(height / 2)
            center_x = int(width / 2)
            if depth_data_filtered[center_y, center_x] > INVALID_DEPTH_THRESHOLD_MM:
                center_distance = depth_data_filtered[center_y, center_x]
            else:
                center_distance = 0
            
            current_time = time.time()
            if current_time - last_print_time >= PRINT_INTERVAL:
                print(f"[CV] Center distance: {center_distance:.1f} mm, {detection_status_text}")
                last_print_time = current_time
            
            frames_processed += 1
            time.sleep(0.2)  # Small delay between frames
        
        print(f"[CV] Cup detection result: {cup_detected}")
        node.destroy_node()
        return cup_detected
        
    except KeyboardInterrupt:
        print("[CV] Exiting by KeyboardInterrupt.")
        node.destroy_node()
        return False
    except Exception as e:
        print(f"[CV] An error occurred during frame processing: {e}")
        import traceback
        traceback.print_exc()
        node.destroy_node()
        return False


SEQUENCES = {
    'detect_cup_gripper': detect_cup_gripper,
}