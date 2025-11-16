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

from pyorbbecsdk import *

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


def detect_cup_gripper(**params):
    """
    Detect if a transparent cup is in the gripper using depth camera.
    Runs headless (no GUI) and returns True if cup detected, False otherwise.
    
    Args:
        **params: Optional parameters (for compatibility with sequence interface)
    
    Returns:
        bool: True if cup detected, False otherwise
    """
    config = Config()
    pipeline = Pipeline()
    temporal_filter = TemporalFilter(alpha=0.5) # Alpha for temporal filtering
    
    try:
        profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        assert profile_list is not None
        # Get the default depth profile (often 640x480 or 1280x720, 30fps)
        depth_profile = profile_list.get_default_video_stream_profile()
        assert depth_profile is not None
        print("Depth profile: ", depth_profile)
        config.enable_stream(depth_profile)
    except Exception as e:
        print(f"Error enabling depth stream: {e}")
        return False

    try:
        pipeline.start(config)
    except Exception as e:
        print(f"Error starting pipeline: {e}")
        return False

    last_print_time = time.time()
    cup_detected = False

    try:
        # Capture a few frames to stabilize
        for _ in range(5):
            frames = pipeline.wait_for_frames(100) # Wait for frames for up to 100ms
            if frames is None:
                continue

            depth_frame = frames.get_depth_frame()
            if depth_frame is None:
                continue

            depth_format = depth_frame.get_format()
            if depth_format != OBFormat.Y16:
                # The camera might support other formats, but Y16 is standard for depth
                print("Depth format is not Y16, skipping frame.")
                continue

            width = depth_frame.get_width()
            height = depth_frame.get_height()
            scale = depth_frame.get_depth_scale() # Scale factor to convert depth units to millimeters

            # Convert depth data to a NumPy array (uint16 raw depth values)
            depth_data_raw = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
            depth_data_raw = depth_data_raw.reshape((height, width))

            # Apply temporal filtering *before* converting to real-world units for consistency
            # Ensure temporal filter operates on a compatible type (e.g., uint16)
            filtered_depth_data_raw = temporal_filter.process(depth_data_raw)

            # Convert to float32 and apply depth scale to get depth in millimeters
            depth_data_mm = filtered_depth_data_raw.astype(np.float32) * scale

            # Apply depth range filter (MIN_DEPTH to MAX_DEPTH)
            # Pixels outside this range are set to 3000 mm (background)
            depth_data_filtered = np.where((depth_data_mm >= MIN_DEPTH) & (depth_data_mm <= MAX_DEPTH),
                                            depth_data_mm, 3000) # Set out-of-range to 3000 mm

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

            if total_roi_pixels > 0: # Avoid division by zero if ROI is empty
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
                center_distance = 0 # Or np.nan, or a specific string for 'missing'

            current_time = time.time()
            if current_time - last_print_time >= PRINT_INTERVAL:
                print(f"[CV] Center distance: {center_distance:.1f} mm, {detection_status_text}")
                last_print_time = current_time
        
        # After capturing frames, stop pipeline and return result
        pipeline.stop()
        print(f"[CV] Cup detection result: {cup_detected}")
        return cup_detected
        
    except KeyboardInterrupt:
        print("[CV] Exiting by KeyboardInterrupt.")
        pipeline.stop()
        return False
    except Exception as e:
        print(f"[CV] An error occurred during frame processing: {e}")
        pipeline.stop()
        return False


SEQUENCES = {
    'detect_cup_gripper': detect_cup_gripper,
}