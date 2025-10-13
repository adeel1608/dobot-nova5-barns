"""
Configuration for Cup Detection System
Modify these settings for your setup
"""

import numpy as np

# Camera Settings
RTSP_URL = "rtsp://admin:123456@192.168.200.60:554/stream1"

# Model Settings
MODEL_PATH = "models/yolov8l.pt"
DEVICE = "cpu"  # Use "cpu" if no GPU
CONFIDENCE = 0.05  # Even lower confidence for better detection
IMG_SIZE = 1920     # Larger image size for better detection
PROCESS_SCALE = 0.5  # Reduced scale for faster processing

# Detection Settings - OPTIMIZED FOR SPEED
FRAMES = 1        # Use only 1 frame for maximum speed
THRESHOLD = 0.2   # Lower threshold for better detection

# Adaptive Detection Settings
DETECTION_HISTORY_SIZE = 5     # Reduced history for faster processing
POSITION_BOOST_RADIUS = 100    # Larger boost radius for detections near known positions (pixels)
MIN_CUP_SIZE = 15             # Smaller minimum cup size in pixels
MAX_CUP_SIZE = 300            # Larger maximum cup size in pixels
ASPECT_RATIO_MIN = 0.3        # More lenient minimum aspect ratio (height/width)
ASPECT_RATIO_MAX = 3.0        # More lenient maximum aspect ratio (height/width)

# Enhanced ROI Detection Settings
ROI_OVERLAP_THRESHOLD = 0.1  # Lower minimum overlap ratio for ROI inclusion (0.0-1.0)
ROI_CENTER_WEIGHT = 0.7      # Weight for center point in ROI check (0.0-1.0)

# Transparent Cup Detection Settings
TRANSPARENT_BOOST_FACTOR = 1.5  # Higher boost factor for transparent cup detection
EDGE_DETECTION_ENABLED = True   # Enable edge-based detection for transparent cups
CONTOUR_ANALYSIS_ENABLED = True # Enable contour analysis for transparent cups
REFLECTION_FILTER_ENABLED = True # Enable reflection filtering

# Performance Optimization Settings
FRAME_BUFFER_SIZE = 3       # Keep only last 3 frames in buffer
SKIP_FRAMES = 1             # Skip 1 frame for faster processing (0 = no skip)
ENABLE_FRAME_SKIPPING = True # Enable frame skipping for speed

# Debug Settings
DEBUG_MODE = True  # Set to True to enable debug output
SAVE_FRAMES = True  # Set to True to save processed frames
DEBUG_FOLDER = "debug_frames"  # Folder to save debug frames
MAX_DEBUG_FRAMES = 10  # Maximum number of debug frames to keep
CLEANUP_AFTER_DETECTION = True  # Automatically cleanup old debug frames



# ROI (Region of Interest) - Adjust these coordinates for your camera
# Default coordinates for 1920x1080 resolution
ROI_POLYGON = np.array([
    [821, 653],
    [1377, 442],
    [1693, 534],
    [1086, 842]
], dtype=np.int32)

# Cup Positions - Adjust these coordinates for your setup
# Default positions for 1920x1080 resolution (4 cups in a row)
CUP_POSITIONS = [
    (1067, 688),
    (1232, 615),
    (1345, 565),
    (1427, 534)
]

# Connection Settings
RECONNECT_RETRIES = 999999
RECONNECT_DELAY = 0.5

# RTSP Connection Settings
CONNECTION_TIMEOUT = 5.0      # Connection timeout in seconds
FRAME_TIMEOUT = 3.0           # Frame read timeout in seconds
RETRY_DELAY = 1.0             # Initial retry delay in seconds
MAX_RETRY_DELAY = 10.0        # Maximum retry delay in seconds
RETRY_BACKOFF = 1.5           # Exponential backoff multiplier