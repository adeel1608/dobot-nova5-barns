"""
Configuration for Cup Detection System
Modify these settings for your setup
"""

import numpy as np

# Camera Settings
SNAPSHOT_URL = "http://qltyss:QSS2030QSS@192.168.200.60/cgi-bin/snapshot.cgi"

# Model Settings
MODEL_PATH = "models/yolov8l.pt"
# DEVICE = "cuda:0"  # Use "cpu" if no GPU
DEVICE = "cpu"  # Use "cpu" if no GPU
CONFIDENCE = 0.15  # Lower confidence for better detection
IMG_SIZE = 640     # Larger image size for better detection
PROCESS_SCALE = 1.0  # Full resolution processing

# Detection Settings
FRAMES = 3        # Fewer frames for faster processing
THRESHOLD = 0.3   # Lower threshold for better detection

# Debug Settings
DEBUG_MODE = True  # Set to True to enable debug output
SAVE_FRAMES = True  # Set to True to save processed frames
DEBUG_FOLDER = "./debug_frames_cup"  # Folder to save debug frames

# ROI (Region of Interest) - Adjust these coordinates for your camera
# Default coordinates for 1920x1080 resolution
ROI_POLYGON = np.array([
    [284, 359],
    [465, 249],
    [514, 273],
    [356, 400]
], dtype=np.int32)

# Cup Positions - Adjust these coordinates for your setup
# Default positions for 1920x1080 resolution (4 cups in a row)
CUP_POSITIONS = [
    (366, 333),
    (408, 306),
    (446, 282),
    (480, 265)
]

# Connection Settings
RECONNECT_RETRIES = 999999
RECONNECT_DELAY = 0.5
