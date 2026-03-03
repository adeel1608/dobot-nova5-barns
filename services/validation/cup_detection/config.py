import numpy as np
# Production config – trimmed to only what the code actually reads
# If your password contains '@', encode it as %40.
RTSP_URL = "rtsp://admin:QSS2030QSS@192.168.200.106:554/stream1"

# RF-DETR local model settings
RFDETR_VARIANT = "large"           # "base" or "large"
RFDETR_CONFIDENCE = 0.1        # 0..1 (lowered to catch more cups at milk dispenser)

# Class filters per detection type
# Station detection: use all classes (set to None or empty list)
STATION_ALLOWED_CLASSES = None  # None = all classes, or [] = all classes
# Milk/Sauce detection: only cup and bowl
MILK_ALLOWED_CLASSES = ["cup", "bowl","glass","glass_with_handle","plate"]
SAUCE_ALLOWED_CLASSES = ["cup", "bowl","glass","glass_with_handle","plate"]

# Legacy: kept for backward compatibility (used as fallback)
ALLOWED_CLASSES = ["cup", "bowl","glass","glass_with_handle","plate"]     # Include bowl (cups sometimes detected as bowls)

# Local model paths (set to None to use default download behavior)
# Paths are relative to the config.py file location
RFDETR_MODEL_PATHS = {
    "large": "models/rf-detr-large.pth",      # Path to your local large model
    "base": "models/rf-detr-base.pth",        # Path to your local base model  
    "medium": "models/rf-detr-medium.pth"  # Path to your local medium model
}

# Preprocess
MAX_SIDE = 1920                 # resize longest side to this (keeps aspect)

# ROI & cups
# Provide polygon as list of (x,y). Example below is placeholder.
ROI_POLYGON = np.array([
    [863, 571],
    [857, 1021],
    [994, 1025],
    [1006, 563]
], dtype=np.int32)
# Expected cup centers (pixels). Update to your layout.
CUP_POSITIONS = [
    (925, 967),
    (931, 855),
    (924, 734),
    (928, 615)
]

# Filters / heuristics
MIN_CUP_SIZE = 15                  # px (min bbox min side) - lowered to catch smaller cups
MAX_CUP_SIZE = 300                 # px (max bbox max side)
ASPECT_RATIO_MIN = 0.1           # w/h lower bound
ASPECT_RATIO_MAX = 2.5             # w/h upper bound - slightly increased
ROI_OVERLAP_THRESHOLD = 0.20       # IoU with ROI mask for acceptance - lowered for milk dispenser

# Distance threshold for cup assignment (reduce false positives)
# Only assign detection to cup position if within this distance (pixels)
MAX_CUP_DISTANCE = 180.0           # px (increased to catch all cups on station)

# History / voting
FRAMES = 1                         # detections to aggregate per result
THRESHOLD = 1                      # min votes to accept
DETECTION_HISTORY_SIZE = 20

# Buffering / skipping
FRAME_BUFFER_SIZE = 3
ENABLE_FRAME_SKIPPING = True
SKIP_FRAMES = 0

# Connection & retry
RECONNECT_RETRIES = 999999
RECONNECT_DELAY = 0.5              # initial seconds before retry
RETRY_BACKOFF = 1.5                # multiplier
MAX_RETRY_DELAY = 10.0             # seconds (cap)
CONNECTION_TIMEOUT = 5.0           # seconds for initial connect
FRAME_TIMEOUT = 3.0                # seconds since last fresh frame before considered stale

# Debug
DEBUG_MODE = True
SAVE_FRAMES = True
DEBUG_FOLDER = "debug_frames"


# --- Per-dispenser ROI & cup positions (edited by roi_selector.py) ---
# Milk dispenser configuration
MILK_ROI_POLYGON = np.array([
    [767, 50],
    [763, 121],
    [855, 111],
    [851, 43]
], dtype=np.int32)

MILK_CUP_POSITIONS = [
    (804, 77)
]

# Sauce dispenser configuration  
SAUCE_ROI_POLYGON = np.array([
    [670, 65],
    [663, 140],
    [743, 123],
    [748, 53]
], dtype=np.int32)

SAUCE_CUP_POSITIONS = [
    (704, 94)
]
# --- Per-dispenser debug folders ---
MILK_DEBUG_FOLDER = "debug_frames/milk"
SAUCE_DEBUG_FOLDER = "debug_frames/sauce"

# ROI cropping settings
# Padding around ROI before sending to model (pixels in original frame)
ROI_PADDING = 2  # Extra pixels around ROI bounding box
