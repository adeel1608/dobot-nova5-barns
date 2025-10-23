import numpy as np
# Production config – trimmed to only what the code actually reads
# If your password contains '@', encode it as %40.
RTSP_URL = "rtsp://admin:QSS2030QSS@192.168.200.106:554/stream1"

# RF-DETR local model settings
RFDETR_VARIANT = "large"           # "base" or "large"
RFDETR_CONFIDENCE = 0.20           # 0..1
ALLOWED_CLASSES = ["cup"]          # subset of model's label space

# Local model paths (set to None to use default download behavior)
RFDETR_MODEL_PATHS = {
    "large": "models/rf-detr-large.pth",      # Path to your local large model
    "base": "models/rf-detr-base.pth",        # Path to your local base model  
    "medium": "models/rf-detr-medium.pth"  # Path to your local medium model
}

# Preprocess
MAX_SIDE = 960                     # resize longest side to this (keeps aspect)

# ROI & cups
# Provide polygon as list of (x,y). Example below is placeholder.
ROI_POLYGON = np.array([
    [537, 311],
    [592, 191],
    [933, 295],
    [892, 430]
], dtype=np.int32)
# Expected cup centers (pixels). Update to your layout.
CUP_POSITIONS = [
    (870, 350),
    (808, 329),
    (736, 310),
    (663, 292)
]

# Filters / heuristics
MIN_CUP_SIZE = 20                  # px (min bbox min side)
MAX_CUP_SIZE = 300                 # px (max bbox max side)
ASPECT_RATIO_MIN = 0.5             # w/h lower bound
ASPECT_RATIO_MAX = 2.0             # w/h upper bound
ROI_OVERLAP_THRESHOLD = 0.30       # IoU with ROI mask for acceptance

# History / voting
FRAMES = 1                         # detections to aggregate per result
THRESHOLD = 1                      # min votes to accept
DETECTION_HISTORY_SIZE = 5

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