# RF-DETR (local) detector replacing Roboflow SDK.
# - RTSP reader thread with reconnects
# - Resize + RGB conversion
# - RF-DETR local .pth weights (auto-download on first run)
# - Threshold + NMS + size/aspect filters + ROI overlap
# - Assign to 4 cup positions + history voting

import os
import cv2
import time

import types
import numpy as np
import logging
import threading
import importlib.util
from collections import deque

# ---- Logging (simple; swap with your logger_config if desired) ----
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
log = logging.getLogger("rfdetr-app")

# ---- RF-DETR + Supervision ----
from PIL import Image
import torch
from torchvision.ops import nms
import supervision as sv

from rfdetr import RFDETRLarge  # choose RFDETRLarge() if you want the large model
from rfdetr.util.coco_classes import COCO_CLASSES


# ------------------------------
# Config loader (from config.py)
# ------------------------------

def _load_config_module(path: str) -> types.ModuleType:
    spec = importlib.util.spec_from_file_location("app_config", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load config from: {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Config:
    """Materialize config values with safe defaults."""
    def __init__(self, mod: types.ModuleType):
        g = mod.__dict__

        # Camera
        self.rtsp_url = g.get("RTSP_URL", "")

        # RF-DETR local inference (no API key needed)
        # You can set RFDETR_VARIANT="base" or "large" in config.py if you like.
        self.rfdetr_variant = str(g.get("RFDETR_VARIANT", "base")).lower()
        self.confidence = float(g.get("RFDETR_CONFIDENCE", 0.30))
        self.allowed_classes = set(map(str, g.get("ALLOWED_CLASSES", ["cup"])))  # default to "cup" only
        
        # Local model paths
        model_paths = g.get("RFDETR_MODEL_PATHS", {})
        self.model_path = model_paths.get(self.rfdetr_variant)

        # Preprocess sizing
        self.max_side = int(g.get("MAX_SIDE", 960))

        # ROI & cups (edited by roi_selector.py)
        self.roi_polygon = np.array(g.get("ROI_POLYGON", []), dtype=np.int32)
        self.cup_positions = list(g.get("CUP_POSITIONS", [(0, 0), (0, 0), (0, 0), (0, 0)]))

        # Milk dispenser specific config
        self.milk_roi_polygon = np.array(g.get("MILK_ROI_POLYGON", []), dtype=np.int32)
        self.milk_cup_positions = list(g.get("MILK_CUP_POSITIONS", [(0, 0), (0, 0), (0, 0), (0, 0)]))
        self.milk_debug_folder = g.get("MILK_DEBUG_FOLDER", "debug_frames/milk")
        milk_classes = g.get("MILK_ALLOWED_CLASSES", None)
        self.milk_allowed_classes = milk_classes if milk_classes is not None else g.get("ALLOWED_CLASSES", [])

        # Sauce dispenser specific config
        self.sauce_roi_polygon = np.array(g.get("SAUCE_ROI_POLYGON", []), dtype=np.int32)
        self.sauce_cup_positions = list(g.get("SAUCE_CUP_POSITIONS", [(0, 0), (0, 0), (0, 0), (0, 0)]))
        self.sauce_debug_folder = g.get("SAUCE_DEBUG_FOLDER", "debug_frames/sauce")
        sauce_classes = g.get("SAUCE_ALLOWED_CLASSES", None)
        self.sauce_allowed_classes = sauce_classes if sauce_classes is not None else g.get("ALLOWED_CLASSES", [])

        # Station detection classes (all classes if None or empty)
        station_classes = g.get("STATION_ALLOWED_CLASSES", None)
        self.station_allowed_classes = station_classes if station_classes is not None else g.get("ALLOWED_CLASSES", [])

        # ROI cropping padding
        self.roi_padding = int(g.get("ROI_PADDING", 50))

        # Filters / heuristics
        self.min_cup_size = int(g.get("MIN_CUP_SIZE", 16))
        self.max_cup_size = int(g.get("MAX_CUP_SIZE", 10_000))
        self.aspect_min = float(g.get("ASPECT_RATIO_MIN", 0.25))
        self.aspect_max = float(g.get("ASPECT_RATIO_MAX", 4.0))
        self.roi_overlap_threshold = float(g.get("ROI_OVERLAP_THRESHOLD", 0.3))
        
        # Distance threshold for cup assignment (pixels in resized frame)
        # Only assign detection to cup if within this distance
        self.max_cup_distance = float(g.get("MAX_CUP_DISTANCE", 150.0))

        # History / voting
        self.frames_vote = int(g.get("FRAMES", 5))
        self.threshold_vote = int(g.get("THRESHOLD", 3))
        self.history_size = int(g.get("DETECTION_HISTORY_SIZE", 10))

        # Buffering / skipping
        self.frame_buffer_size = int(g.get("FRAME_BUFFER_SIZE", 2))
        self.enable_frame_skipping = bool(g.get("ENABLE_FRAME_SKIPPING", True))
        self.skip_frames = int(g.get("SKIP_FRAMES", 1))          # infer every (skip+1)th detect()

        # Connection & retry
        self.reconnect_retries = int(g.get("RECONNECT_RETRIES", 999_999))
        self.reconnect_delay = float(g.get("RECONNECT_DELAY", 0.5))
        self.retry_backoff = float(g.get("RETRY_BACKOFF", 1.5))
        self.max_retry_delay = float(g.get("MAX_RETRY_DELAY", 10.0))
        self.connection_timeout = float(g.get("CONNECTION_TIMEOUT", 5.0))
        self.frame_timeout = float(g.get("FRAME_TIMEOUT", 3.0))

        # Debug
        self.debug_mode = bool(g.get("DEBUG_MODE", False))
        self.save_frames = bool(g.get("SAVE_FRAMES", False))
        self.debug_folder = g.get("DEBUG_FOLDER", "debug_frames")


# ------------------------------
# RTSP reader thread
# ------------------------------

class RTSPStreamReader:
    def __init__(self, url: str, buffer_size: int, cfg: Config):
        self.url = url
        self.buffer = deque(maxlen=max(1, buffer_size))
        self.cfg = cfg
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name="RTSPReader", daemon=True)
        self.connected = False
        self.attempts = 0
        self.last_frame_ts = 0.0
        self._cap = None

    def start(self):
        self.thread.start()

    def _open(self):
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
        # Small delay to avoid port conflicts on rapid restarts
        time.sleep(0.1)
        cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._cap = cap

    def _run(self):
        delay = self.cfg.reconnect_delay
        while not self.stop_event.is_set():
            try:
                if self._cap is None or not self._cap.isOpened():
                    self.attempts += 1
                    self._open()
                    t0 = time.monotonic()
                    while (time.monotonic() - t0) < self.cfg.connection_timeout:
                        if self._cap.isOpened():
                            break
                        time.sleep(0.05)
                    self.connected = self._cap.isOpened()
                    if not self.connected:
                        log.warning(f"RTSP open failed (attempt {self.attempts}); retrying in {delay:.1f}s")
                        time.sleep(delay)
                        delay = min(self.cfg.max_retry_delay, delay * self.cfg.retry_backoff)
                        continue
                    delay = self.cfg.reconnect_delay

                ok, frame = self._cap.read()
                if not ok or frame is None:
                    time.sleep(0.01)
                    if (time.monotonic() - self.last_frame_ts) > self.cfg.frame_timeout:
                        self.connected = False
                        try:
                            self._cap.release()
                        except Exception:
                            pass
                        self._cap = None
                    continue

                self.last_frame_ts = time.monotonic()
                self.connected = True
                with self.lock:
                    self.buffer.append(frame)

            except Exception:
                self.connected = False
                log.exception("RTSP read error")
                try:
                    if self._cap is not None:
                        self._cap.release()
                except Exception:
                    pass
                self._cap = None
                time.sleep(delay)
                delay = min(self.cfg.max_retry_delay, delay * self.cfg.retry_backoff)

    def get_latest(self):
        with self.lock:
            return None if not self.buffer else self.buffer[-1].copy()

    def status(self):
        connected = self.connected
        age = (time.monotonic() - self.last_frame_ts) if self.last_frame_ts else None
        return {"status": "Connected" if connected else "Disconnected",
                "connected": connected, "attempts": self.attempts,
                "last_frame_age_s": age}

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        try:
            if self._cap is not None:
                self._cap.release()
                # Force close the connection
                self._cap = None
        except Exception:
            pass
        self._cap = None


# ------------------------------
# Utility: polygon resize + ROI integral mask
# ------------------------------

def _resize_polygon(poly: np.ndarray, scale_x: float, scale_y: float) -> np.ndarray:
    if poly.size == 0:
        return poly
    P = poly.astype(np.float32)
    P[:, 0] *= scale_x
    P[:, 1] *= scale_y
    return np.round(P).astype(np.int32)

def _build_roi_integral(h: int, w: int, roi_poly: np.ndarray):
    if roi_poly.size == 0:
        mask = np.ones((h, w), dtype=np.uint8)
    else:
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, [roi_poly], 1)
    integral = cv2.integral(mask)  # (h+1, w+1)
    return mask, integral


# ------------------------------
# Main detector
# ------------------------------

# Sentinel to distinguish between "not provided" and "explicitly None" for allowed_classes
_SENTINEL_ALLOWED_CLASSES = object()

class RFDETRDetector:
    def __init__(self, config_path: str = "config.py"):
        mod = _load_config_module(config_path)
        self.config = Config(mod)
        self.config_path = config_path  # Store for path resolution

        self._validate_config()
        
        # Warn if ROIs are identical (likely configuration error)
        if self.config.debug_mode:
            if (self.config.roi_polygon.size > 0 and 
                self.config.milk_roi_polygon.size > 0 and
                np.array_equal(self.config.roi_polygon, self.config.milk_roi_polygon)):
                log.warning("⚠️  Station ROI and Milk ROI are identical! Milk detection will use the same area as station.")
            if (self.config.roi_polygon.size > 0 and 
                self.config.sauce_roi_polygon.size > 0 and
                np.array_equal(self.config.roi_polygon, self.config.sauce_roi_polygon)):
                log.warning("⚠️  Station ROI and Sauce ROI are identical! Sauce detection will use the same area as station.")

        # Stream
        self.reader = RTSPStreamReader(
            url=self.config.rtsp_url,
            buffer_size=self.config.frame_buffer_size,
            cfg=self.config,
        )
        self.reader.start()

        # RF-DETR model (local)
        log.info("Initializing RF-DETR model...")
        
        # Use local model path if specified and file exists
        # Resolve model path relative to config file's directory
        model_kwargs = {}
        if self.config.model_path:
            # Make path absolute relative to config file's directory
            config_dir = os.path.dirname(os.path.abspath(self.config_path))
            abs_model_path = os.path.join(config_dir, self.config.model_path)
            
            if os.path.exists(abs_model_path):
                model_kwargs["pretrain_weights"] = abs_model_path
                log.info(f"Using local model: {abs_model_path}")
            else:
                log.warning(f"Model file not found at {abs_model_path}, will download default model")
                log.info(f"Using default model (will download if needed)")
        else:
            log.info(f"Using default model (will download if needed)")
        
        # Initialize model based on variant
        if self.config.rfdetr_variant == "large":
            self.model = RFDETRLarge(**model_kwargs)
        elif self.config.rfdetr_variant == "base":
            from rfdetr import RFDETRBase
            self.model = RFDETRBase(**model_kwargs)
        elif self.config.rfdetr_variant == "medium":
            from rfdetr import RFDETRMedium
            self.model = RFDETRMedium(**model_kwargs)
        else:
            log.warning(f"Unknown variant '{self.config.rfdetr_variant}', using large")
            self.model = RFDETRLarge(**model_kwargs)
            
        log.info("RF-DETR ready.")

        # Runtime state
        self._last_result = None
        self._ticks = 0

        # Prealloc / caches tied to resized dims
        self._prealloc_frame = None     # uint8 BGR (h', w', 3)
        self._input_shape = None        # (h', w')
        self._roi_poly_resized = None
        self._roi_mask = None
        self._roi_integral = None
        self._cups_resized = None

        # History
        self.history = {k: deque(maxlen=self.config.history_size) for k in range(4)}

        # Debug frame saving setup
        if self.config.debug_mode or self.config.save_frames:
            os.makedirs(self.config.debug_folder, exist_ok=True)
            self._frame_count = 0

        # Build name->id map for class filtering (supports dict or list)
        if isinstance(COCO_CLASSES, dict):
            if all(isinstance(k, int) for k in COCO_CLASSES.keys()):
                self.name2id = {v.lower(): k for k, v in COCO_CLASSES.items()}
            else:
                self.name2id = {k.lower(): int(v) for k, v in COCO_CLASSES.items()}
        else:
            self.name2id = {name.lower(): i for i, name in enumerate(COCO_CLASSES)}

        # Legacy: kept for backward compatibility
        # Load legacy ALLOWED_CLASSES from config module for backward compatibility
        mod = _load_config_module(self.config_path)
        allowed_classes_legacy = mod.__dict__.get("ALLOWED_CLASSES", [])
        self.target_ids = set()
        if allowed_classes_legacy:
            missing = [n for n in allowed_classes_legacy if n.lower() not in self.name2id]
            if missing:
                log.warning(f"Unknown class names in ALLOWED_CLASSES: {missing}")
            self.target_ids = {self.name2id[n.lower()] for n in allowed_classes_legacy if n.lower() in self.name2id}

    def _get_class_ids(self, allowed_classes):
        """
        Convert class names to class IDs.
        If allowed_classes is None or empty, returns None (all classes allowed).
        """
        if not allowed_classes:
            return None  # None means all classes
        target_ids = set()
        missing = [n for n in allowed_classes if n.lower() not in self.name2id]
        if missing:
            log.warning(f"Unknown class names: {missing}")
        target_ids = {self.name2id[n.lower()] for n in allowed_classes if n.lower() in self.name2id}
        return target_ids if target_ids else None

    def get_connection_status(self):
        return self.reader.status()

    def release(self):
        try:
            self.reader.stop()
            # Give the RTSP connection time to fully close
            time.sleep(0.5)
        except Exception:
            pass

    def _save_debug_frame(self, frame, bboxes=None, positions=None, all_dets=None, cup_assign=None, 
                          roi_poly=None, debug_folder=None, label_prefix="", filtered_dets=None):
        """Generic debug frame saving with configurable parameters"""
        if not (self.config.debug_mode or self.config.save_frames): 
            return
        try:
            folder = debug_folder or self.config.debug_folder
            os.makedirs(folder, exist_ok=True)
            dbg = frame.copy()
            
            # Draw ROI polygon if available
            if roi_poly is not None and len(roi_poly) > 0:
                cv2.polylines(dbg, [roi_poly], True, (0,255,0), 2)
            
            # Draw all detections (light orange)
            if all_dets:
                for i,(x1,y1,x2,y2) in enumerate(all_dets):
                    cv2.rectangle(dbg,(x1,y1),(x2,y2),(0,100,255),1)
                    cv2.putText(dbg,f"All {i+1}",(x1,y1-8),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,100,255),1)
            
            # Draw filtered out detections (yellow) - these failed filters
            if filtered_dets:
                for i,(x1,y1,x2,y2,reason) in enumerate(filtered_dets):
                    cv2.rectangle(dbg,(x1,y1),(x2,y2),(0,255,255),1)
                    cv2.putText(dbg,f"X:{reason}",(x1,y1-8),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,255,255),1)
            
            # Draw filtered detections (red) - these passed all filters
            if bboxes:
                for i,(x1,y1,x2,y2) in enumerate(bboxes):
                    cv2.rectangle(dbg,(x1,y1),(x2,y2),(0,0,255),2)
                    if cup_assign and i in cup_assign:
                        cv2.putText(dbg,f"{label_prefix}Cup {i+1}->Pos {cup_assign[i]+1}",(x1,y1-10),
                                    cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)
            
            # Draw cup positions (blue)
            if positions:
                for i,(x,y) in enumerate(positions):
                    cv2.circle(dbg,(x,y),8,(255,0,0),-1)
                    cv2.putText(dbg,f"{label_prefix}Pos {i+1}",(x+10,y-10),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,0),2)
                    # Draw distance threshold circle
                    cv2.circle(dbg,(x,y),int(self.config.max_cup_distance),(200,200,200),1)
            
            # Add timestamp and frame info
            ts = time.strftime("%H:%M:%S")
            cv2.putText(dbg,f"{label_prefix}Frame {getattr(self,'_frame_count',0)} - {ts}",(10,30),
                       cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255),2)
            
            # Save to debug folder (overwrites each time)
            filename = f"{folder}/debug_frame.jpg"
            cv2.imwrite(filename, dbg)
        except Exception as e:
            log.error(f"Error saving debug frame: {e}")

    # --------------- Generic Detection Method ---------------
    def _detect_generic(self, roi_polygon, cup_positions, debug_folder=None, label_prefix="", 
                        return_dict=True, num_positions=4, max_distance=None, allowed_classes=_SENTINEL_ALLOWED_CLASSES):
        """
        Generic detection method used by all detection functions.
        
        Args:
            roi_polygon: ROI polygon for filtering
            cup_positions: List of (x,y) cup positions
            debug_folder: Folder to save debug frames
            label_prefix: Prefix for debug labels
            return_dict: If True, return dict. If False, return bool (for single cup)
            num_positions: Number of cup positions (4 for station, 1 for milk/sauce)
            max_distance: Maximum distance for cup assignment (uses config default if None)
            allowed_classes: List of class names to allow (None = all classes)
            
        Returns:
            dict {0: bool, 1: bool, ...} or bool or {"error": "..."}
        """
        try:
            frame = self.reader.get_latest()
            if frame is None:
                return {"error": "No frame available yet."}

            orig_h, orig_w = frame.shape[:2]
            
            # Crop to ROI bounding box + padding if ROI is defined
            crop_x1, crop_y1, crop_x2, crop_y2 = 0, 0, orig_w, orig_h
            crop_offset_x, crop_offset_y = 0, 0
            
            if roi_polygon.size > 0:
                # Get bounding box of ROI
                roi_x_min = int(np.min(roi_polygon[:, 0]))
                roi_y_min = int(np.min(roi_polygon[:, 1]))
                roi_x_max = int(np.max(roi_polygon[:, 0]))
                roi_y_max = int(np.max(roi_polygon[:, 1]))
                
                # Add padding
                padding = self.config.roi_padding
                crop_x1 = max(0, roi_x_min - padding)
                crop_y1 = max(0, roi_y_min - padding)
                crop_x2 = min(orig_w, roi_x_max + padding)
                crop_y2 = min(orig_h, roi_y_max + padding)
                
                # Crop frame
                frame = frame[crop_y1:crop_y2, crop_x1:crop_x2]
                crop_offset_x = crop_x1
                crop_offset_y = crop_y1
                
                # Adjust ROI polygon and cup positions relative to cropped frame
                roi_polygon = roi_polygon.copy()
                roi_polygon[:, 0] -= crop_offset_x
                roi_polygon[:, 1] -= crop_offset_y
                
                # Update dimensions
                orig_h = crop_y2 - crop_y1
                orig_w = crop_x2 - crop_x1

            scale = min(1.0, float(self.config.max_side) / float(max(orig_h, orig_w)))
            new_w = int(orig_w * scale)
            new_h = int(orig_h * scale)
            if new_w < 2 or new_h < 2:
                return {"error": "Resized frame too small."}

            # Build ROI caches
            if roi_polygon.size > 0:
                sx = new_w / float(orig_w)
                sy = new_h / float(orig_h)
                roi_poly_resized = _resize_polygon(roi_polygon, sx, sy)
            else:
                roi_poly_resized = np.array([], dtype=np.int32)
            roi_mask, roi_integral = _build_roi_integral(new_h, new_w, roi_poly_resized)

            # Resize cup positions (already adjusted for crop)
            cups = np.array(cup_positions, dtype=np.float32)
            if len(cups.shape) == 1:  # Handle single position
                cups = cups.reshape(1, -1)
            # Adjust for crop offset first
            cups[:, 0] -= crop_offset_x
            cups[:, 1] -= crop_offset_y
            # Then resize
            cups[:, 0] *= (new_w / float(orig_w))
            cups[:, 1] *= (new_h / float(orig_h))
            cups_resized = cups.astype(np.float32)

            # Resize frame
            resized_frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)

            # ---- RF-DETR inference ----
            pil_img = Image.fromarray(cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB))
            detections = self.model.predict(pil_img, threshold=self.config.confidence)

            # Supervision → tensors
            boxes_xyxy = torch.as_tensor(detections.xyxy, dtype=torch.float32)
            scores     = torch.as_tensor(detections.confidence, dtype=torch.float32)
            class_ids  = torch.as_tensor(detections.class_id, dtype=torch.int64)

            # Store original detections for debug
            all_dets_list = [(int(x1), int(y1), int(x2), int(y2)) for x1, y1, x2, y2 in detections.xyxy] if detections.xyxy.size > 0 else []
            filtered_dets = []  # Store filtered out detections with reason

            # Confidence filter
            keep = scores >= self.config.confidence
            boxes_xyxy, scores, class_ids = boxes_xyxy[keep], scores[keep], class_ids[keep]

            # Class filter (use dynamic allowed_classes if provided)
            # Use sentinel to distinguish between "not provided" and "explicitly None"
            if allowed_classes is not _SENTINEL_ALLOWED_CLASSES:
                # Explicitly provided (could be None, empty list, or list of classes)
                target_ids = self._get_class_ids(allowed_classes)
                # target_ids will be None if allowed_classes is None or empty (all classes allowed)
            else:
                # Not provided (default parameter), use legacy filter
                target_ids = self.target_ids
            
            # Only filter if target_ids is set (not None and not empty)
            if target_ids:
                try:
                    mask = torch.isin(class_ids, torch.tensor(list(target_ids), dtype=class_ids.dtype))
                except AttributeError:
                    mask = torch.tensor([int(int(cid) in target_ids) for cid in class_ids], dtype=torch.bool)
                
                # Track filtered by class
                for i, keep_det in enumerate(mask):
                    if not keep_det and i < len(boxes_xyxy):
                        box = boxes_xyxy[i].cpu().numpy()
                        filtered_dets.append((int(box[0]), int(box[1]), int(box[2]), int(box[3]), "class"))
                
                boxes_xyxy, scores, class_ids = boxes_xyxy[mask], scores[mask], class_ids[mask]

            if boxes_xyxy.numel() == 0:
                if self.config.debug_mode or self.config.save_frames:
                    self._save_debug_frame(resized_frame, bboxes=[], positions=[(int(x), int(y)) for x, y in cups_resized],
                                          all_dets=all_dets_list, cup_assign={}, roi_poly=roi_poly_resized,
                                          debug_folder=debug_folder, label_prefix=label_prefix, filtered_dets=filtered_dets)
                return {i: False for i in range(num_positions)} if return_dict else False

            # NMS
            keep_nms = nms(boxes_xyxy, scores, 0.5)
            
            # Track filtered by NMS
            nms_mask = torch.zeros(len(boxes_xyxy), dtype=torch.bool)
            nms_mask[keep_nms] = True
            for i, keep_det in enumerate(nms_mask):
                if not keep_det:
                    box = boxes_xyxy[i].cpu().numpy()
                    filtered_dets.append((int(box[0]), int(box[1]), int(box[2]), int(box[3]), "NMS"))
            
            boxes_xyxy, scores, class_ids = boxes_xyxy[keep_nms], scores[keep_nms], class_ids[keep_nms]

            if boxes_xyxy.numel() == 0:
                if self.config.debug_mode or self.config.save_frames:
                    self._save_debug_frame(resized_frame, bboxes=[], positions=[(int(x), int(y)) for x, y in cups_resized],
                                          all_dets=all_dets_list, cup_assign={}, roi_poly=roi_poly_resized,
                                          debug_folder=debug_folder, label_prefix=label_prefix, filtered_dets=filtered_dets)
                return {i: False for i in range(num_positions)} if return_dict else False

            # Size & aspect filters
            bw = (boxes_xyxy[:, 2] - boxes_xyxy[:, 0]).clamp(1)
            bh = (boxes_xyxy[:, 3] - boxes_xyxy[:, 1]).clamp(1)
            area_ok = (bw >= self.config.min_cup_size) & (bh >= self.config.min_cup_size) \
                      & (bw <= self.config.max_cup_size) & (bh <= self.config.max_cup_size)
            aspect = bw / bh
            aspect_ok = (aspect >= self.config.aspect_min) & (aspect <= self.config.aspect_max)
            keep = area_ok & aspect_ok
            
            # Track filtered by size/aspect
            for i, keep_det in enumerate(keep):
                if not keep_det:
                    box = boxes_xyxy[i].cpu().numpy()
                    reason = "size" if not area_ok[i] else "aspect"
                    filtered_dets.append((int(box[0]), int(box[1]), int(box[2]), int(box[3]), reason))
            
            boxes_xyxy = boxes_xyxy[keep]
            
            if boxes_xyxy.numel() == 0:
                if self.config.debug_mode or self.config.save_frames:
                    self._save_debug_frame(resized_frame, bboxes=[], positions=[(int(x), int(y)) for x, y in cups_resized],
                                          all_dets=all_dets_list, cup_assign={}, roi_poly=roi_poly_resized,
                                          debug_folder=debug_folder, label_prefix=label_prefix, filtered_dets=filtered_dets)
                return {i: False for i in range(num_positions)} if return_dict else False

            # ROI overlap filter
            if roi_integral is not None and roi_polygon.size > 0:
                H, W = roi_mask.shape[:2]
                x1 = boxes_xyxy[:, 0].to(torch.int32).clamp(0, W - 1)
                y1 = boxes_xyxy[:, 1].to(torch.int32).clamp(0, H - 1)
                x2 = boxes_xyxy[:, 2].to(torch.int32).clamp(0, W - 1)
                y2 = boxes_xyxy[:, 3].to(torch.int32).clamp(0, H - 1)

                S = torch.as_tensor(roi_integral, dtype=torch.int64)
                roi_pix = S[y2 + 1, x2 + 1] - S[y1, x2 + 1] - S[y2 + 1, x1] + S[y1, x1]
                box_area = (x2 - x1 + 1) * (y2 - y1 + 1)
                overlap = roi_pix.to(torch.float32) / torch.clamp(box_area.to(torch.float32), min=1.0)
                keep = overlap >= self.config.roi_overlap_threshold
                
                # Track filtered by ROI
                for i, keep_det in enumerate(keep):
                    if not keep_det:
                        box = boxes_xyxy[i].cpu().numpy()
                        filtered_dets.append((int(box[0]), int(box[1]), int(box[2]), int(box[3]), "ROI"))
                
                boxes_xyxy = boxes_xyxy[keep]

            # Assign to cup positions with distance threshold
            result = {i: False for i in range(num_positions)} if return_dict else False
            cup_assign = {}
            
            if cups_resized is not None and len(cups_resized) >= 1 and boxes_xyxy.numel() > 0:
                cx = 0.5 * (boxes_xyxy[:, 0] + boxes_xyxy[:, 2])
                cy = 0.5 * (boxes_xyxy[:, 1] + boxes_xyxy[:, 3])
                centers = torch.stack([cx, cy], dim=1).cpu().numpy()
                cups_np = cups_resized.astype(np.float32)
                boxes_np = boxes_xyxy.cpu().numpy()
                
                # Calculate distances
                d2 = ((centers[:, None, :] - cups_np[None, :, :]) ** 2).sum(axis=2)
                assign = np.argmin(d2, axis=1)
                min_distances = np.min(d2, axis=1)
                
                # Use distance threshold
                distance_threshold = max_distance if max_distance is not None else self.config.max_cup_distance
                distance_threshold_sq = distance_threshold ** 2
                
                # Track detections filtered by distance
                valid_assignments = min_distances <= distance_threshold_sq
                for i, (is_valid, min_dist) in enumerate(zip(valid_assignments, min_distances)):
                    if is_valid:
                        cup_id = assign[i]
                        cup_pos = cups_np[cup_id]
                        box = boxes_np[i]
                        
                        # Check if cup position point is inside the detected cup's bounding box
                        x1, y1, x2, y2 = box[0], box[1], box[2], box[3]
                        cup_x, cup_y = cup_pos[0], cup_pos[1]
                        
                        point_inside_box = (x1 <= cup_x <= x2) and (y1 <= cup_y <= y2)
                        
                        if point_inside_box:
                            if return_dict:
                                result[cup_id] = True
                            else:
                                result = True
                            cup_assign[i] = int(cup_id)
                        else:
                            # Cup position point is not inside the bounding box
                            filtered_dets.append((int(box[0]), int(box[1]), int(box[2]), int(box[3]), "not_in_box"))
                    else:
                        # This detection is too far from any cup position
                        box = boxes_xyxy[i].cpu().numpy()
                        filtered_dets.append((int(box[0]), int(box[1]), int(box[2]), int(box[3]), "dist"))

            # Save debug frame
            if self.config.debug_mode or self.config.save_frames:
                bboxes_list = [(int(x1), int(y1), int(x2), int(y2)) for x1, y1, x2, y2 in boxes_xyxy.cpu().numpy()] if boxes_xyxy.numel() > 0 else []
                positions_list = [(int(x), int(y)) for x, y in cups_resized] if cups_resized is not None else []
                
                self._save_debug_frame(resized_frame, bboxes_list, positions_list, all_dets_list, cup_assign,
                                      roi_poly=roi_poly_resized, debug_folder=debug_folder,
                                      label_prefix=label_prefix, filtered_dets=filtered_dets)

            return result

        except Exception:
            log.exception(f"{label_prefix}detect() failure")
            return {"error": f"Internal error during {label_prefix}detect()."}

    # --------------- Detection Methods ---------------
    def detect_cups_on_station(self):
        """
        Main cup detection for 4 positions on the station.
        Note: Position mapping is FLIPPED/REVERSED (mirrored).
        Camera position 0 → Output position 3, Camera position 1 → Output position 2, etc.
        Returns: {0: bool, 1: bool, 2: bool, 3: bool} or {"error": "..."}
        """
        # Frame skipping
        self._ticks += 1
        if self.config.enable_frame_skipping:
            stride = max(1, int(self.config.skip_frames) + 1)
            if (self._ticks % stride) != 0 and self._last_result is not None:
                return dict(self._last_result)
        
        # Increment frame count for debug
        if self.config.debug_mode or self.config.save_frames:
            self._frame_count += 1
        
        # Use generic detection method (station uses all classes)
        present = self._detect_generic(
            roi_polygon=self.config.roi_polygon,
            cup_positions=self.config.cup_positions,
            debug_folder=self.config.debug_folder,
            label_prefix="",
            return_dict=True,
            num_positions=4,
            allowed_classes=self.config.station_allowed_classes
        )
        
        # Apply voting/history if it's a valid result
        if isinstance(present, dict) and "error" not in present:
            voted_result = self._vote_presence(present)
            # Flip/reverse the position mapping (mirror the positions)
            # Position 0 → Position 3, Position 1 → Position 2, etc.
            flipped_result = {
                0: voted_result.get(3, False),
                1: voted_result.get(2, False),
                2: voted_result.get(1, False),
                3: voted_result.get(0, False)
            }
            self._last_result = flipped_result
            return dict(self._last_result)
        
        return present

    # --------------- Voting / History ---------------
    def _vote_presence(self, present_now: dict):
        for k in range(4):
            self.history[k].append(bool(present_now.get(k, False)))

        voted = {}
        win = max(1, self.config.frames_vote)
        thr = max(1, min(win, self.config.threshold_vote))
        for k in range(4):
            recent = list(self.history[k])[-win:]
            voted[k] = (sum(1 for v in recent if v) >= thr)
        return voted

    # --------------- Validation ---------------
    def _validate_config(self):
        if not self.config.rtsp_url:
            raise ValueError("RTSP_URL is required in config.py")
        if not (0.0 <= self.config.confidence <= 1.0):
            raise ValueError("RFDETR_CONFIDENCE must be within [0,1].")
        if self.config.max_side < 320:
            raise ValueError("MAX_SIDE too small; set at least 320.")
        if len(self.config.cup_positions) != 4:
            raise ValueError("CUP_POSITIONS must contain exactly 4 (x,y) points.")
        if self.config.roi_polygon.size != 0 and self.config.roi_polygon.shape[1] != 2:
            raise ValueError("ROI_POLYGON must be an Nx2 array.")

    # --------------- Milk Detection Method ---------------
    def detect_cup_milk_dispenser(self):
        """
        Milk dispenser detection using MILK_ROI_POLYGON and MILK_CUP_POSITIONS
        Returns: bool or {"error": "..."}
        """
        # Debug: log which ROI is being used
        if self.config.debug_mode:
            log.info(f"Milk detection using ROI polygon with {len(self.config.milk_roi_polygon)} points")
            if self.config.milk_roi_polygon.size > 0:
                log.info(f"Milk ROI bounds: x=[{np.min(self.config.milk_roi_polygon[:, 0])}, {np.max(self.config.milk_roi_polygon[:, 0])}], y=[{np.min(self.config.milk_roi_polygon[:, 1])}, {np.max(self.config.milk_roi_polygon[:, 1])}]")
        
        return self._detect_generic(
            roi_polygon=self.config.milk_roi_polygon,
            cup_positions=self.config.milk_cup_positions,
            debug_folder=self.config.milk_debug_folder,
            label_prefix="Milk ",
            return_dict=False,
            num_positions=1,
            allowed_classes=self.config.milk_allowed_classes
        )

    # --------------- Sauce Detection Method ---------------
    def detect_cup_sauce_dispenser(self):
        """
        Sauce dispenser detection using SAUCE_ROI_POLYGON and SAUCE_CUP_POSITIONS
        Returns: bool or {"error": "..."}
        """
        return self._detect_generic(
            roi_polygon=self.config.sauce_roi_polygon,
            cup_positions=self.config.sauce_cup_positions,
            debug_folder=self.config.sauce_debug_folder,
            label_prefix="Sauce ",
            return_dict=False,
            num_positions=1,
            allowed_classes=self.config.sauce_allowed_classes
        )


# Alias for backward compatibility with imports
CupDetector = RFDETRDetector