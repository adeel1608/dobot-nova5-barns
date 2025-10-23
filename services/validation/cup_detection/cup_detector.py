# RF-DETR (local) detector replacing Roboflow SDK.
# - RTSP reader thread with reconnects
# - Resize + RGB conversion
# - RF-DETR local .pth weights (auto-download on first run)
# - Threshold + NMS + size/aspect filters + ROI overlap
# - Assign to 4 cup positions + history voting

import os
import cv2
import time
import math
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
        # Resolve relative paths against this directory
        if self.model_path and not os.path.isabs(self.model_path):
            self.model_path = os.path.join(os.path.dirname(__file__), self.model_path)

        # Preprocess sizing
        self.max_side = int(g.get("MAX_SIDE", 960))

        # ROI & cups (edited by roi_selector.py)
        self.roi_polygon = np.array(g.get("ROI_POLYGON", []), dtype=np.int32)
        self.cup_positions = list(g.get("CUP_POSITIONS", [(0, 0), (0, 0), (0, 0), (0, 0)]))

        # Filters / heuristics
        self.min_cup_size = int(g.get("MIN_CUP_SIZE", 16))
        self.max_cup_size = int(g.get("MAX_CUP_SIZE", 10_000))
        self.aspect_min = float(g.get("ASPECT_RATIO_MIN", 0.25))
        self.aspect_max = float(g.get("ASPECT_RATIO_MAX", 4.0))
        self.roi_overlap_threshold = float(g.get("ROI_OVERLAP_THRESHOLD", 0.3))

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

class CupDetector:
    def __init__(self, config_path: str = "config.py"):
        mod = _load_config_module(config_path)
        self.config = Config(mod)

        self._validate_config()

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
        model_kwargs = {}
        log.info(f"Model path from config: {self.config.model_path}")
        log.info(f"Model path exists: {os.path.exists(self.config.model_path) if self.config.model_path else False}")
        log.info(f"Current working directory: {os.getcwd()}")
        log.info(f"Config file directory: {os.path.dirname(__file__)}")
        
        if self.config.model_path and os.path.exists(self.config.model_path):
            model_kwargs["pretrain_weights"] = self.config.model_path
            log.info(f"✅ Using local model: {self.config.model_path}")
        else:
            log.info(f"⬇️ Using default model (will download if needed)")
            if self.config.model_path:
                log.warning(f"❌ Local model path not found: {self.config.model_path}")
        
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

        self.target_ids = set()
        if self.config.allowed_classes:
            missing = [n for n in self.config.allowed_classes if n.lower() not in self.name2id]
            if missing:
                log.warning(f"Unknown class names in ALLOWED_CLASSES: {missing}")
            self.target_ids = {self.name2id[n.lower()] for n in self.config.allowed_classes if n.lower() in self.name2id}

    def get_connection_status(self):
        return self.reader.status()

    def release(self):
        try:
            self.reader.stop()
            # Give the RTSP connection time to fully close
            time.sleep(0.5)
        except Exception:
            pass

    def _save_debug_frame(self, frame, bboxes=None, positions=None, all_dets=None, cup_assign=None):
        if not (self.config.debug_mode or self.config.save_frames): 
            return
        try:
            os.makedirs(self.config.debug_folder, exist_ok=True)
            dbg = frame.copy()
            
            # Draw ROI polygon if available
            if self._roi_poly_resized is not None and len(self._roi_poly_resized) > 0:
                cv2.polylines(dbg, [self._roi_poly_resized], True, (0,255,0), 2)
            
            # Draw all detections (light orange)
            if all_dets:
                for i,(x1,y1,x2,y2) in enumerate(all_dets):
                    cv2.rectangle(dbg,(x1,y1),(x2,y2),(0,100,255),1)
                    cv2.putText(dbg,f"All {i+1}",(x1,y1-8),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,100,255),1)
            
            # Draw filtered detections (red)
            if bboxes:
                for i,(x1,y1,x2,y2) in enumerate(bboxes):
                    cv2.rectangle(dbg,(x1,y1),(x2,y2),(0,0,255),2)
                    if cup_assign and i in cup_assign:
                        cv2.putText(dbg,f"Cup {i+1}->Pos {cup_assign[i]+1}",(x1,y1-10),
                                    cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)
            
            # Draw cup positions (blue)
            if positions:
                for i,(x,y) in enumerate(positions):
                    cv2.circle(dbg,(x,y),8,(255,0,0),-1)
                    cv2.putText(dbg,f"Pos {i+1}",(x+10,y-10),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,0),2)
            
            # Add timestamp and frame info
            ts = time.strftime("%H:%M:%S")
            cv2.putText(dbg,f"Frame {self._frame_count} - {ts}",(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255),2)
            
            # Save to debug folder (overwrites each time)
            filename = f"{self.config.debug_folder}/debug_frame.jpg"
            cv2.imwrite(filename, dbg)
        except Exception as e:
            log.error(f"Error saving debug frame: {e}")

    # --------------- Detection ---------------
    def detect(self):
        """
        Returns: {1: bool, 2: bool, 3: bool, 4: bool} or {"error": "..."}
        Note: Results are reversed (position 0 -> 3, 1 -> 2, etc.) and 1-indexed
        """
        try:
            frame = self.reader.get_latest()
            if frame is None:
                return {"error": "No frame available yet."}

            orig_h, orig_w = frame.shape[:2]
            scale = min(1.0, float(self.config.max_side) / float(max(orig_h, orig_w)))
            new_w = int(orig_w * scale)
            new_h = int(orig_h * scale)
            if new_w < 2 or new_h < 2:
                return {"error": "Resized frame too small."}

            # (Re)allocate only on size change
            size_changed = (self._input_shape != (new_h, new_w))
            if size_changed:
                self._prealloc_frame = np.empty((new_h, new_w, 3), dtype=np.uint8)
                self._input_shape = (new_h, new_w)

                # Rebuild ROI caches
                if self.config.roi_polygon.size > 0:
                    sx = new_w / float(orig_w)
                    sy = new_h / float(orig_h)
                    self._roi_poly_resized = _resize_polygon(self.config.roi_polygon, sx, sy)
                else:
                    self._roi_poly_resized = np.array([], dtype=np.int32)
                self._roi_mask, self._roi_integral = _build_roi_integral(new_h, new_w, self._roi_poly_resized)

                # Resize cup positions
                cups = np.array(self.config.cup_positions, dtype=np.float32)
                cups[:, 0] *= (new_w / float(orig_w))
                cups[:, 1] *= (new_h / float(orig_h))
                self._cups_resized = cups.astype(np.float32)

            # Resize BGR -> prealloc
            cv2.resize(frame, (new_w, new_h), dst=self._prealloc_frame, interpolation=cv2.INTER_AREA)

            # Frame skipping
            self._ticks += 1
            if self.config.enable_frame_skipping:
                stride = max(1, int(self.config.skip_frames) + 1)
                if (self._ticks % stride) != 0 and self._last_result is not None:
                    return dict(self._last_result)

            # ---- RF-DETR inference (local) ----
            pil_img = Image.fromarray(cv2.cvtColor(self._prealloc_frame, cv2.COLOR_BGR2RGB))
            detections = self.model.predict(pil_img, threshold=self.config.confidence)

            # Supervision → tensors
            boxes_xyxy = torch.as_tensor(detections.xyxy, dtype=torch.float32)
            scores     = torch.as_tensor(detections.confidence, dtype=torch.float32)
            class_ids  = torch.as_tensor(detections.class_id, dtype=torch.int64)

            # Confidence filter (strict recheck)
            keep = scores >= self.config.confidence
            boxes_xyxy, scores, class_ids = boxes_xyxy[keep], scores[keep], class_ids[keep]

            # Class filter (only allowed_classes)
            if self.target_ids:
                try:
                    mask = torch.isin(class_ids, torch.tensor(list(self.target_ids), dtype=class_ids.dtype))
                except AttributeError:
                    mask = torch.tensor([int(int(cid) in self.target_ids) for cid in class_ids], dtype=torch.bool)
                boxes_xyxy, scores, class_ids = boxes_xyxy[mask], scores[mask], class_ids[mask]

            if boxes_xyxy.numel() == 0:
                # save a debug frame even when nothing detected
                if self.config.debug_mode or self.config.save_frames:
                    self._save_debug_frame(self._prealloc_frame, bboxes=[], positions=[], all_dets=[], cup_assign={})
                    self._frame_count += 1
                present = {0: False, 1: False, 2: False, 3: False}

                self._last_result = self._transform_result(self._vote_presence(present))
                return dict(self._last_result)

            # NMS
            keep_nms = nms(boxes_xyxy, scores, 0.5)
            boxes_xyxy, scores, class_ids = boxes_xyxy[keep_nms], scores[keep_nms], class_ids[keep_nms]

            if boxes_xyxy.numel() == 0:
                # save a debug frame even when nothing detected
                if self.config.debug_mode or self.config.save_frames:
                    self._save_debug_frame(self._prealloc_frame, bboxes=[], positions=[], all_dets=[], cup_assign={})
                    self._frame_count += 1
                present = {0: False, 1: False, 2: False, 3: False}
                self._last_result = self._transform_result(self._vote_presence(present))
                return dict(self._last_result)

            # Size & aspect filters
            bw = (boxes_xyxy[:, 2] - boxes_xyxy[:, 0]).clamp(1)
            bh = (boxes_xyxy[:, 3] - boxes_xyxy[:, 1]).clamp(1)
            area_ok = (bw >= self.config.min_cup_size) & (bh >= self.config.min_cup_size) \
                      & (bw <= self.config.max_cup_size) & (bh <= self.config.max_cup_size)
            aspect = bw / bh
            aspect_ok = (aspect >= self.config.aspect_min) & (aspect <= self.config.aspect_max)
            keep = area_ok & aspect_ok
            boxes_xyxy = boxes_xyxy[keep]
            if boxes_xyxy.numel() == 0:
                # save a debug frame even when nothing detected
                if self.config.debug_mode or self.config.save_frames:
                    self._save_debug_frame(self._prealloc_frame, bboxes=[], positions=[], all_dets=[], cup_assign={})
                    self._frame_count += 1
                present = {0: False, 1: False, 2: False, 3: False}
                self._last_result = self._transform_result(self._vote_presence(present))
                return dict(self._last_result)

            # ROI overlap via integral mask
            if self._roi_integral is not None:
                H, W = self._roi_mask.shape[:2]
                x1 = boxes_xyxy[:, 0].to(torch.int32).clamp(0, W - 1)
                y1 = boxes_xyxy[:, 1].to(torch.int32).clamp(0, H - 1)
                x2 = boxes_xyxy[:, 2].to(torch.int32).clamp(0, W - 1)
                y2 = boxes_xyxy[:, 3].to(torch.int32).clamp(0, H - 1)

                S = torch.as_tensor(self._roi_integral, dtype=torch.int64)  # (H+1, W+1)
                roi_pix = S[y2 + 1, x2 + 1] - S[y1, x2 + 1] - S[y2 + 1, x1] + S[y1, x1]
                box_area = (x2 - x1 + 1) * (y2 - y1 + 1)
                overlap = roi_pix.to(torch.float32) / torch.clamp(box_area.to(torch.float32), min=1.0)
                keep = overlap >= self.config.roi_overlap_threshold
                boxes_xyxy = boxes_xyxy[keep]

            # Assign to nearest of 4 cups
            present = {0: False, 1: False, 2: False, 3: False}
            cup_assign = {}
            if self._cups_resized is not None and len(self._cups_resized) >= 4 and boxes_xyxy.numel() > 0:
                cx = 0.5 * (boxes_xyxy[:, 0] + boxes_xyxy[:, 2])
                cy = 0.5 * (boxes_xyxy[:, 1] + boxes_xyxy[:, 3])
                centers = torch.stack([cx, cy], dim=1).cpu().numpy()     # (N,2)
                cups = self._cups_resized.astype(np.float32)             # (4,2)
                d2 = ((centers[:, None, :] - cups[None, :, :]) ** 2).sum(axis=2)
                assign = np.argmin(d2, axis=1)
                for k in range(4):
                    present[k] = bool(np.any(assign == k))
                # Create cup assignment mapping for debug
                for i, cup_id in enumerate(assign):
                    cup_assign[i] = int(cup_id)

            # Save debug frame
            if self.config.debug_mode or self.config.save_frames:
                # Convert tensors to lists for debug visualization
                bboxes_list = []
                all_dets_list = []
                if boxes_xyxy.numel() > 0:
                    bboxes_list = [(int(x1), int(y1), int(x2), int(y2)) for x1, y1, x2, y2 in boxes_xyxy.cpu().numpy()]
                if detections.xyxy.size > 0:
                    all_dets_list = [(int(x1), int(y1), int(x2), int(y2)) for x1, y1, x2, y2 in detections.xyxy]
                
                positions_list = []
                if self._cups_resized is not None and len(self._cups_resized) >= 4:
                    positions_list = [(int(x), int(y)) for x, y in self._cups_resized]
                
                self._save_debug_frame(self._prealloc_frame, bboxes_list, positions_list, all_dets_list, cup_assign)
                self._frame_count += 1

            self._last_result = self._transform_result(self._vote_presence(present))
            return dict(self._last_result)

        except Exception:
            log.exception("detect() failure")
            return {"error": "Internal error during detect()."}

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
    
    def _transform_result(self, result: dict):
        """
        Transform 0-indexed result to 1-indexed and reverse the values.
        Example: {0:True, 1:False, 2:False, 3:False} -> {1:False, 2:False, 3:False, 4:True}
        """
        # Reverse the values (position 0 -> 3, 1 -> 2, 2 -> 1, 3 -> 0)
        reversed_values = [result.get(3-i, False) for i in range(4)]
        # Convert to 1-indexed dict
        return {i+1: reversed_values[i] for i in range(4)}

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