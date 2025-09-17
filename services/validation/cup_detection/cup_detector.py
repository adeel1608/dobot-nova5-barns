"""
Production Cup Detection System
Simple, robust cup presence detector for production use.
"""

import cv2
import numpy as np
from ultralytics import YOLO
import time
import threading
import logging
import os
import requests
from typing import Dict, List, Tuple, Optional, Union

# ---------- Configuration ----------

class Config:
    """Simple configuration class"""
    def __init__(self):
        self.model_path = "models/yolo11l.pt"
        self.process_scale = 0.5
        self.device = "cuda:0"
        self.conf = 0.25
        self.imgsz = 416
        self.reconnect_retries = 999999
        self.reconnect_delay = 0.5
        
        # Detection settings
        self.frames = 5
        self.threshold = 0.5
        
        # Debug settings
        self.debug_mode = False
        self.save_frames = False
        self.debug_folder = "debug_frames"
        
        # Default ROI and positions
        self.default_roi = np.array([[781, 713], [2435, 1060], [2346, 1450], [445, 884]], dtype=np.int32)
        self.default_positions = [(1642, 1148), (1834, 1046), (1976, 984), (2122, 908)]

# ---------- Helper Functions ----------

def is_point_in_polygon(point: Tuple[float, float], polygon: np.ndarray) -> bool:
    """Check if a point is inside a polygon"""
    point = tuple(map(float, point))
    return cv2.pointPolygonTest(polygon, point, False) >= 0

def check_circle_bbox_overlap(circle_center: Tuple[int, int], circle_radius: int, bbox: Tuple[int, int, int, int]) -> bool:
    """Check if a circle overlaps with a bounding box"""
    cx, cy = circle_center
    x1, y1, x2, y2 = bbox
    closest_x = max(x1, min(cx, x2))
    closest_y = max(y1, min(cy, y2))
    dx = cx - closest_x
    dy = cy - closest_y
    return (dx * dx + dy * dy) < (circle_radius * circle_radius)

# ---------- HTTP Snapshot Reader ----------

class CameraError(Exception):
    pass

class HttpSnapshotReader:
    """HTTP snapshot reader - more efficient than RTSP stream"""
    
    def __init__(self, snapshot_url: str, config: Config):
        self.snapshot_url = snapshot_url
        self.config = config
        self._frame_count = 0
        
    def get_frame(self, timeout: float = 5.0):
        """Get a single frame from HTTP snapshot"""
        try:
            # Make HTTP request to get snapshot
            response = requests.get(
                self.snapshot_url, 
                timeout=timeout,
                auth=(self.snapshot_url.split('://')[1].split('@')[0].split(':')[0], 
                      self.snapshot_url.split('://')[1].split('@')[0].split(':')[1])
            )
            response.raise_for_status()
            
            # Convert response to OpenCV image
            image_array = np.frombuffer(response.content, dtype=np.uint8)
            frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
            
            if frame is None:
                raise CameraError("Failed to decode image from HTTP response")
            
            self._frame_count += 1
            return frame, self._frame_count
            
        except requests.exceptions.RequestException as e:
            raise CameraError(f"HTTP request failed: {str(e)}")
        except Exception as e:
            raise CameraError(f"Frame capture failed: {str(e)}")
    
    def stop(self):
        """No cleanup needed for HTTP requests"""
        pass

# ---------- Main Detector ----------

class CupDetector:
    """Simple cup presence detector"""
    
    def __init__(self, config_file: str = "config.py"):
        # Load config from file
        self.config = self._load_config(config_file)
        self.logger = logging.getLogger("CupDetector")
        
        # Setup debug folder
        if self.config.debug_mode or self.config.save_frames:
            os.makedirs(self.config.debug_folder, exist_ok=True)
        
        # Load model
        self.model = YOLO(self.config.model_path)
        
        # Set ROI and positions from config
        self.roi_polygon = self.config.roi_polygon
        self.positions = self.config.positions
        
        # Start HTTP snapshot reader
        self.camera = HttpSnapshotReader(self.config.snapshot_url, self.config)
        self._frame_count = 0
    
    def _load_config(self, config_file: str):
        """Load configuration from config.py file"""
        import importlib.util
        import sys
        
        # Load config module
        spec = importlib.util.spec_from_file_location("config", config_file)
        config_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(config_module)
        
        # Create config object
        config_obj = Config()
        
        # Load settings from config file
        config_obj.model_path = getattr(config_module, 'MODEL_PATH', config_obj.model_path)
        config_obj.process_scale = getattr(config_module, 'PROCESS_SCALE', config_obj.process_scale)
        config_obj.device = getattr(config_module, 'DEVICE', config_obj.device)
        config_obj.conf = getattr(config_module, 'CONFIDENCE', config_obj.conf)
        config_obj.imgsz = getattr(config_module, 'IMG_SIZE', config_obj.imgsz)
        config_obj.reconnect_retries = getattr(config_module, 'RECONNECT_RETRIES', config_obj.reconnect_retries)
        config_obj.reconnect_delay = getattr(config_module, 'RECONNECT_DELAY', config_obj.reconnect_delay)
        
        # Debug settings
        config_obj.debug_mode = getattr(config_module, 'DEBUG_MODE', config_obj.debug_mode)
        config_obj.save_frames = getattr(config_module, 'SAVE_FRAMES', config_obj.save_frames)
        config_obj.debug_folder = getattr(config_module, 'DEBUG_FOLDER', config_obj.debug_folder)
        
        # Detection settings
        config_obj.frames = getattr(config_module, 'FRAMES', 5)
        config_obj.threshold = getattr(config_module, 'THRESHOLD', 0.5)
        
        # ROI and positions
        config_obj.roi_polygon = getattr(config_module, 'ROI_POLYGON', config_obj.default_roi)
        config_obj.positions = getattr(config_module, 'CUP_POSITIONS', config_obj.default_positions)
        
        # Snapshot URL
        config_obj.snapshot_url = getattr(config_module, 'SNAPSHOT_URL', "http://qltyss:QSS2030QSS@192.168.200.60/cgi-bin/snapshot.cgi")
        
        return config_obj
    
    def _auto_positions(self, scaled_roi: np.ndarray) -> List[Tuple[int, int]]:
        """Generate automatic positions"""
        roi_w = np.max(scaled_roi[:, 0]) - np.min(scaled_roi[:, 0])
        roi_center = np.mean(scaled_roi, axis=0).astype(int)
        left_x = int(np.min(scaled_roi[:, 0]) + roi_w * 0.15)
        right_x = int(np.max(scaled_roi[:, 0]) - roi_w * 0.15)
        spacing = (right_x - left_x) / 3.0
        return [(int(left_x + spacing * i), int(roi_center[1])) for i in range(4)]
    
    def _detection_radius(self, scaled_roi: np.ndarray) -> int:
        """Calculate detection radius"""
        roi_w = np.max(scaled_roi[:, 0]) - np.min(scaled_roi[:, 0])
        roi_h = np.max(scaled_roi[:, 1]) - np.min(scaled_roi[:, 1])
        return max(4, int(min(roi_w, roi_h) * 0.05))
    
    def _save_debug_frame(self, frame: np.ndarray, frame_type: str, bboxes: List = None, positions: List = None):
        """Save debug frame with annotations (overwrites same file)"""
        if not (self.config.debug_mode or self.config.save_frames):
            return
        
        debug_frame = frame.copy()
        
        # Draw ROI polygon
        cv2.polylines(debug_frame, [self.roi_polygon], True, (0, 255, 0), 2)
        cv2.putText(debug_frame, "ROI", (self.roi_polygon[0][0], self.roi_polygon[0][1]-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw detected bboxes
        if bboxes:
            for i, (x1, y1, x2, y2) in enumerate(bboxes):
                cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(debug_frame, f"Cup {i+1}", (x1, y1-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Draw cup positions
        if positions:
            for i, (x, y) in enumerate(positions):
                cv2.circle(debug_frame, (x, y), 8, (255, 0, 0), -1)
                cv2.putText(debug_frame, f"Pos {i+1}", (x+10, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # Add timestamp and frame info
        timestamp = time.strftime("%H:%M:%S")
        cv2.putText(debug_frame, f"Frame {self._frame_count} - {timestamp}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Save frame (overwrites same file)
        filename = f"{self.config.debug_folder}/debug_frame.jpg"
        cv2.imwrite(filename, debug_frame)

    def _detect_cups_in_roi(self, process_frame: np.ndarray, scaled_roi: np.ndarray):
        """Detect cups in ROI"""
        x_min, y_min = np.min(scaled_roi, axis=0)
        x_max, y_max = np.max(scaled_roi, axis=0)
        roi_crop = process_frame[y_min:y_max, x_min:x_max]
        
        results = self.model.predict(
            source=roi_crop,
            device=self.config.device,
            classes=[41, 45],
            conf=self.config.conf,
            imgsz=self.config.imgsz,
            verbose=False,
        )
        
        detected = []
        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                x1 += x_min; x2 += x_min
                y1 += y_min; y2 += y_min
                
                # Check if bbox is mostly inside ROI
                corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
                inside = sum(1 for c in corners if is_point_in_polygon(c, scaled_roi))
                if inside >= 2:
                    detected.append((x1, y1, x2, y2))
        
        return detected
    
    def detect(self) -> Union[Dict[int, bool], Dict[str, str]]:
        """Detect cup presence
        
        Returns:
            Success: {0: bool, 1: bool, 2: bool, 3: bool}
            Error: {"error": str}
        """
        try:
            vote_counts = [0, 0, 0, 0]
            
            for i in range(self.config.frames):
                # Get frame from HTTP snapshot
                frame, idx = self.camera.get_frame()
                self._frame_count += 1
                
                # Preprocess
                h, w = frame.shape[:2]
                ph, pw = int(h * self.config.process_scale), int(w * self.config.process_scale)
                proc = cv2.resize(frame, (pw, ph))
                scaled_roi = (self.roi_polygon * self.config.process_scale).astype(np.int32)
                
                # Get positions
                if self.positions is None:
                    pos_proc = self._auto_positions(scaled_roi)
                else:
                    pos_proc = [(int(x * self.config.process_scale), int(y * self.config.process_scale)) 
                               for (x, y) in self.positions]
                
                # Detect
                det_radius = self._detection_radius(scaled_roi)
                bboxes = self._detect_cups_in_roi(proc, scaled_roi)
                
                # Save debug frame if enabled
                if self.config.debug_mode or self.config.save_frames:
                    # Scale back to original frame size for debug
                    scale_back = 1.0 / self.config.process_scale
                    debug_roi = (self.roi_polygon).astype(np.int32)
                    debug_positions = [(int(x * scale_back), int(y * scale_back)) for (x, y) in pos_proc]
                    debug_bboxes = [(int(x1 * scale_back), int(y1 * scale_back), 
                                   int(x2 * scale_back), int(y2 * scale_back)) for (x1, y1, x2, y2) in bboxes]
                    
                    self._save_debug_frame(frame, "detection", debug_bboxes, debug_positions)
                
                for k, center in enumerate(pos_proc):
                    if any(check_circle_bbox_overlap(center, det_radius, bb) for bb in bboxes):
                        vote_counts[k] += 1
            
            # Return results
            result = {i: (vote_counts[i] / float(self.config.frames) >= self.config.threshold) for i in range(4)}
            return result
            
        except CameraError as e:
            return {"error": f"Camera error: {str(e)}"}
        except Exception as e:
            return {"error": f"Detection failed: {str(e)}"}
    
    def release(self):
        """Release resources"""
        if hasattr(self, 'camera'):
            self.camera.stop()
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()
