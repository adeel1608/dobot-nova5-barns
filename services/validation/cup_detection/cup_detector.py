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
from typing import Dict, List, Tuple, Union
from collections import deque

# ---------- Configuration ----------

class Config:
    """Simple configuration class"""
    def __init__(self):
        self.model_path = "models/yolo11l.pt"
        self.process_scale = 0.5
        self.device = "cpu"
        self.conf = 0.25
        self.imgsz = 416
        self.reconnect_retries = 999999
        self.reconnect_delay = 0.5
        
        # RTSP connection settings
        self.connection_timeout = 5.0      # Connection timeout in seconds
        self.frame_timeout = 3.0           # Frame read timeout in seconds
        self.retry_delay = 1.0             # Initial retry delay
        self.max_retry_delay = 10.0        # Maximum retry delay
        self.retry_backoff = 1.5           # Exponential backoff multiplier
        
        # Detection settings - OPTIMIZED FOR SPEED
        self.frames = 1        # Use only 1 frame for speed
        self.threshold = 0.3   # Lower threshold since we use fewer frames
        
        # Adaptive detection settings
        self.detection_history_size = 5   # Reduced history for faster processing
        self.position_boost_radius = 50   # Boost confidence for detections near known positions
        self.min_cup_size = 20           # Minimum cup size in pixels
        self.max_cup_size = 200          # Maximum cup size in pixels
        self.aspect_ratio_min = 0.5      # Minimum aspect ratio (height/width)
        self.aspect_ratio_max = 2.0      # Maximum aspect ratio (height/width)
        
        # Enhanced ROI detection settings
        self.roi_overlap_threshold = 0.3  # Minimum overlap ratio for ROI inclusion
        self.roi_center_weight = 0.7      # Weight for center point in ROI check
        
        # Transparent cup detection settings
        self.transparent_boost_factor = 1.2  # Boost factor for transparent cup detection
        self.edge_detection_enabled = True   # Enable edge-based detection for transparent cups
        self.contour_analysis_enabled = True # Enable contour analysis for transparent cups
        self.reflection_filter_enabled = True # Enable reflection filtering
        
        # Performance optimization settings
        self.frame_buffer_size = 3       # Keep only last 3 frames in buffer
        self.skip_frames = 0             # Skip frames for faster processing (0 = no skip)
        self.enable_frame_skipping = True # Enable frame skipping for speed
        
        # Debug settings
        self.debug_mode = False
        self.save_frames = False
        self.debug_folder = "debug_frames"
        self.max_debug_frames = 10
        self.cleanup_after_detection = True
        
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


# ---------- RTSP Stream Reader ----------

class CameraError(Exception):
    pass

class RTSPStreamReader:
    """RTSP stream reader with robust connection handling and frame buffering"""
    
    def __init__(self, rtsp_url: str, config: Config):
        self.rtsp_url = rtsp_url
        self.config = config
        self._frame_count = 0
        self.cap = None
        self._connection_retries = 0
        self._current_retry_delay = config.retry_delay
        self._last_successful_frame = time.time()
        self._frame_buffer = deque(maxlen=config.frame_buffer_size)
        self._frame_lock = threading.Lock()
        self._streaming_thread = None
        self._stop_streaming = False
        self._connect()
        self._start_streaming()
        
    def _connect(self):
        """Connect to RTSP stream with timeout and retry logic"""
        start_time = time.time()
        
        while time.time() - start_time < self.config.connection_timeout:
            try:
                # Release existing connection if any
                if self.cap is not None:
                    self.cap.release()
                
                # Create new connection
                self.cap = cv2.VideoCapture(self.rtsp_url)
                
                # Set connection properties for faster connection and lower latency
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimal buffer
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                
                # Test connection by trying to read a frame
                if self.cap.isOpened():
                    ret, test_frame = self.cap.read()
                    if ret and test_frame is not None:
                        # Connection successful
                        self._connection_retries = 0
                        self._current_retry_delay = self.config.retry_delay
                        self._last_successful_frame = time.time()
                        return
                
                # Connection failed, try again
                self.cap.release()
                self.cap = None
                
            except Exception as e:
                if self.cap is not None:
                    self.cap.release()
                    self.cap = None
            
            # Wait before retry
            time.sleep(0.1)
        
        # If we get here, connection failed
        raise CameraError(f"Failed to connect to RTSP stream: {self.rtsp_url} (timeout: {self.config.connection_timeout}s)")
    
    def _start_streaming(self):
        """Start background thread for continuous frame streaming"""
        self._stop_streaming = False
        self._streaming_thread = threading.Thread(target=self._stream_frames, daemon=True)
        self._streaming_thread.start()
    
    def _stream_frames(self):
        """Background thread to continuously read frames"""
        skip_counter = 0
        
        while not self._stop_streaming:
            try:
                # Check if connection is still valid
                if self.cap is None or not self.cap.isOpened():
                    self._reconnect_with_backoff()
                    continue
                
                # Try to read frame
                ret, frame = self.cap.read()
                
                if ret and frame is not None:
                    # Success - reset retry counters
                    self._connection_retries = 0
                    self._current_retry_delay = self.config.retry_delay
                    self._last_successful_frame = time.time()
                    self._frame_count += 1
                    
                    # Frame skipping for performance
                    if self.config.enable_frame_skipping and self.config.skip_frames > 0:
                        skip_counter += 1
                        if skip_counter <= self.config.skip_frames:
                            continue
                        skip_counter = 0
                    
                    # Add frame to buffer
                    with self._frame_lock:
                        self._frame_buffer.append((frame.copy(), self._frame_count))
                    
                else:
                    # Frame read failed, try to reconnect
                    self._reconnect_with_backoff()
                    
            except Exception as e:
                # Any error, try to reconnect
                try:
                    self._reconnect_with_backoff()
                except CameraError:
                    # If reconnection fails, wait a bit before trying again
                    time.sleep(1.0)
        
    def _reconnect_with_backoff(self):
        """Reconnect with exponential backoff"""
        self._connection_retries += 1
        
        # Calculate delay with exponential backoff
        delay = min(self._current_retry_delay, self.config.max_retry_delay)
        time.sleep(delay)
        
        # Increase delay for next retry
        self._current_retry_delay = min(
            self._current_retry_delay * self.config.retry_backoff,
            self.config.max_retry_delay
        )
        
        # Attempt reconnection
        try:
            self._connect()
        except CameraError:
            # If still failing, try again with longer delay
            if self._connection_retries < 5:  # Limit retries to prevent infinite loop
                self._reconnect_with_backoff()
            else:
                raise CameraError(f"Failed to reconnect after {self._connection_retries} attempts")
        
    def get_latest_frame(self, timeout: float = None):
        """Get the latest frame from buffer (non-blocking)"""
        if timeout is None:
            timeout = 1.0  # Short timeout for speed
            
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            with self._frame_lock:
                if self._frame_buffer:
                    frame, frame_id = self._frame_buffer[-1]  # Get latest frame
                    return frame, frame_id
            
            time.sleep(0.01)  # Short sleep to avoid busy waiting
        
        # Timeout reached
        raise CameraError(f"No frame available after {timeout}s")
    
    def get_frame(self, timeout: float = None):
        """Get a single frame from RTSP stream with robust error handling (legacy method)"""
        return self.get_latest_frame(timeout)
    
    def stop(self):
        """Release RTSP connection"""
        self._stop_streaming = True
        if self._streaming_thread and self._streaming_thread.is_alive():
            self._streaming_thread.join(timeout=2.0)
        
        if self.cap is not None:
            try:
                self.cap.release()
            except:
                pass
            self.cap = None
    
    def is_connected(self):
        """Check if connection is active"""
        return self.cap is not None and self.cap.isOpened()
    
    def get_connection_info(self):
        """Get connection status information"""
        with self._frame_lock:
            buffer_size = len(self._frame_buffer)
        
        return {
            'connected': self.is_connected(),
            'frame_count': self._frame_count,
            'retry_count': self._connection_retries,
            'last_success': self._last_successful_frame,
            'retry_delay': self._current_retry_delay,
            'buffer_size': buffer_size
        }

# ---------- Main Detector ----------

class CupDetector:
    """Simple cup presence detector - OPTIMIZED FOR SPEED"""
    
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
        
        # Start RTSP stream reader
        self.camera = RTSPStreamReader(self.config.rtsp_url, self.config)
        self._frame_count = 0
        
        # Initialize detection history for adaptive detection
        self.detection_history = [[] for _ in range(4)]  # History for each cup position
    
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
        config_obj.max_debug_frames = getattr(config_module, 'MAX_DEBUG_FRAMES', config_obj.max_debug_frames)
        config_obj.cleanup_after_detection = getattr(config_module, 'CLEANUP_AFTER_DETECTION', config_obj.cleanup_after_detection)
        
        # Detection settings
        config_obj.frames = getattr(config_module, 'FRAMES', 1)  # Default to 1 for speed
        config_obj.threshold = getattr(config_module, 'THRESHOLD', 0.3)
        
        # Adaptive detection settings
        config_obj.detection_history_size = getattr(config_module, 'DETECTION_HISTORY_SIZE', config_obj.detection_history_size)
        config_obj.position_boost_radius = getattr(config_module, 'POSITION_BOOST_RADIUS', config_obj.position_boost_radius)
        config_obj.min_cup_size = getattr(config_module, 'MIN_CUP_SIZE', config_obj.min_cup_size)
        config_obj.max_cup_size = getattr(config_module, 'MAX_CUP_SIZE', config_obj.max_cup_size)
        config_obj.aspect_ratio_min = getattr(config_module, 'ASPECT_RATIO_MIN', config_obj.aspect_ratio_min)
        config_obj.aspect_ratio_max = getattr(config_module, 'ASPECT_RATIO_MAX', config_obj.aspect_ratio_max)
        
        # Enhanced ROI detection settings
        config_obj.roi_overlap_threshold = getattr(config_module, 'ROI_OVERLAP_THRESHOLD', config_obj.roi_overlap_threshold)
        config_obj.roi_center_weight = getattr(config_module, 'ROI_CENTER_WEIGHT', config_obj.roi_center_weight)
        
        # Transparent cup detection settings
        config_obj.transparent_boost_factor = getattr(config_module, 'TRANSPARENT_BOOST_FACTOR', config_obj.transparent_boost_factor)
        config_obj.edge_detection_enabled = getattr(config_module, 'EDGE_DETECTION_ENABLED', config_obj.edge_detection_enabled)
        config_obj.contour_analysis_enabled = getattr(config_module, 'CONTOUR_ANALYSIS_ENABLED', config_obj.contour_analysis_enabled)
        config_obj.reflection_filter_enabled = getattr(config_module, 'REFLECTION_FILTER_ENABLED', config_obj.reflection_filter_enabled)
        
        # Performance optimization settings
        config_obj.frame_buffer_size = getattr(config_module, 'FRAME_BUFFER_SIZE', config_obj.frame_buffer_size)
        config_obj.skip_frames = getattr(config_module, 'SKIP_FRAMES', config_obj.skip_frames)
        config_obj.enable_frame_skipping = getattr(config_module, 'ENABLE_FRAME_SKIPPING', config_obj.enable_frame_skipping)
        
        # ROI and positions
        config_obj.roi_polygon = getattr(config_module, 'ROI_POLYGON', config_obj.default_roi)
        config_obj.positions = getattr(config_module, 'CUP_POSITIONS', config_obj.default_positions)
        
        # RTSP URL
        config_obj.rtsp_url = getattr(config_module, 'RTSP_URL', "rtsp://admin:123456@192.168.200.60:554/stream1")
        
        # RTSP connection settings
        config_obj.connection_timeout = getattr(config_module, 'CONNECTION_TIMEOUT', config_obj.connection_timeout)
        config_obj.frame_timeout = getattr(config_module, 'FRAME_TIMEOUT', config_obj.frame_timeout)
        config_obj.retry_delay = getattr(config_module, 'RETRY_DELAY', config_obj.retry_delay)
        config_obj.max_retry_delay = getattr(config_module, 'MAX_RETRY_DELAY', config_obj.max_retry_delay)
        config_obj.retry_backoff = getattr(config_module, 'RETRY_BACKOFF', config_obj.retry_backoff)
        
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
    
    def _is_valid_cup_size(self, bbox: Tuple[int, int, int, int]) -> bool:
        """Check if detected cup has valid size and aspect ratio"""
        x1, y1, x2, y2 = bbox
        width = x2 - x1
        height = y2 - y1
        
        # Check size constraints
        if width < self.config.min_cup_size or height < self.config.min_cup_size:
            return False
        if width > self.config.max_cup_size or height > self.config.max_cup_size:
            return False
        
        # Check aspect ratio
        aspect_ratio = height / width if width > 0 else 0
        if aspect_ratio < self.config.aspect_ratio_min or aspect_ratio > self.config.aspect_ratio_max:
            return False
        
        return True
    
    def _calculate_bbox_roi_overlap(self, bbox: Tuple[int, int, int, int], roi_polygon: np.ndarray) -> float:
        """Calculate the overlap ratio between bounding box and ROI polygon"""
        x1, y1, x2, y2 = bbox
        
        # Ensure bbox coordinates are valid
        if x2 <= x1 or y2 <= y1:
            return 0.0
        
        # Create a larger mask to handle ROI that extends outside bbox
        # Add padding to ensure we capture the full ROI
        padding = 50
        mask_x1 = max(0, x1 - padding)
        mask_y1 = max(0, y1 - padding)
        mask_x2 = x2 + padding
        mask_y2 = y2 + padding
        
        # Create mask for the extended area
        roi_mask = np.zeros((mask_y2 - mask_y1, mask_x2 - mask_x1), dtype=np.uint8)
        
        # Adjust ROI coordinates relative to the extended mask
        adjusted_roi = roi_polygon - np.array([mask_x1, mask_y1])
        
        # Fill the ROI polygon in the mask
        cv2.fillPoly(roi_mask, [adjusted_roi], 255)
        
        # Extract the bbox area from the mask
        bbox_in_mask_x1 = x1 - mask_x1
        bbox_in_mask_y1 = y1 - mask_y1
        bbox_in_mask_x2 = x2 - mask_x1
        bbox_in_mask_y2 = y2 - mask_y1
        
        # Ensure coordinates are within mask bounds
        bbox_in_mask_x1 = max(0, bbox_in_mask_x1)
        bbox_in_mask_y1 = max(0, bbox_in_mask_y1)
        bbox_in_mask_x2 = min(roi_mask.shape[1], bbox_in_mask_x2)
        bbox_in_mask_y2 = min(roi_mask.shape[0], bbox_in_mask_y2)
        
        # Extract bbox region from mask
        bbox_mask = roi_mask[bbox_in_mask_y1:bbox_in_mask_y2, bbox_in_mask_x1:bbox_in_mask_x2]
        
        # Calculate overlap ratio
        total_pixels = (x2 - x1) * (y2 - y1)
        roi_pixels = np.sum(bbox_mask > 0)
        
        return roi_pixels / total_pixels if total_pixels > 0 else 0.0
    
    def _is_bbox_in_roi_enhanced(self, bbox: Tuple[int, int, int, int], roi_polygon: np.ndarray) -> bool:
        """Enhanced ROI check using multiple criteria"""
        x1, y1, x2, y2 = bbox
        
        # Method 1: Check bottom corners (original method)
        bottom_left = (x1, y2)
        bottom_right = (x2, y2)
        bottom_left_inside = is_point_in_polygon(bottom_left, roi_polygon)
        bottom_right_inside = is_point_in_polygon(bottom_right, roi_polygon)
        
        # Method 2: Check center point
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        center_inside = is_point_in_polygon((center_x, center_y), roi_polygon)
        
        # Method 3: Check all four corners
        top_left = (x1, y1)
        top_right = (x2, y1)
        top_left_inside = is_point_in_polygon(top_left, roi_polygon)
        top_right_inside = is_point_in_polygon(top_right, roi_polygon)
        
        # Method 4: Check overlap ratio
        overlap_ratio = self._calculate_bbox_roi_overlap(bbox, roi_polygon)
        
        # Count how many corners are inside
        corners_inside = sum([bottom_left_inside, bottom_right_inside, top_left_inside, top_right_inside])
        
        # More lenient criteria for ROI inclusion
        return (corners_inside >= 1 or  # At least one corner inside
                center_inside or        # Center point inside
                overlap_ratio > 0.1)    # At least 10% overlap
    
    def _enhance_transparent_cup_detection(self, frame: np.ndarray, bbox: Tuple[int, int, int, int]) -> float:
        """Enhance detection for transparent cups using edge detection and contour analysis"""
        x1, y1, x2, y2 = bbox
        
        # Extract ROI from frame
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return 1.0
        
        enhancement_factor = 1.0
        
        if self.config.edge_detection_enabled:
            # Edge detection for transparent cups
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)
            edge_density = np.sum(edges > 0) / edges.size
            
            # Transparent cups typically have more edge density
            if edge_density > 0.1:  # High edge density indicates transparent cup
                enhancement_factor *= self.config.transparent_boost_factor
        
        if self.config.contour_analysis_enabled:
            # Contour analysis for cup-like shapes
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Find the largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                bbox_area = (x2 - x1) * (y2 - y1)
                
                # Check if contour fills a reasonable portion of the bbox
                if area / bbox_area > 0.3:
                    # Check for circular/oval shape (typical for cups)
                    perimeter = cv2.arcLength(largest_contour, True)
                    if perimeter > 0:
                        circularity = 4 * np.pi * area / (perimeter * perimeter)
                        if 0.3 < circularity < 0.9:  # Cup-like circularity
                            enhancement_factor *= self.config.transparent_boost_factor
        
        if self.config.reflection_filter_enabled:
            # Filter out pure reflections (very bright, low contrast areas)
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            mean_brightness = np.mean(gray)
            std_brightness = np.std(gray)
            
            # Pure reflections are very bright with low contrast
            if mean_brightness > 200 and std_brightness < 30:
                enhancement_factor *= 0.5  # Reduce confidence for pure reflections
        
        return enhancement_factor
    
    def _boost_confidence_near_positions(self, bbox: Tuple[int, int, int, int], confidence: float) -> float:
        """Boost confidence for detections near known cup positions"""
        x1, y1, x2, y2 = bbox
        bbox_center = ((x1 + x2) // 2, (y1 + y2) // 2)
        
        # Check distance to each known position
        for pos_x, pos_y in self.positions:
            distance = np.sqrt((bbox_center[0] - pos_x)**2 + (bbox_center[1] - pos_y)**2)
            if distance <= self.config.position_boost_radius:
                # Boost confidence based on proximity
                boost_factor = 1.0 + (1.0 - distance / self.config.position_boost_radius) * 0.3
                return min(1.0, confidence * boost_factor)
        
        return confidence
    
    def _update_detection_history(self, position_results: List[bool]):
        """Update detection history for adaptive thresholding"""
        for i, detected in enumerate(position_results):
            self.detection_history[i].append(detected)
            # Keep only recent history
            if len(self.detection_history[i]) > self.config.detection_history_size:
                self.detection_history[i].pop(0)
    
    def _get_adaptive_threshold(self, position_idx: int) -> float:
        """Get adaptive threshold based on detection history"""
        history = self.detection_history[position_idx]
        if len(history) < 3:
            return self.config.threshold
        
        # Calculate recent detection rate
        recent_detections = sum(history[-5:])  # Last 5 detections
        recent_rate = recent_detections / min(5, len(history))
        
        # Adjust threshold based on recent detection rate
        if recent_rate > 0.7:  # Cup has been consistently detected
            return max(0.2, self.config.threshold - 0.1)  # Lower threshold
        elif recent_rate < 0.3:  # Cup has been consistently absent
            return min(0.8, self.config.threshold + 0.1)  # Higher threshold
        else:
            return self.config.threshold  # Use default threshold
    
    def _cleanup_old_debug_frames(self):
        """Remove old debug frames keeping only the most recent ones"""
        if not self.config.cleanup_after_detection:
            return
        
        try:
            debug_dir = self.config.debug_folder
            if not os.path.exists(debug_dir):
                return
            
            # Get all debug frame files
            frame_files = []
            for filename in os.listdir(debug_dir):
                if filename.startswith('debug_frame_') and filename.endswith('.jpg'):
                    filepath = os.path.join(debug_dir, filename)
                    if os.path.isfile(filepath):
                        # Get file modification time
                        mtime = os.path.getmtime(filepath)
                        frame_files.append((mtime, filepath, filename))
            
            if len(frame_files) <= self.config.max_debug_frames:
                return
            
            # Sort by modification time descending (newest first)
            frame_files.sort(reverse=True)
            
            # Keep only the most recent max_debug_frames files
            files_to_delete = frame_files[self.config.max_debug_frames:]
            
            # Delete old frames
            deleted_count = 0
            for mtime, filepath, filename in files_to_delete:
                try:
                    os.remove(filepath)
                    deleted_count += 1
                except Exception as e:
                    self.logger.error(f"Failed to delete {filename}: {e}")
            
            if deleted_count > 0:
                self.logger.debug(f"Cleaned up {deleted_count} old debug frames, kept {self.config.max_debug_frames} most recent frames")
                
        except Exception as e:
            self.logger.error(f"Error during debug frame cleanup: {e}")
    
    def _save_debug_frame(self, frame: np.ndarray, frame_type: str, bboxes: List = None, positions: List = None, all_detections: List = None):
        """Save debug frame with annotations"""
        if not (self.config.debug_mode or self.config.save_frames):
            return
        
        debug_frame = frame.copy()
        
        # Draw ROI polygon
        cv2.polylines(debug_frame, [self.roi_polygon], True, (0, 255, 0), 2)
        cv2.putText(debug_frame, "ROI", (self.roi_polygon[0][0], self.roi_polygon[0][1]-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw ALL detections (including those outside ROI) in light red
        if all_detections:
            for i, (x1, y1, x2, y2) in enumerate(all_detections):
                cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (0, 100, 255), 1)  # Light red
                cv2.putText(debug_frame, f"All {i+1}", (x1, y1-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 1)
        
        # Draw filtered detections (ROI + valid) in bright red
        if bboxes:
            for i, (x1, y1, x2, y2) in enumerate(bboxes):
                cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Bright red
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
        
        # Add detection count info
        all_count = len(all_detections) if all_detections else 0
        filtered_count = len(bboxes) if bboxes else 0
        cv2.putText(debug_frame, f"All: {all_count} | Filtered: {filtered_count}", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Add legend
        cv2.putText(debug_frame, "Green: ROI | Blue: Positions | Light Red: All Detections | Bright Red: Filtered", 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Save frame with timestamp to prevent overwriting and enable cleanup
        timestamp = int(time.time() * 1000)
        filename = f"{self.config.debug_folder}/debug_frame_{timestamp}.jpg"
        cv2.imwrite(filename, debug_frame)

    def _detect_cups_in_roi(self, process_frame: np.ndarray, scaled_roi: np.ndarray):
        """Detect cups in full frame, then filter by ROI with enhanced detection"""
        # Run YOLO on the full frame to catch cups that extend outside ROI
        results = self.model.predict(
            source=process_frame,
            device=self.config.device,
            classes=[41, 45],
            conf=self.config.conf * 0.5,  # Even lower initial threshold for more candidates
            imgsz=self.config.imgsz,
            verbose=False,
        )
        
        all_detections = []
        detected = []
        
        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                confidence = float(box.conf[0].cpu().numpy())
                
                # Add to all detections list (for debugging)
                all_detections.append((x1, y1, x2, y2))
                
                # Enhanced ROI check
                if self._is_bbox_in_roi_enhanced((x1, y1, x2, y2), scaled_roi):
                    # Apply size and aspect ratio filtering
                    if self._is_valid_cup_size((x1, y1, x2, y2)):
                        # Enhance detection for transparent cups
                        transparent_enhancement = self._enhance_transparent_cup_detection(process_frame, (x1, y1, x2, y2))
                        enhanced_confidence = confidence * transparent_enhancement
                        
                        # Boost confidence for detections near known positions
                        boosted_confidence = self._boost_confidence_near_positions((x1, y1, x2, y2), enhanced_confidence)
                        
                        # Only keep if boosted confidence meets threshold
                        if boosted_confidence >= self.config.conf:
                            detected.append((x1, y1, x2, y2))
        
        return detected, all_detections
    
    def detect(self) -> Union[Dict[int, bool], Dict[str, str]]:
        """Detect cup presence - OPTIMIZED FOR SPEED
        
        Returns:
            Success: {0: bool, 1: bool, 2: bool, 3: bool}
            Error: {"error": str}
        """
        try:
            # Use only 1 frame for speed (configurable)
            vote_counts = [0, 0, 0, 0]
            
            for i in range(self.config.frames):
                # Get latest frame from buffer (non-blocking)
                frame, idx = self.camera.get_latest_frame(timeout=0.5)  # Short timeout
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
                bboxes, all_detections = self._detect_cups_in_roi(proc, scaled_roi)
                
                # Save debug frame if enabled (save for each frame to show live updates)
                if self.config.debug_mode or self.config.save_frames:
                    # Scale back to original frame size for debug
                    scale_back = 1.0 / self.config.process_scale
                    debug_roi = (self.roi_polygon).astype(np.int32)
                    debug_positions = [(int(x * scale_back), int(y * scale_back)) for (x, y) in pos_proc]
                    debug_bboxes = [(int(x1 * scale_back), int(y1 * scale_back), 
                                   int(x2 * scale_back), int(y2 * scale_back)) for (x1, y1, x2, y2) in bboxes]
                    debug_all_detections = [(int(x1 * scale_back), int(y1 * scale_back), 
                                           int(x2 * scale_back), int(y2 * scale_back)) for (x1, y1, x2, y2) in all_detections]
                    
                    self._save_debug_frame(frame, "detection", debug_bboxes, debug_positions, debug_all_detections)
                
                for k, center in enumerate(pos_proc):
                    if any(check_circle_bbox_overlap(center, det_radius, bb) for bb in bboxes):
                        vote_counts[k] += 1
            
            # Apply adaptive thresholds and return results
            result = {}
            for i in range(4):
                detection_rate = vote_counts[i] / float(self.config.frames)
                adaptive_threshold = self._get_adaptive_threshold(i)
                result[i] = detection_rate >= adaptive_threshold
            
            # Update detection history for next cycle
            position_results = [result[i] for i in range(4)]
            self._update_detection_history(position_results)
            
            # Cleanup old debug frames to save disk space
            if self.config.debug_mode or self.config.save_frames:
                self._cleanup_old_debug_frames()
            
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