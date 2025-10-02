"""
Production-Ready Coffee Detection System
Optimized for 10-minute interval calls with proper resource management
"""

from __future__ import annotations
import logging
import os
import threading
import time
import json
import gc
from typing import Optional, Tuple, Dict, Any
from contextlib import contextmanager
from dataclasses import dataclass
import cv2
import numpy as np
import requests
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry

# Configure logging - errors only to file
logging.basicConfig(
    level=logging.ERROR,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('coffee_detection.log')
    ]
)
logger = logging.getLogger(__name__)

@dataclass
class DetectionConfig:
    """Configuration for coffee detection system"""
    snapshot_url: str = "http://qltyss:QSS2030QSS@192.168.200.88/cgi-bin/snapshot.cgi"
    frame_width: int = 1920
    frame_height: int = 1080
    frames_to_sample: int = 5
    coffee_threshold: float = 5.0
    timeout: float = 1.0
    connect_timeout: float = 0.5
    read_timeout: float = 1.0
    target_fps: float = 5.0
    enable_debug: bool = True
    debug_frame_dir: str = "debug_frames_coffee"
    max_retries: int = 3
    retry_backoff: float = 0.1
    roi_points: list = None

class CameraError(Exception):
    """Custom exception for camera-related errors"""
    pass

class ProductionSnapshotReader:
    """Production-ready snapshot reader with proper resource management"""
    
    def __init__(self, config: DetectionConfig):
        self.config = config
        self._session: Optional[requests.Session] = None
        self._lock = threading.Lock()
        self._is_initialized = False
        
    def _create_session(self) -> requests.Session:
        """Create a configured requests session with retry strategy"""
        session = requests.Session()
        
        # Configure retry strategy
        retry_strategy = Retry(
            total=self.config.max_retries,
            backoff_factor=self.config.retry_backoff,
            status_forcelist=[429, 500, 502, 503, 504],
        )
        
        adapter = HTTPAdapter(max_retries=retry_strategy)
        session.mount("http://", adapter)
        session.mount("https://", adapter)
        
        return session
    
    def _get_or_create_session(self) -> requests.Session:
        """Get existing session or create new one if needed"""
        if self._session is None:
            self._session = self._create_session()
        return self._session
    
    def _recreate_session(self):
        """Recreate session (called on failure)"""
        if self._session:
            try:
                self._session.close()
            except:
                pass
            self._session = None
    
    @contextmanager
    def get_frame(self):
        """Context manager for getting a single frame with session reuse"""
        try:
            session = self._get_or_create_session()
            
            response = session.get(
                self.config.snapshot_url,
                timeout=(self.config.connect_timeout, self.config.read_timeout),
                stream=True
            )
            response.raise_for_status()
            
            # Read image data
            img_data = response.content
            img_array = np.frombuffer(img_data, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            
            if frame is None:
                raise CameraError("Failed to decode image from camera")
                
            yield frame
            
        except requests.exceptions.RequestException as e:
            logger.error(f"Camera request failed: {e}")
            self._recreate_session()  # Recreate session on failure
            raise CameraError(f"Camera request failed: {e}")
        except Exception as e:
            logger.error(f"Unexpected error getting frame: {e}")
            self._recreate_session()  # Recreate session on failure
            raise CameraError(f"Frame capture error: {e}")

class ProductionCoffeeDetector:
    """Production-ready coffee detector with optimized performance"""
    
    def __init__(self, config: DetectionConfig = None):
        self.config = config or DetectionConfig()
        self.reader = ProductionSnapshotReader(self.config)
        
        # Initialize detection parameters
        # Use ROI points from config, fallback to default if not provided
        if self.config.roi_points:
            self.original_roi_points = [tuple(point) for point in self.config.roi_points]
        else:
            # Default ROI points (fallback)
            self.original_roi_points = [
                (121, 481), (953, 411), (1873, 420), (1867, 712),
                (1825, 998), (1009, 1064), (188, 992), (152, 742)
            ]
        self.coffee_lower = np.array([0, 30, 0], np.uint8)
        self.coffee_upper = np.array([25, 255, 150], np.uint8)
        self.kernel = np.ones((5, 5), np.uint8)
        
        # Cached resources (initialized lazily)
        self._roi_mask: Optional[np.ndarray] = None
        self._roi_polygon: Optional[np.ndarray] = None
        self._frame_shape: Optional[Tuple[int, int, int]] = None
        self._initialization_lock = threading.Lock()
        
        # Setup debug directory if enabled
        if self.config.enable_debug:
            os.makedirs(self.config.debug_frame_dir, exist_ok=True)
    
    def _ensure_initialized(self):
        """Lazy initialization of masks and polygons"""
        if self._roi_mask is not None:
            return
            
        with self._initialization_lock:
            if self._roi_mask is not None:  # Double-check after acquiring lock
                return
                
            try:
                with self.reader.get_frame() as frame:
                    # Resize frame to target dimensions
                    resized_frame = cv2.resize(
                        frame, 
                        (self.config.frame_width, self.config.frame_height),
                        interpolation=cv2.INTER_LINEAR
                    )
                    
                    self._frame_shape = resized_frame.shape
                    self._roi_polygon = np.array(self.original_roi_points, dtype=np.int32)
                    
                    # Create ROI mask
                    self._roi_mask = np.zeros(resized_frame.shape[:2], np.uint8)
                    cv2.fillPoly(self._roi_mask, [self._roi_polygon], 255)
                    
                    # Save ROI mask for debugging if enabled
                    if self.config.enable_debug:
                        cv2.imwrite(
                            os.path.join(self.config.debug_frame_dir, "roi_mask.png"),
                            self._roi_mask
                        )
                    
                    
            except Exception as e:
                logger.error(f"Failed to initialize ROI masks: {e}")
                raise CameraError(f"Initialization failed: {e}")
    
    def _resize_frame(self, frame: np.ndarray) -> np.ndarray:
        """Resize frame to target dimensions"""
        return cv2.resize(
            frame, 
            (self.config.frame_width, self.config.frame_height),
            interpolation=cv2.INTER_LINEAR
        )
    
    def _detect_coffee_single_frame(self, frame: np.ndarray, frame_idx: int) -> int:
        """Detect coffee in a single frame and return percentage"""
        try:
            # Convert to HSV for better color detection
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Create coffee mask
            coffee_mask = cv2.inRange(hsv, self.coffee_lower, self.coffee_upper)
            
            # Apply morphological operations to clean up the mask
            coffee_mask = cv2.morphologyEx(coffee_mask, cv2.MORPH_CLOSE, self.kernel)
            coffee_mask = cv2.morphologyEx(coffee_mask, cv2.MORPH_OPEN, self.kernel)
            
            # Apply ROI mask
            final_mask = cv2.bitwise_and(coffee_mask, self._roi_mask)
            
            # Save debug frames if enabled
            if self.config.enable_debug:
                self._save_debug_frames(frame, coffee_mask, final_mask, frame_idx)
                logger.debug("frame saved")
            logger.debug("frame saved2")
            # cv2.imshow("frame", frame)
            # cv2.waitKey(0)
            # show the frame with imshow
            

            # Calculate coffee percentage
            roi_pixels = int(np.count_nonzero(self._roi_mask))
            coffee_pixels = int(np.count_nonzero(final_mask)) if roi_pixels > 0 else 0
            
            percentage = int((coffee_pixels / roi_pixels) * 100) if roi_pixels > 0 else 0
            
            return percentage
            
        except Exception as e:
            logger.error(f"Error processing frame {frame_idx}: {e}")
            return 0
    
    def _save_debug_frames(self, frame: np.ndarray, coffee_mask: np.ndarray, 
                          final_mask: np.ndarray, frame_idx: int):
        """Save debug frames for analysis"""
        try:
            print(f"🖼️ Saving debug frame {frame_idx} to: {self.config.debug_frame_dir}")
            print(f"📂 Directory exists: {os.path.exists(self.config.debug_frame_dir)}")
            # Draw ROI overlay
            vis = frame.copy()
            if self._roi_polygon is not None:
                cv2.polylines(vis, [self._roi_polygon], True, (0, 255, 255), 2)
                fill = np.zeros_like(vis)
                cv2.fillPoly(fill, [self._roi_polygon], (255, 0, 0))
                vis = cv2.addWeighted(vis, 1.0, fill, 0.2, 0)
            # Save frames with explicit error checking
            roi_path = os.path.join(self.config.debug_frame_dir, f"frame_{frame_idx:04d}_scaled_roi.jpg")
            mask_path = os.path.join(self.config.debug_frame_dir, f"frame_{frame_idx:04d}_coffee_mask.png")
            final_path = os.path.join(self.config.debug_frame_dir, f"frame_{frame_idx:04d}_final_mask.png")
            
            logger.debug(f"💾 Saving to: {roi_path}")
            success1 = cv2.imwrite(roi_path, vis)
            success2 = cv2.imwrite(mask_path, coffee_mask)
            success3 = cv2.imwrite(final_path, final_mask)
            
            logger.debug(f"✅ Save results: ROI={success1}, Mask={success2}, Final={success3}")
            # Save frames
            cv2.imwrite(
                os.path.join(self.config.debug_frame_dir, f"frame_{frame_idx:04d}_scaled_roi.jpg"),
                vis
            )
            cv2.imwrite(
                os.path.join(self.config.debug_frame_dir, f"frame_{frame_idx:04d}_coffee_mask.png"),
                coffee_mask
            )
            cv2.imwrite(
                os.path.join(self.config.debug_frame_dir, f"frame_{frame_idx:04d}_final_mask.png"),
                final_mask
            )
            logger.debug(f"Not Failed to save debug frames for frame")
            logger.debug(os.path.join(self.config.debug_frame_dir, f"frame_{frame_idx:04d}_final_mask.png"))
            
            # Create overlay
            overlay = cv2.addWeighted(
                frame, 0.8,
                cv2.applyColorMap(final_mask, cv2.COLORMAP_JET), 0.4, 0
            )
            cv2.imwrite(
                os.path.join(self.config.debug_frame_dir, f"frame_{frame_idx:04d}_overlay.jpg"),
                overlay
            )
            
        except Exception as e:
            logger.error(f"Failed to save debug frames for frame {frame_idx}: {e}")
    
    def detect_coffee(self) -> Dict[str, Any]:
        """
        Main detection method - optimized for production use
        Returns comprehensive detection results
        """
        start_time = time.time()
        detection_id = int(time.time() * 1000)  # Unique detection ID
        
        try:
            # Ensure masks are initialized
            self._ensure_initialized()
            
            percentages = []
            successful_frames = 0
            
            # Sample multiple frames for better accuracy
            for i in range(self.config.frames_to_sample):
                try:
                    with self.reader.get_frame() as frame:
                        resized_frame = self._resize_frame(frame)
                        percentage = self._detect_coffee_single_frame(resized_frame, i)
                        percentages.append(percentage)
                        successful_frames += 1
                        
                        # Small delay between frames to avoid overwhelming the camera
                        if i < self.config.frames_to_sample - 1:
                            time.sleep(0.1)
                            
                except CameraError as e:
                    logger.warning(f"Failed to capture frame {i}: {e}")
                    continue
                except Exception as e:
                    logger.error(f"Unexpected error processing frame {i}: {e}")
                    continue
            
            # Calculate results
            if not percentages:
                raise CameraError("No frames were successfully processed")
            
            avg_percentage = float(np.mean(percentages))
            coffee_present = avg_percentage > self.config.coffee_threshold
            
            detection_time = time.time() - start_time
            
            result = {
                "coffee": coffee_present,
                "percentage": round(avg_percentage, 2),
                "error": None
            }
            # print(result)
            
            return result
            
        except Exception as e:
            
            error_msg = f"Detection failed: {e}"
            logger.error(f"Detection failed (ID: {detection_id}): {e}")
            
            return {
                "coffee": False,
                "percentage": 0.0,
                "error": error_msg
            }
        finally:
            # Force garbage collection to free memory
            gc.collect()
    
    def health_check(self) -> Dict[str, Any]:
        """Perform a health check of the detection system"""
        try:
            start_time = time.time()
            
            # Test camera connectivity
            with self.reader.get_frame() as frame:
                if frame is None or frame.size == 0:
                    raise CameraError("Camera returned empty frame")
            
            # Test ROI initialization
            self._ensure_initialized()
            
            health_time = time.time() - start_time
            
            return {
                "status": "healthy",
                "camera_accessible": True,
                "roi_initialized": self._roi_mask is not None,
                "health_check_time": round(health_time, 3),
                "timestamp": time.time()
            }
            
        except Exception as e:
            logger.error(f"Health check failed: {e}")
            return {
                "status": "unhealthy",
                "camera_accessible": False,
                "roi_initialized": self._roi_mask is not None,
                "error": str(e),
                "timestamp": time.time()
            }
    
    def cleanup(self):
        """Clean up resources"""
        # Clean up reader session
        if hasattr(self.reader, '_session') and self.reader._session:
            try:
                self.reader._session.close()
                self.reader._session = None
            except:
                pass
        
        # Clear cached data
        self._roi_mask = None
        self._roi_polygon = None
        self._frame_shape = None
        
        # Force garbage collection
        gc.collect()

def load_config(config_path: str = "detection_config.json") -> DetectionConfig:
    """Load configuration from JSON file"""
    try:
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config_data = json.load(f)
            return DetectionConfig(**config_data)
        else:
            return DetectionConfig()
    except Exception as e:
        logger.error(f"Failed to load config: {e}, using defaults")
        return DetectionConfig()

def save_config(config: DetectionConfig, config_path: str = "detection_config.json"):
    """Save configuration to JSON file"""
    try:
        config_dict = {
            "snapshot_url": config.snapshot_url,
            "frame_width": config.frame_width,
            "frame_height": config.frame_height,
            "frames_to_sample": config.frames_to_sample,
            "coffee_threshold": config.coffee_threshold,
            "timeout": config.timeout,
            "connect_timeout": config.connect_timeout,
            "read_timeout": config.read_timeout,
            "target_fps": config.target_fps,
            "enable_debug": config.enable_debug,
            "debug_frame_dir": config.debug_frame_dir,
            "max_retries": config.max_retries,
            "retry_backoff": config.retry_backoff
        }
        
        with open(config_path, 'w') as f:
            json.dump(config_dict, f, indent=2)
        
    except Exception as e:
        logger.error(f"Failed to save config: {e}")

def main():
    """Main function for testing"""
    # Load configuration
    config = load_config()
    
    # Create detector
    detector = ProductionCoffeeDetector(config)
    
    try:
        # Perform health check
        health = detector.health_check()
        print(f"Health Check: {health}")
        
        if health["status"] != "healthy":
            print("System is not healthy. Exiting.")
            return
        
        # Main detection loop
        while True:
            input("Press Enter to detect coffee (Ctrl+C to exit)...")
            
            result = detector.detect_coffee()
            print(result)
            
    except KeyboardInterrupt:
        print("\nStopping detection system...")
    except Exception as e:
        logger.error(f"Unexpected error in main: {e}")
    finally:
        detector.cleanup()
        

if __name__ == "__main__":
    main()