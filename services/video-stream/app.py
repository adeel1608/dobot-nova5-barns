from fastapi import FastAPI, HTTPException, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
import cv2
import numpy as np
import time
from typing import Dict
import logging
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
from shared.logger import log
import math

app = FastAPI(title="BARNS Video Stream Service")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "http://localhost:3001",
        "http://127.0.0.1:3001"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class Camera:
    """Generic camera class - streams video or shows test pattern."""
    
    def __init__(self, name: str, camera_id: str, source=None, use_test_pattern=False):
        self.name = name
        self.camera_id = camera_id
        self.source = source  # Camera index or None for mock
        self.use_test_pattern = use_test_pattern
        self.cap = None
        self.active = False
        self.error_frame = None
        self.frame_count = 0
        self._initialize()
    
    def _initialize(self):
        """Try to initialize camera, fallback to test pattern or error frame."""
        if self.source is not None:
            try:
                log("INFO", "Attempting to initialize camera {self.name} with source {self.source}", service="video_stream")
                self.cap = cv2.VideoCapture(self.source)
                
                # Set some properties for better webcam compatibility
                if self.cap.isOpened():
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    self.cap.set(cv2.CAP_PROP_FPS, 30)
                    
                    # Test reading a frame
                    ret, frame = self.cap.read()
                    if ret and frame is not None:
                        self.active = True
                        log("INFO", "Camera {self.name} initialized successfully - Frame size: {frame.shape}", service="video_stream")
                        return
                    else:
                        log("ERROR", "Camera {self.name} opened but failed to read frame", service="video_stream")
                else:
                    log("ERROR", "Camera {self.name} failed to open", service="video_stream")
                    
            except Exception as e:
                log("ERROR", "Camera {self.name} failed to initialize: {e}", service="video_stream")
        
        # Create test pattern or error frame
        self.active = False
        if self.use_test_pattern:
            log("INFO", "Camera {self.name} using test pattern", service="video_stream")
        else:
            self._create_error_frame()
            log("INFO", "Camera {self.name} using error frame", service="video_stream")
    
    def _create_error_frame(self):
        """Create a simple error frame."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(frame, self.name, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame, "Camera not available", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(frame, "Using error frame", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
        
        success, jpeg = cv2.imencode('.jpg', frame)
        self.error_frame = jpeg.tobytes()
    
    def _create_test_pattern(self):
        """Create simple lightweight test pattern."""
        # Create simple black frame - very lightweight
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Add simple static text - no animations
        cv2.putText(frame, self.name, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame, "Test Video Stream", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
        cv2.putText(frame, f"Frame: {self.frame_count}", (20, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)
        
        # Simple timestamp - updates only once per second to reduce CPU
        timestamp = time.strftime("%H:%M:%S")
        cv2.putText(frame, timestamp, (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 1)
        
        # Add simple border for visual confirmation
        cv2.rectangle(frame, (10, 10), (630, 470), (64, 64, 64), 2)
        
        # Simple status indicator - just increment frame count
        self.frame_count += 1
        if self.frame_count > 999999:  # Reset to prevent overflow
            self.frame_count = 0
            
        return frame
    
    def get_frame(self) -> bytes:
        """Get current frame, test pattern, or error frame."""
        if self.active and self.cap and self.cap.isOpened():
            try:
                ret, frame = self.cap.read()
                if ret:
                    success, jpeg = cv2.imencode('.jpg', frame)
                    if success:
                        return jpeg.tobytes()
            except Exception as e:
                log("ERROR", "Error reading from {self.name}: {e}", service="video_stream")
                self.active = False
        
        if self.use_test_pattern:
            frame = self._create_test_pattern()
            success, jpeg = cv2.imencode('.jpg', frame)
            if success:
                return jpeg.tobytes()
        
        return self.error_frame if self.error_frame else b''
    
    def stop(self):
        """Stop camera."""
        if self.cap:
            self.cap.release()
        self.active = False
        log("INFO", "Camera {self.name} stopped", service="video_stream")

# Initialize cameras with test patterns for demonstration
RTSP_URL = "rtsp://admin:QSS2030QSS@192.168.200.106:554/stream1"

cameras: Dict[str, Camera] = {
    "ceiling": Camera("Ceiling Camera", "ceiling", RTSP_URL, use_test_pattern=False),  # RTSP ceiling camera
    "test_pattern": Camera("Test Pattern Demo", "test_pattern", None, use_test_pattern=True),  # Pure test pattern
    "camera1": Camera("Camera 1", "camera1", None),  # Mock camera with error frame
    "camera2": Camera("Camera 2", "camera2", None),  # Mock camera with error frame  
}

@app.on_event("startup")
async def startup_event():
    log("INFO", "BARNS Video service starting with {len(cameras)} cameras", service="video_stream")

@app.on_event("shutdown")
async def shutdown_event():
    log("INFO", "BARNS Video service shutting down", service="video_stream")
    for camera in cameras.values():
        camera.stop()

def gen_frames(camera_id: str):
    """Generate video frames."""
    camera = cameras.get(camera_id)
    if not camera:
        return
    
    while True:
        try:
            frame_bytes = camera.get_frame()
            if frame_bytes:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.1)  # ~10 FPS - reduced for lower CPU usage
        except Exception as e:
            log("ERROR", "Error streaming {camera_id}: {e}", service="video_stream")
            break

@app.get("/cameras")
def list_cameras():
    """List all cameras."""
    camera_info = {}
    for cam_id, camera in cameras.items():
        camera_info[cam_id] = {
            "name": camera.name,
            "status": "active" if camera.active else ("test_pattern" if camera.use_test_pattern else "error"),
            "stream_url": f"/stream/{cam_id}",
            "type": "real" if camera.active else ("test_pattern" if camera.use_test_pattern else "error")
        }
    return {"cameras": camera_info}

@app.get("/stream/{camera_id}")
async def get_stream(camera_id: str):
    """Stream video from camera."""
    if camera_id not in cameras:
        raise HTTPException(status_code=404, detail=f"Camera {camera_id} not found")
    
    return StreamingResponse(
        gen_frames(camera_id),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.get("/status")
def get_status():
    """Get service status."""
    camera_status = {}
    for cam_id, camera in cameras.items():
        camera_status[cam_id] = {
            "name": camera.name,
            "active": camera.active,
            "type": "real" if camera.active else ("test_pattern" if camera.use_test_pattern else "error")
        }
    
    return {
        "status": "operational",
        "cameras": camera_status,
        "message": "Video streaming service with test patterns for development"
    }

@app.get("/still/{camera_id}")
async def get_still_image(camera_id: str):
    """Get still image from camera."""
    if camera_id not in cameras:
        raise HTTPException(status_code=404, detail=f"Camera {camera_id} not found")
    
    frame_bytes = cameras[camera_id].get_frame()
    return Response(content=frame_bytes, media_type="image/jpeg")

@app.get("/debug")
async def debug_cameras():
    """Debug camera information and system status."""
    import os
    import glob
    
    debug_info = {
        "video_devices": [],
        "camera_backends": cv2.getBuildInformation(),
        "opencv_version": cv2.__version__,
        "system_info": {},
        "note": "Running in Docker - Windows webcams may not be accessible. Using test patterns for demonstration."
    }
    
    # Check for video devices
    video_devices = glob.glob("/dev/video*")
    debug_info["video_devices"] = video_devices
    
    # Check permissions
    for device in video_devices:
        try:
            if os.access(device, os.R_OK):
                debug_info["system_info"][device] = "readable"
            else:
                debug_info["system_info"][device] = "not readable"
        except Exception as e:
            debug_info["system_info"][device] = f"error: {e}"
    
    # Test camera initialization
    debug_info["camera_tests"] = {}
    for i in range(3):  # Test first 3 camera indices
        try:
            test_cap = cv2.VideoCapture(i)
            if test_cap.isOpened():
                ret, frame = test_cap.read()
                debug_info["camera_tests"][f"camera_{i}"] = {
                    "available": True,
                    "can_read": ret,
                    "frame_shape": frame.shape if ret else None
                }
                test_cap.release()
            else:
                debug_info["camera_tests"][f"camera_{i}"] = {"available": False}
        except Exception as e:
            debug_info["camera_tests"][f"camera_{i}"] = {"error": str(e)}
    
    return debug_info 