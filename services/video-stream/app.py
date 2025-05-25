from fastapi import FastAPI, HTTPException, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
import cv2
import numpy as np
import time
from typing import Dict
import logging

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
    """Generic camera class - streams video or shows error."""
    
    def __init__(self, name: str, camera_id: str, source=None):
        self.name = name
        self.camera_id = camera_id
        self.source = source  # Camera index or None for mock
        self.cap = None
        self.active = False
        self.error_frame = None
        self._initialize()
    
    def _initialize(self):
        """Try to initialize camera, fallback to error frame."""
        if self.source is not None:
            try:
                self.cap = cv2.VideoCapture(self.source)
                if self.cap.isOpened():
                    self.active = True
                    logger.info(f"Camera {self.name} initialized successfully")
                    return
            except Exception as e:
                logger.warning(f"Camera {self.name} failed to initialize: {e}")
        
        # Create simple error frame
        self.active = False
        self._create_error_frame()
        logger.info(f"Camera {self.name} using error frame")
    
    def _create_error_frame(self):
        """Create a simple error frame."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(frame, self.name, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame, "Camera not available", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        success, jpeg = cv2.imencode('.jpg', frame)
        self.error_frame = jpeg.tobytes()
    
    def get_frame(self) -> bytes:
        """Get current frame or error frame."""
        if self.active and self.cap and self.cap.isOpened():
            try:
                ret, frame = self.cap.read()
                if ret:
                    success, jpeg = cv2.imencode('.jpg', frame)
                    if success:
                        return jpeg.tobytes()
            except Exception as e:
                logger.error(f"Error reading from {self.name}: {e}")
                self.active = False
        
        return self.error_frame if self.error_frame else b''
    
    def stop(self):
        """Stop camera."""
        if self.cap:
            self.cap.release()
        self.active = False
        logger.info(f"Camera {self.name} stopped")

# Initialize cameras
cameras: Dict[str, Camera] = {
    "webcam": Camera("Live Webcam", "webcam", 0),  # Try real webcam
    "camera1": Camera("Camera 1", "camera1", None),  # Mock camera
    "camera2": Camera("Camera 2", "camera2", None),  # Mock camera  
    "camera3": Camera("Camera 3", "camera3", None),  # Mock camera
}

@app.on_event("startup")
async def startup_event():
    logger.info(f"BARNS Video service starting with {len(cameras)} cameras")

@app.on_event("shutdown")
async def shutdown_event():
    logger.info("BARNS Video service shutting down")
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
            time.sleep(0.1)  # 10 FPS
        except Exception as e:
            logger.error(f"Error streaming {camera_id}: {e}")
            break

@app.get("/cameras")
def list_cameras():
    """List all cameras."""
    camera_info = {}
    for cam_id, camera in cameras.items():
        camera_info[cam_id] = {
            "name": camera.name,
            "status": "active" if camera.active else "error",
            "stream_url": f"/stream/{cam_id}"
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
            "active": camera.active
        }
    
    return {
        "status": "operational",
        "cameras": camera_status
    }

@app.get("/still/{camera_id}")
async def get_still_image(camera_id: str):
    """Get still image from camera."""
    if camera_id not in cameras:
        raise HTTPException(status_code=404, detail=f"Camera {camera_id} not found")
    
    frame_bytes = cameras[camera_id].get_frame()
    return Response(content=frame_bytes, media_type="image/jpeg") 