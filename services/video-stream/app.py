from fastapi import FastAPI, HTTPException, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
import cv2
import time
from typing import Dict, Set
import logging
import sys
import os
from threading import Lock
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
from shared.logger import log

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
    """Generic camera class - streams video from real sources only."""
    
    def __init__(self, name: str, camera_id: str, source: str):
        self.name = name
        self.camera_id = camera_id
        self.source = source  # Camera source (RTSP URL, device index, etc.)
        self.cap = None
        self.active = False
        self._initialize()
    
    def _initialize(self):
        """Try to initialize camera with timeout."""
        try:
            log("INFO", f"Attempting to initialize camera {self.name} with source {self.source}", service="video_stream")
            self.cap = cv2.VideoCapture(self.source)
            
            # Set timeout properties for RTSP streams to prevent hanging
            if isinstance(self.source, str) and self.source.startswith('rtsp'):
                self.cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000)  # 5 second connection timeout
                self.cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000)  # 5 second read timeout
            
            # Set some properties for better webcam compatibility
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                
                # Test reading a frame with timeout
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    self.active = True
                    log("INFO", f"Camera {self.name} initialized successfully - Frame size: {frame.shape}", service="video_stream")
                    return
                else:
                    log("ERROR", f"Camera {self.name} opened but failed to read frame", service="video_stream")
            else:
                log("ERROR", f"Camera {self.name} failed to open", service="video_stream")
                
        except Exception as e:
            log("ERROR", f"Camera {self.name} failed to initialize: {e}", service="video_stream")
        
        # Camera failed to initialize
        self.active = False
        log("WARNING", f"Camera {self.name} is not available - will show as offline", service="video_stream")
    
    def get_frame(self) -> bytes:
        """Get current frame from real camera only."""
        if self.active and self.cap and self.cap.isOpened():
            try:
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    success, jpeg = cv2.imencode('.jpg', frame)
                    if success:
                        return jpeg.tobytes()
            except Exception as e:
                log("ERROR", f"Error reading from {self.name}: {e}", service="video_stream")
                self.active = False
        
        # No frame available - camera is offline
        return None
    
    def stop(self):
        """Stop camera."""
        if self.cap:
            self.cap.release()
        self.active = False
        log("INFO", f"Camera {self.name} stopped", service="video_stream")
    
    def restart(self):
        """Restart camera connection."""
        self.stop()
        self._initialize()

# Stream session management
class StreamManager:
    """Manages active stream sessions to optimize resource usage."""
    
    def __init__(self):
        self.active_streams: Dict[str, Set[str]] = {}  # camera_id -> set of session_ids
        self.lock = Lock()
    
    def start_stream(self, camera_id: str, session_id: str = "default"):
        """Register a new stream session."""
        with self.lock:
            if camera_id not in self.active_streams:
                self.active_streams[camera_id] = set()
            self.active_streams[camera_id].add(session_id)
            log("INFO", f"Stream started for {camera_id}, session: {session_id}, active sessions: {len(self.active_streams[camera_id])}", service="video_stream")
    
    def stop_stream(self, camera_id: str, session_id: str = "default"):
        """Unregister a stream session."""
        with self.lock:
            if camera_id in self.active_streams:
                self.active_streams[camera_id].discard(session_id)
                if not self.active_streams[camera_id]:
                    del self.active_streams[camera_id]
                    log("INFO", f"All streams stopped for {camera_id}", service="video_stream")
                else:
                    log("INFO", f"Stream session {session_id} stopped for {camera_id}, remaining: {len(self.active_streams[camera_id])}", service="video_stream")
    
    def is_stream_active(self, camera_id: str) -> bool:
        """Check if any sessions are active for a camera."""
        with self.lock:
            return camera_id in self.active_streams and len(self.active_streams[camera_id]) > 0
    
    def get_active_cameras(self) -> Set[str]:
        """Get set of all cameras with active streams."""
        with self.lock:
            return set(self.active_streams.keys())
    
    def get_total_sessions(self) -> int:
        """Get total number of active sessions across all cameras."""
        with self.lock:
            return sum(len(sessions) for sessions in self.active_streams.values())

stream_manager = StreamManager()

# Initialize real cameras only
RTSP_URL = "rtsp://admin:QSS2030QSS@192.168.200.106:554/stream1"

cameras: Dict[str, Camera] = {
    "ceiling": Camera("Ceiling Camera", "ceiling", RTSP_URL),  # RTSP ceiling camera
    # Add more real cameras here as needed
}

@app.on_event("startup")
async def startup_event():
    log("INFO", f"BARNS Video service starting with {len(cameras)} cameras", service="video_stream")

@app.on_event("shutdown")
async def shutdown_event():
    log("INFO", "BARNS Video service shutting down", service="video_stream")
    for camera in cameras.values():
        camera.stop()

def gen_frames(camera_id: str, session_id: str = "default"):
    """Generate video frames - only when stream is active and camera is available."""
    camera = cameras.get(camera_id)
    if not camera:
        return
    
    # Check if camera is active before starting stream
    if not camera.active:
        log("WARNING", f"Attempted to stream from inactive camera {camera_id}", service="video_stream")
        return
    
    # Register this stream session
    stream_manager.start_stream(camera_id, session_id)
    log("INFO", f"Stream generator started for {camera_id}, session: {session_id}", service="video_stream")
    
    frame_count = 0
    try:
        while stream_manager.is_stream_active(camera_id):
            try:
                # Check stream status BEFORE expensive frame capture
                if not stream_manager.is_stream_active(camera_id):
                    log("INFO", f"Stream {camera_id} marked inactive, stopping generator", service="video_stream")
                    break
                
                frame_bytes = camera.get_frame()
                if frame_bytes:
                    # Yield frame - if client disconnected, this will raise an exception
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                    frame_count += 1
                else:
                    # Camera became unavailable during streaming
                    log("WARNING", f"Camera {camera_id} became unavailable during streaming", service="video_stream")
                    break
                
                # Sleep in smaller increments to check stop status more frequently
                should_continue = True
                for _ in range(5):  # 5 x 0.02s = 0.1s total, but checks every 0.02s
                    if not stream_manager.is_stream_active(camera_id):
                        log("INFO", f"Stream {camera_id} stopped during sleep, exiting", service="video_stream")
                        should_continue = False
                        break
                    time.sleep(0.02)
                
                if not should_continue:
                    break
            except GeneratorExit:
                # Client disconnected - this is the clean way to exit
                log("INFO", f"Client disconnected from {camera_id}, session: {session_id}, frames sent: {frame_count}", service="video_stream")
                break
            except Exception as e:
                log("ERROR", f"Error streaming {camera_id}: {e}", service="video_stream")
                break
    finally:
        # Cleanup: unregister this stream session
        stream_manager.stop_stream(camera_id, session_id)
        log("INFO", f"Stream generator ended for {camera_id}, session: {session_id}, total frames: {frame_count}", service="video_stream")

@app.get("/cameras")
def list_cameras():
    """List all cameras with stream status."""
    camera_info = {}
    active_cameras = stream_manager.get_active_cameras()
    for cam_id, camera in cameras.items():
        camera_info[cam_id] = {
            "name": camera.name,
            "status": "active" if camera.active else "offline",
            "stream_url": f"/stream/{cam_id}",
            "type": "RTSP Stream" if camera.active else "Offline",
            "streaming": cam_id in active_cameras
        }
    return {"cameras": camera_info}

@app.get("/stream/{camera_id}")
async def get_stream(camera_id: str):
    """Stream video from camera - stream will auto-start when client connects."""
    if camera_id not in cameras:
        raise HTTPException(status_code=404, detail=f"Camera {camera_id} not found")
    
    # Generate a unique session ID for this stream connection
    import uuid
    session_id = str(uuid.uuid4())
    
    return StreamingResponse(
        gen_frames(camera_id, session_id),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.post("/stream/start/{camera_id}")
async def start_stream(camera_id: str):
    """Manually start a stream (for pre-warming)."""
    if camera_id not in cameras:
        raise HTTPException(status_code=404, detail=f"Camera {camera_id} not found")
    
    stream_manager.start_stream(camera_id, "manual")
    return {
        "status": "success",
        "message": f"Stream started for {camera_id}",
        "camera": camera_id
    }

@app.post("/stream/stop/{camera_id}")
async def stop_stream(camera_id: str):
    """Manually stop a stream."""
    if camera_id not in cameras:
        raise HTTPException(status_code=404, detail=f"Camera {camera_id} not found")
    
    stream_manager.stop_stream(camera_id, "manual")
    return {
        "status": "success",
        "message": f"Stream stopped for {camera_id}",
        "camera": camera_id
    }

@app.post("/stream/stop-all")
async def stop_all_streams():
    """Stop all active streams by clearing all sessions."""
    stopped_count = 0
    stopped_cameras = []
    
    with stream_manager.lock:
        # Get all active cameras and their session counts
        for cam_id in list(stream_manager.active_streams.keys()):
            session_count = len(stream_manager.active_streams[cam_id])
            stopped_count += session_count
            stopped_cameras.append({
                "camera_id": cam_id,
                "sessions_stopped": session_count
            })
        
        # Clear ALL sessions for ALL cameras
        stream_manager.active_streams.clear()
    
    log("INFO", f"Stopped all streams - {len(stopped_cameras)} cameras, {stopped_count} total sessions", service="video_stream")
    return {
        "status": "success",
        "message": "All streams stopped",
        "cameras_affected": len(stopped_cameras),
        "total_sessions_stopped": stopped_count,
        "details": stopped_cameras
    }

@app.get("/stream/active")
async def get_active_streams():
    """Get currently active streams with detailed session information."""
    active_cameras = stream_manager.get_active_cameras()
    stream_details = {}
    total_sessions = 0
    
    with stream_manager.lock:
        for cam_id in active_cameras:
            sessions = stream_manager.active_streams.get(cam_id, set())
            session_count = len(sessions)
            total_sessions += session_count
            stream_details[cam_id] = {
                "camera_name": cameras[cam_id].name if cam_id in cameras else "Unknown",
                "active_sessions": session_count,
                "session_ids": list(sessions)[:5]  # Show first 5 session IDs for debugging
            }
    
    return {
        "status": "success",
        "active_camera_count": len(active_cameras),
        "total_active_sessions": total_sessions,
        "active_cameras": list(active_cameras),
        "details": stream_details,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
    }

@app.get("/status")
def get_status():
    """Get service status with stream information."""
    camera_status = {}
    active_cameras = stream_manager.get_active_cameras()
    total_sessions = stream_manager.get_total_sessions()
    
    for cam_id, camera in cameras.items():
        camera_status[cam_id] = {
            "name": camera.name,
            "active": camera.active,
            "type": "RTSP Stream" if camera.active else "Offline",
            "streaming": cam_id in active_cameras
        }
    
    return {
        "status": "operational",
        "cameras": camera_status,
        "active_cameras": len(active_cameras),
        "active_sessions": total_sessions,
        "message": "Video streaming service with on-demand streaming"
    }

@app.get("/still/{camera_id}")
async def get_still_image(camera_id: str):
    """Get still image from camera."""
    if camera_id not in cameras:
        raise HTTPException(status_code=404, detail=f"Camera {camera_id} not found")
    
    camera = cameras[camera_id]
    if not camera.active:
        raise HTTPException(status_code=503, detail=f"Camera {camera_id} is offline")
    
    frame_bytes = camera.get_frame()
    if not frame_bytes:
        raise HTTPException(status_code=503, detail=f"Camera {camera_id} failed to capture frame")
    
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