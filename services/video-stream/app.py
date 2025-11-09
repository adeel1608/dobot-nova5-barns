from fastapi import FastAPI, HTTPException, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
import cv2
import time
from typing import Dict, Set
import logging
import sys
import os
from threading import Lock, Thread
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
        "http://127.0.0.1:3001",
        "http://localhost:5173",
        "http://127.0.0.1:5173"
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
        self.running = False
        self.capture_thread: Thread | None = None
        self.frame_lock = Lock()
        self.latest_jpeg: bytes | None = None
        self.last_init_attempt_ts = 0.0
        self.last_frame_ts = 0.0  # Last successful frame timestamp
        self._initialize()
    
    def _initialize(self):
        """Try to initialize camera with timeout and RTSP buffering optimization."""
        try:
            log("INFO", f"Attempting to initialize camera {self.name} with source {self.source}", service="video_stream")
            self.last_init_attempt_ts = time.time()
            # Guard camera object with a read lock to prevent concurrent cap access
            if not hasattr(self, "read_lock"):
                from threading import Lock as _Lock
                self.read_lock = _Lock()
            
            # For RTSP streams, use FFMPEG backend for better buffering control
            if isinstance(self.source, str) and self.source.startswith('rtsp'):
                # Try FFMPEG backend first (better for RTSP)
                self.cap = cv2.VideoCapture(self.source, cv2.CAP_FFMPEG)
                
                # Set timeout properties for RTSP streams
                self.cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000)  # 5 second connection timeout
                self.cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 3000)  # 3 second read timeout
                
                # Use single-threaded decoding to avoid pthread_frame assertion issues
                try:
                    self.cap.set(cv2.CAP_PROP_THREADS, 1)
                except Exception:
                    pass
                
                # Critical: Reduce buffer size to minimize latency and prevent buffering
                # Buffer size of 1 means minimal buffering - always get latest frame
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                # Additional RTSP optimizations
                # Set to drop frames when buffer is full (get latest frame only)
                # This prevents old frames from being displayed
                try:
                    # Some backends support this property
                    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'H264'))
                except:
                    pass  # Not critical if not supported
            else:
                self.cap = cv2.VideoCapture(self.source)
            
            # Set some properties for better webcam compatibility
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                # Lower target FPS slightly to reduce CPU and decoder pressure
                try:
                    self.cap.set(cv2.CAP_PROP_FPS, 15)
                except Exception:
                    pass
                
                # For RTSP, ensure buffer is minimal
                if isinstance(self.source, str) and self.source.startswith('rtsp'):
                    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                # Test reading a frame with timeout
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    self.active = True
                    self._start_capture()
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
    
    def ensure_ready(self, min_retry_interval_sec: float = 2.0):
        """Attempt to (re)initialize the camera if inactive, throttled by min_retry_interval_sec."""
        if self.active:
            # Make sure capture thread is running even if camera is active
            if not self.running:
                self._start_capture()
            return
        now = time.time()
        if now - self.last_init_attempt_ts < min_retry_interval_sec:
            return
        self._initialize()
    
    def _start_capture(self):
        """Start background capture thread."""
        if self.running:
            return
        
        # Ensure locks are initialized
        if not hasattr(self, "read_lock"):
            self.read_lock = Lock()
        if not hasattr(self, "frame_lock"):
            self.frame_lock = Lock()
        
        self.running = True
        log("INFO", f"Starting capture thread for {self.name}", service="video_stream")
        
        def _loop():
            delay = 1.0 / 15.0  # ~15 FPS target
            stall_reset_seconds = 3.0  # If no fresh frame for this long, force reconnect
            consecutive_failures = 0
            while self.running and self.cap and self.cap.isOpened():
                try:
                    with self.read_lock:
                        ret, frame = self.cap.read()

                    if ret and frame is not None:
                        consecutive_failures = 0
                        success, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        if success:
                            with self.frame_lock:
                                self.latest_jpeg = jpeg.tobytes()
                                self.last_frame_ts = time.time()
                    else:
                        consecutive_failures += 1
                        # Back off slightly on read failure
                        time.sleep(0.05)

                    # Detect capture stall even if cap remains open
                    if self.last_frame_ts and (time.time() - self.last_frame_ts) > stall_reset_seconds:
                        log("WARNING", f"Capture appears stalled for {self.name} (> {stall_reset_seconds}s) - forcing reconnect", service="video_stream")
                        self.active = False
                        try:
                            with self.read_lock:
                                if self.cap:
                                    self.cap.release()
                        except Exception:
                            pass
                        break

                    # Too many consecutive failures, force reconnect
                    if consecutive_failures > 30:
                        log("WARNING", f"Consecutive capture failures for {self.name} - forcing reconnect", service="video_stream")
                        self.active = False
                        try:
                            with self.read_lock:
                                if self.cap:
                                    self.cap.release()
                        except Exception:
                            pass
                        break

                except Exception as e:
                    log("ERROR", f"Capture loop error for {self.name}: {e}", service="video_stream")
                    time.sleep(0.1)

                time.sleep(delay)

            self.running = False
            log("INFO", f"Capture thread ended for {self.name}", service="video_stream")
        
        self.capture_thread = Thread(target=_loop, name=f"cap-{self.camera_id}", daemon=True)
        self.capture_thread.start()
    
    def get_frame(self) -> bytes:
        """Return the most recently captured JPEG frame, if any."""
        if not (self.active and self.cap and self.cap.isOpened()):
            return None
        
        # Ensure capture thread is running
        if not self.running:
            self._start_capture()
            # Wait briefly for first frame
            for _ in range(20):  # Up to 2 seconds
                time.sleep(0.1)
                with self.frame_lock:
                    if self.latest_jpeg:
                        return self.latest_jpeg
            return None
        
        # If frames are stale, mark camera inactive to trigger quick reconnect
        if self.last_frame_ts and (time.time() - self.last_frame_ts) > 3.0:
            log("WARNING", f"Stale frame detected for {self.name} - marking inactive for reconnect", service="video_stream")
            self.active = False
            return None

        with self.frame_lock:
            return self.latest_jpeg
    
    def stop(self):
        """Stop camera and capture thread."""
        # Stop capture thread first
        self.running = False
        if hasattr(self, 'capture_thread') and self.capture_thread and self.capture_thread.is_alive():
            try:
                self.capture_thread.join(timeout=0.5)
            except Exception:
                pass
        
        # Release camera
        if self.cap:
            try:
                if hasattr(self, "read_lock"):
                    with self.read_lock:
                        self.cap.release()
                else:
                    self.cap.release()
            except Exception:
                pass
        
        self.active = False
        with self.frame_lock:
            self.latest_jpeg = None
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
        self.cooldown_until: Dict[str, float] = {}  # camera_id -> timestamp when cooldown expires
        self.session_start_time: Dict[str, float] = {}  # session_id -> start timestamp
        self.lock = Lock()
        self.COOLDOWN_SECONDS = 2.0  # Cooldown period after stopping streams (2 seconds)
        self.MIN_CONNECTION_AGE = 0.5  # Minimum time a connection should live before cleanup (500ms)
    
    def start_stream(self, camera_id: str, session_id: str = "default") -> bool:
        """Register a new stream session. Returns False if in cooldown."""
        with self.lock:
            # Check if camera is in cooldown
            if camera_id in self.cooldown_until:
                cooldown_remaining = self.cooldown_until[camera_id] - time.time()
                if cooldown_remaining > 0:
                    log("WARNING", f"Cannot start stream for {camera_id} - cooldown active for {cooldown_remaining:.2f}s", service="video_stream")
                    return False
                else:
                    # Cooldown expired, clean it up
                    del self.cooldown_until[camera_id]
            
            # Check for duplicate session (rapid reconnection)
            if camera_id in self.active_streams and session_id in self.active_streams[camera_id]:
                log("WARNING", f"Duplicate session detected for {camera_id}, session: {session_id} - ignoring", service="video_stream")
                return False
            
            if camera_id not in self.active_streams:
                self.active_streams[camera_id] = set()
            
            self.active_streams[camera_id].add(session_id)
            self.session_start_time[session_id] = time.time()
            log("INFO", f"Stream started for {camera_id}, session: {session_id}, active sessions: {len(self.active_streams[camera_id])}", service="video_stream")
            return True
    
    def stop_stream(self, camera_id: str, session_id: str = "default", apply_cooldown: bool = False):
        """Unregister a stream session. Optionally set cooldown (only for explicit stops, not natural disconnections)."""
        with self.lock:
            # Check if this was a short-lived connection (possible hot reload or rapid toggle)
            connection_age = 0
            if session_id in self.session_start_time:
                connection_age = time.time() - self.session_start_time[session_id]
                del self.session_start_time[session_id]
            
            if connection_age < self.MIN_CONNECTION_AGE and connection_age > 0:
                log("INFO", f"Short-lived connection detected ({connection_age:.2f}s) for {camera_id}, session: {session_id} - likely hot reload or rapid toggle", service="video_stream")
            
            if camera_id in self.active_streams:
                self.active_streams[camera_id].discard(session_id)
                if not self.active_streams[camera_id]:
                    del self.active_streams[camera_id]
                    
                    if apply_cooldown:
                        # Only apply cooldown for explicit stops (stop-all), not natural disconnections
                        self.cooldown_until[camera_id] = time.time() + self.COOLDOWN_SECONDS
                        log("INFO", f"All streams stopped for {camera_id}, cooldown until {self.cooldown_until[camera_id]:.2f}", service="video_stream")
                    else:
                        log("INFO", f"Stream naturally ended for {camera_id} - no cooldown applied", service="video_stream")
                else:
                    log("INFO", f"Stream session {session_id} stopped for {camera_id}, remaining: {len(self.active_streams[camera_id])}", service="video_stream")
    
    def stop_all_streams(self):
        """Stop all streams and set cooldown for all cameras."""
        with self.lock:
            stopped_cameras = list(self.active_streams.keys())
            cooldown_until = time.time() + self.COOLDOWN_SECONDS
            
            for cam_id in stopped_cameras:
                self.cooldown_until[cam_id] = cooldown_until
            
            self.active_streams.clear()
            log("INFO", f"Stopped all streams - {len(stopped_cameras)} cameras, cooldown until {cooldown_until:.2f}", service="video_stream")
            return stopped_cameras
    
    def is_stream_active(self, camera_id: str) -> bool:
        """Check if any sessions are active for a camera."""
        with self.lock:
            return camera_id in self.active_streams and len(self.active_streams[camera_id]) > 0
    
    def get_active_cameras(self) -> Set[str]:
        """Get set of all cameras with active streams."""
        with self.lock:
            return set(self.active_streams.keys())
    
    def get_cooldown_status(self, camera_id: str) -> dict:
        """Get cooldown status for a camera."""
        with self.lock:
            if camera_id not in self.cooldown_until:
                return {"in_cooldown": False, "remaining_seconds": 0}
            
            remaining = self.cooldown_until[camera_id] - time.time()
            if remaining <= 0:
                # Cooldown expired, clean it up
                del self.cooldown_until[camera_id]
                return {"in_cooldown": False, "remaining_seconds": 0}
            
            return {"in_cooldown": True, "remaining_seconds": round(remaining, 2)}

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
    # Ensure all active cameras have capture threads running
    for cam_id, camera in cameras.items():
        if camera.active and not camera.running:
            log("INFO", f"Starting capture thread for {cam_id}", service="video_stream")
            camera._start_capture()

@app.on_event("shutdown")
async def shutdown_event():
    log("INFO", "BARNS Video service shutting down", service="video_stream")
    for camera in cameras.values():
        camera.stop()

def gen_frames(camera_id: str, session_id: str = "default"):
    """Generate video frames - only when stream is active and camera is available."""
    camera = cameras.get(camera_id)
    if not camera:
        log("ERROR", f"Camera {camera_id} not found", service="video_stream")
        return
    
    # Check cooldown before starting
    cooldown_status = stream_manager.get_cooldown_status(camera_id)
    if cooldown_status["in_cooldown"]:
        log("WARNING", f"Cannot start stream for {camera_id} - cooldown active for {cooldown_status['remaining_seconds']}s", service="video_stream")
        return
    
    # Check if camera is active before starting stream
    if not camera.active:
        log("WARNING", f"Attempted to stream from inactive camera {camera_id}", service="video_stream")
        return
    
    # Register this stream session
    if not stream_manager.start_stream(camera_id, session_id):
        log("WARNING", f"Failed to start stream for {camera_id} - cooldown or duplicate session", service="video_stream")
        return
    
    log("INFO", f"Stream generator started for {camera_id}, session: {session_id}", service="video_stream")
    
    frame_count = 0
    last_log_time = time.time()
    
    try:
        while stream_manager.is_stream_active(camera_id):
            try:
                # Check status before expensive frame capture
                if not stream_manager.is_stream_active(camera_id):
                    log("INFO", f"Stream {camera_id} marked inactive, stopping generator (frames: {frame_count})", service="video_stream")
                    break
                
                # Ensure camera is ready (attempt reconnection if needed, throttled)
                camera.ensure_ready()
                
                frame_bytes = camera.get_frame()
                if frame_bytes:
                    frame_count += 1
                    
                    # Log progress every 10 seconds
                    if time.time() - last_log_time > 10:
                        log("INFO", f"Stream {camera_id} session {session_id}: {frame_count} frames delivered", service="video_stream")
                        last_log_time = time.time()
                    
                    # Yield frame - if client disconnected, this will raise an exception
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                else:
                    # No frame available now - keep connection alive and retry
                    time.sleep(0.05)
                    continue
                
                # Sleep in smaller increments to check stop status more frequently
                # This allows streams to stop faster when stop_all is called
                for _ in range(10):  # 10 x 0.01s = 0.1s total, but checks every 0.01s
                    if not stream_manager.is_stream_active(camera_id):
                        log("INFO", f"Stream {camera_id} stopped during sleep, exiting (frames: {frame_count})", service="video_stream")
                        break
                    time.sleep(0.01)
                else:
                    continue  # Continue if loop completed normally
                break  # Break outer loop if inner loop broke
                
            except GeneratorExit:
                # Client disconnected - this is the clean way to exit
                log("INFO", f"Client disconnected from {camera_id}, session: {session_id} (frames delivered: {frame_count})", service="video_stream")
                break
            except Exception as e:
                log("ERROR", f"Error streaming {camera_id}, session {session_id}: {e}", service="video_stream")
                # Brief backoff before breaking - connection might be broken
                time.sleep(0.1)
                break  # Exit on errors to clean up the connection
    finally:
        # Cleanup: unregister this stream session (no cooldown for natural disconnections)
        stream_manager.stop_stream(camera_id, session_id, apply_cooldown=False)
        log("INFO", f"Stream generator ended for {camera_id}, session: {session_id}, total frames: {frame_count}", service="video_stream")

@app.get("/cameras")
def list_cameras():
    """List all cameras with stream status and cooldown info."""
    camera_info = {}
    active_cameras = stream_manager.get_active_cameras()
    for cam_id, camera in cameras.items():
        cooldown = stream_manager.get_cooldown_status(cam_id)
        camera_info[cam_id] = {
            "name": camera.name,
            "status": "active" if camera.active else "offline",
            "stream_url": f"/stream/{cam_id}",
            "type": "RTSP Stream" if camera.active else "Offline",
            "streaming": cam_id in active_cameras,
            "cooldown": cooldown
        }
    return {"cameras": camera_info}

@app.get("/stream/{camera_id}")
async def get_stream(camera_id: str):
    """Stream video from camera - stream will auto-start when client connects."""
    if camera_id not in cameras:
        raise HTTPException(status_code=404, detail=f"Camera {camera_id} not found")
    
    # If in cooldown, return 503 so clients don't open empty streams
    cooldown = stream_manager.get_cooldown_status(camera_id)
    if cooldown.get("in_cooldown"):
        from fastapi import HTTPException as _HTTPException
        raise _HTTPException(
            status_code=503,
            detail=f"Camera {camera_id} cooling down",
            headers={"Retry-After": str(int(cooldown.get("remaining_seconds", 1) or 1))}
        )

    # If camera inactive, return 503
    cam = cameras[camera_id]
    if not cam.active:
        # Try a quick reconnect attempt before failing
        cam.ensure_ready()
        if not cam.active:
            from fastapi import HTTPException as _HTTPException
            raise _HTTPException(status_code=503, detail=f"Camera {camera_id} is offline", headers={"Retry-After": "1"})

    # Generate a unique session ID for this stream connection
    import uuid
    session_id = str(uuid.uuid4())
    
    return StreamingResponse(
        gen_frames(camera_id, session_id),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={
            "Cache-Control": "no-cache, no-store, must-revalidate",
            "Pragma": "no-cache",
            "Expires": "0",
            "Connection": "keep-alive"
        }
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
    """Manually stop a stream with cooldown."""
    if camera_id not in cameras:
        raise HTTPException(status_code=404, detail=f"Camera {camera_id} not found")
    
    # Manual stop should apply cooldown
    stream_manager.stop_stream(camera_id, "manual", apply_cooldown=True)
    return {
        "status": "success",
        "message": f"Stream stopped for {camera_id}",
        "camera": camera_id
    }

@app.post("/stream/stop-all")
async def stop_all_streams():
    """Stop all active streams and set cooldown."""
    stopped_cameras = stream_manager.stop_all_streams()
    cooldown_seconds = stream_manager.COOLDOWN_SECONDS
    
    return {
        "status": "success",
        "message": "All streams stopped",
        "stopped_cameras": stopped_cameras,
        "cooldown_seconds": cooldown_seconds,
        "ready_at": time.time() + cooldown_seconds
    }

@app.get("/stream/active")
async def get_active_streams():
    """Get currently active streams."""
    active_cameras = stream_manager.get_active_cameras()
    stream_details = {}
    
    with stream_manager.lock:
        for cam_id in active_cameras:
            session_count = len(stream_manager.active_streams.get(cam_id, set()))
            stream_details[cam_id] = {
                "camera_name": cameras[cam_id].name if cam_id in cameras else "Unknown",
                "active_sessions": session_count
            }
    
    return {
        "status": "success",
        "active_camera_count": len(active_cameras),
        "active_cameras": list(active_cameras),
        "details": stream_details
    }

@app.get("/stream/cooldown/all")
async def get_all_cooldown_status():
    """Get cooldown status for all cameras."""
    cooldown_info = {}
    for cam_id in cameras.keys():
        cooldown_info[cam_id] = stream_manager.get_cooldown_status(cam_id)
    
    return {
        "status": "success",
        "cooldowns": cooldown_info
    }

@app.get("/stream/cooldown/{camera_id}")
async def get_cooldown_status(camera_id: str):
    """Get cooldown status for a specific camera."""
    if camera_id not in cameras:
        raise HTTPException(status_code=404, detail=f"Camera {camera_id} not found")
    
    cooldown = stream_manager.get_cooldown_status(camera_id)
    return {
        "status": "success",
        "camera_id": camera_id,
        "cooldown": cooldown
    }

@app.get("/status")
def get_status():
    """Get service status with stream information."""
    camera_status = {}
    active_cameras = stream_manager.get_active_cameras()
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
        "active_streams": len(active_cameras),
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