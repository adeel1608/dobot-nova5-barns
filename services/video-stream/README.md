# Video Stream Service

## Purpose and Workflow

The Video Stream Service provides real-time video streaming capabilities for the BARNS system, enabling monitoring and visualization of the coffee-making process. It manages multiple camera feeds and provides both live streaming and still image capture functionality.

### Core Responsibilities
- **Multi-Camera Management**: Manages multiple camera sources (webcams, IP cameras, mock cameras)
- **Live Video Streaming**: Provides real-time video streams using MJPEG format
- **Still Image Capture**: Captures and serves individual frames from cameras
- **Error Handling**: Graceful fallback when cameras are unavailable
- **Dashboard Integration**: Supplies video feeds for the monitoring dashboard
- **Performance Optimization**: Manages frame rates and streaming quality

### Workflow
1. **Camera Initialization**: Detects and initializes available cameras on startup
2. **Stream Management**: Creates and manages continuous video streams
3. **Frame Processing**: Processes video frames and encodes them for streaming
4. **Error Recovery**: Provides error frames when cameras are unavailable
5. **Client Serving**: Serves video streams to dashboard and other clients

### Streaming Flow
```
Camera Source → Frame Capture → JPEG Encoding → MJPEG Stream → Client Browser
```

## API Structure

### Core Endpoints

#### List Available Cameras
```http
GET /cameras
```

**Response:**
```json
{
  "cameras": {
    "webcam": {
      "name": "Live Webcam",
      "status": "active",
      "stream_url": "/stream/webcam"
    },
    "camera1": {
      "name": "Camera 1",
      "status": "error",
      "stream_url": "/stream/camera1"
    }
  }
}
```

#### Get Video Stream
```http
GET /stream/{camera_id}
```

**Response:** MJPEG video stream
- **Content-Type:** `multipart/x-mixed-replace; boundary=frame`
- **Format:** Continuous JPEG frames

#### Get Still Image
```http
GET /still/{camera_id}
```

**Response:** Single JPEG image
- **Content-Type:** `image/jpeg`

#### Service Status
```http
GET /status
```

**Response:**
```json
{
  "status": "operational",
  "cameras": {
    "webcam": {
      "name": "Live Webcam",
      "active": true
    },
    "camera1": {
      "name": "Camera 1", 
      "active": false
    }
  }
}
```

## Camera Management System

### Camera Class Structure
```python
class Camera:
    def __init__(self, name: str, camera_id: str, source=None):
        self.name = name           # Display name
        self.camera_id = camera_id # Unique identifier
        self.source = source       # Camera source (index, URL, etc.)
        self.active = False        # Camera status
        self.cap = None           # OpenCV VideoCapture object
```

### Camera Source Types
- **Webcam**: Local USB/built-in cameras (`source=0, 1, 2...`)
- **IP Camera**: Network cameras (`source="rtsp://camera_ip:port/stream"`)
- **Mock Camera**: Error frames when hardware unavailable (`source=None`)
- **Video File**: Playback from file (`source="/path/to/video.mp4"`)

### Error Handling
- **Automatic Fallback**: Displays error frames when cameras fail
- **Graceful Degradation**: Service continues operating with available cameras
- **Status Tracking**: Maintains active/inactive status for each camera

## Adding New Modules

### 1. Adding New Camera Sources

**Step 1**: Add camera to initialization:
```python
cameras: Dict[str, Camera] = {
    # ... existing cameras
    "new_camera": Camera("New Camera Name", "new_camera", "rtsp://192.168.1.100/stream"),
    "usb_camera_2": Camera("USB Camera 2", "usb_camera_2", 1),
}
```

**Step 2**: For IP cameras, update initialization:
```python
class IPCamera(Camera):
    def __init__(self, name: str, camera_id: str, ip_address: str, username: str = None, password: str = None):
        # Construct RTSP URL
        if username and password:
            source = f"rtsp://{username}:{password}@{ip_address}/stream"
        else:
            source = f"rtsp://{ip_address}/stream"
        super().__init__(name, camera_id, source)
```

### 2. Adding Video Processing Features

**Step 1**: Create video processor:
```python
# video_stream/processors.py
import cv2
import numpy as np

class VideoProcessor:
    def __init__(self):
        self.filters = []
        self.overlays = []
    
    def add_timestamp_overlay(self, frame: np.ndarray) -> np.ndarray:
        """Add timestamp overlay to frame."""
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(frame, timestamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return frame
    
    def add_motion_detection(self, frame: np.ndarray, background: np.ndarray) -> np.ndarray:
        """Add motion detection overlay."""
        # Motion detection logic
        diff = cv2.absdiff(frame, background)
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)
        
        # Find contours and draw bounding boxes
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Filter small movements
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        return frame
    
    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        """Apply all processing to frame."""
        processed_frame = frame.copy()
        processed_frame = self.add_timestamp_overlay(processed_frame)
        return processed_frame
```

**Step 2**: Integrate processing into Camera class:
```python
class ProcessedCamera(Camera):
    def __init__(self, name: str, camera_id: str, source=None, enable_processing=True):
        super().__init__(name, camera_id, source)
        self.processor = VideoProcessor() if enable_processing else None
    
    def get_frame(self) -> bytes:
        """Get processed frame."""
        if self.active and self.cap and self.cap.isOpened():
            try:
                ret, frame = self.cap.read()
                if ret:
                    # Apply processing if enabled
                    if self.processor:
                        frame = self.processor.process_frame(frame)
                    
                    success, jpeg = cv2.imencode('.jpg', frame)
                    if success:
                        return jpeg.tobytes()
            except Exception as e:
                logger.error(f"Error reading from {self.name}: {e}")
                self.active = False
        
        return self.error_frame if self.error_frame else b''
```

### 3. Adding Recording Capabilities

**Step 1**: Create video recorder:
```python
# video_stream/recorder.py
import cv2
import os
from datetime import datetime

class VideoRecorder:
    def __init__(self, output_dir: str = "recordings"):
        self.output_dir = output_dir
        self.active_recordings = {}
        os.makedirs(output_dir, exist_ok=True)
    
    def start_recording(self, camera_id: str, fps: int = 10):
        """Start recording from camera."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{camera_id}_{timestamp}.avi"
        filepath = os.path.join(self.output_dir, filename)
        
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        writer = cv2.VideoWriter(filepath, fourcc, fps, (640, 480))
        
        self.active_recordings[camera_id] = {
            "writer": writer,
            "filepath": filepath,
            "start_time": datetime.now()
        }
        
        return filepath
    
    def write_frame(self, camera_id: str, frame: np.ndarray):
        """Write frame to recording."""
        if camera_id in self.active_recordings:
            self.active_recordings[camera_id]["writer"].write(frame)
    
    def stop_recording(self, camera_id: str):
        """Stop recording for camera."""
        if camera_id in self.active_recordings:
            self.active_recordings[camera_id]["writer"].release()
            filepath = self.active_recordings[camera_id]["filepath"]
            del self.active_recordings[camera_id]
            return filepath
        return None
```

**Step 2**: Add recording endpoints:
```python
# In app.py
from .recorder import VideoRecorder

recorder = VideoRecorder()

@app.post("/record/start/{camera_id}")
async def start_recording(camera_id: str):
    """Start recording from camera."""
    if camera_id not in cameras:
        raise HTTPException(status_code=404, detail=f"Camera {camera_id} not found")
    
    filepath = recorder.start_recording(camera_id)
    return {"status": "recording_started", "filepath": filepath}

@app.post("/record/stop/{camera_id}")
async def stop_recording(camera_id: str):
    """Stop recording from camera."""
    filepath = recorder.stop_recording(camera_id)
    if filepath:
        return {"status": "recording_stopped", "filepath": filepath}
    else:
        raise HTTPException(status_code=404, detail=f"No active recording for {camera_id}")
```

### 4. Adding Analytics and Computer Vision

**Step 1**: Create analytics module:
```python
# video_stream/analytics.py
import cv2
import numpy as np

class VideoAnalytics:
    def __init__(self):
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.background_subtractor = cv2.createBackgroundSubtractorMOG2()
    
    def detect_faces(self, frame: np.ndarray) -> list:
        """Detect faces in frame."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
        return faces.tolist()
    
    def detect_motion(self, frame: np.ndarray) -> dict:
        """Detect motion in frame."""
        mask = self.background_subtractor.apply(frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        motion_areas = []
        for contour in contours:
            if cv2.contourArea(contour) > 500:
                x, y, w, h = cv2.boundingRect(contour)
                motion_areas.append({"x": x, "y": y, "width": w, "height": h})
        
        return {"motion_detected": len(motion_areas) > 0, "areas": motion_areas}
    
    def analyze_frame(self, frame: np.ndarray) -> dict:
        """Comprehensive frame analysis."""
        return {
            "faces": self.detect_faces(frame),
            "motion": self.detect_motion(frame),
            "timestamp": time.time()
        }
```

**Step 2**: Add analytics endpoints:
```python
# In app.py
from .analytics import VideoAnalytics

analytics = VideoAnalytics()

@app.get("/analytics/{camera_id}")
async def get_analytics(camera_id: str):
    """Get analytics for current frame."""
    if camera_id not in cameras:
        raise HTTPException(status_code=404, detail=f"Camera {camera_id} not found")
    
    camera = cameras[camera_id]
    if camera.active and camera.cap:
        ret, frame = camera.cap.read()
        if ret:
            analysis = analytics.analyze_frame(frame)
            return analysis
    
    return {"error": "Unable to analyze frame"}
```

### 5. Adding Custom Stream Formats

**Step 1**: Create format handler:
```python
# video_stream/formats.py
import cv2
import base64

class StreamFormatter:
    def __init__(self):
        self.formats = {
            "mjpeg": self.format_mjpeg,
            "base64": self.format_base64,
            "webm": self.format_webm
        }
    
    def format_mjpeg(self, frame: np.ndarray) -> bytes:
        """Format frame as MJPEG."""
        success, jpeg = cv2.imencode('.jpg', frame)
        if success:
            return (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        return b''
    
    def format_base64(self, frame: np.ndarray) -> str:
        """Format frame as base64 JSON."""
        success, jpeg = cv2.imencode('.jpg', frame)
        if success:
            b64_string = base64.b64encode(jpeg).decode('utf-8')
            return f'{{"image": "data:image/jpeg;base64,{b64_string}", "timestamp": {time.time()}}}\n'
        return ''
    
    def get_formatter(self, format_type: str):
        """Get formatter for specified format."""
        return self.formats.get(format_type, self.format_mjpeg)
```

### 6. Adding Multi-Resolution Support

**Step 1**: Create resolution manager:
```python
# video_stream/resolutions.py
class ResolutionManager:
    def __init__(self):
        self.resolutions = {
            "low": (320, 240),
            "medium": (640, 480), 
            "high": (1280, 720),
            "full": (1920, 1080)
        }
    
    def resize_frame(self, frame: np.ndarray, resolution: str) -> np.ndarray:
        """Resize frame to specified resolution."""
        if resolution in self.resolutions:
            target_size = self.resolutions[resolution]
            return cv2.resize(frame, target_size)
        return frame
    
    def get_available_resolutions(self) -> dict:
        """Get available resolution options."""
        return self.resolutions
```

**Step 2**: Add resolution endpoint:
```python
@app.get("/stream/{camera_id}/{resolution}")
async def get_stream_resolution(camera_id: str, resolution: str):
    """Stream video at specific resolution."""
    if camera_id not in cameras:
        raise HTTPException(status_code=404, detail=f"Camera {camera_id} not found")
    
    return StreamingResponse(
        gen_frames_resolution(camera_id, resolution),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )
```

## Environment Variables

```env
# Camera Configuration
DEFAULT_CAMERA_FPS=10                    # Default frame rate
MAX_CAMERAS=10                          # Maximum number of cameras
CAMERA_TIMEOUT=30                       # Camera initialization timeout

# Stream Configuration  
STREAM_QUALITY=80                       # JPEG quality (1-100)
STREAM_RESOLUTION=medium                # Default resolution
ENABLE_RECORDING=true                   # Enable recording features

# Directory Configuration
RECORDINGS_DIR=./recordings             # Recording output directory
SNAPSHOTS_DIR=./snapshots              # Snapshot output directory
```

## Development Setup

1. **Install Dependencies**:
   ```bash
   pip install fastapi uvicorn opencv-python numpy
   ```

2. **Camera Setup**:
   ```bash
   # Check available cameras (Linux)
   ls /dev/video*
   
   # Test camera access
   python -c "import cv2; cap = cv2.VideoCapture(0); print(cap.isOpened())"
   ```

3. **Run Service**:
   ```bash
   uvicorn services.video_stream.app:app --host 0.0.0.0 --port 8000 --reload
   ```

## Testing

### Manual Testing
```bash
# Test camera list
curl "http://localhost:8000/cameras"

# Test still image
curl "http://localhost:8000/still/webcam" -o test_image.jpg

# Test video stream in browser
open "http://localhost:8000/stream/webcam"
```

### Automated Testing
```bash
# Unit tests
python -m pytest services/video_stream/tests/

# Load testing
python -m pytest services/video_stream/tests/test_load.py
```

## Integration Points

### With Dashboard
- **Video Feeds**: Provides live video streams for monitoring interface
- **Still Images**: Supplies snapshot images for alerts and logging

### With Validation Service
- **Computer Vision**: Provides frames for AI-based quality control
- **Motion Detection**: Alerts for unexpected movement

### With Alert System
- **Visual Alerts**: Captures images when alerts are triggered
- **Monitoring**: Continuous surveillance of production area

## Performance Considerations

- **Frame Rate Management**: Configurable FPS to balance quality and performance
- **Memory Usage**: Efficient frame processing to prevent memory leaks
- **Concurrent Streams**: Support multiple simultaneous viewers
- **Network Bandwidth**: JPEG compression and resolution options
- **Error Recovery**: Robust handling of camera disconnections
- **Resource Cleanup**: Proper camera release on shutdown 