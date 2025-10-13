# Video Stream Service

## Brief Overview

The Video Stream Service provides real-time video streaming capabilities for the BARNS system. It manages multiple camera feeds, handles webcam access, and delivers MJPEG video streams over HTTP with automatic fallback to test patterns when physical cameras are unavailable.

## Key Features

- Multi-camera support with up to 4 configurable streams
- MJPEG video streaming over HTTP
- Automatic fallback to test patterns when cameras unavailable
- Still image capture from any camera
- CORS-enabled for dashboard integration
- Real-time camera status monitoring
- Debug endpoints for troubleshooting camera issues
- Graceful error handling with visual error frames

## Architecture

```
┌─────────────┐
│  Dashboard  │
└──────┬──────┘
       │ HTTP GET /stream/{id}
       ↓
┌─────────────────────────────┐
│  Video Stream Service       │
│  (FastAPI)                  │
│                             │
│  ┌─────────────────────┐   │
│  │  Camera Manager     │   │
│  │  - webcam          │   │
│  │  - test_pattern    │   │
│  │  - camera1         │   │
│  │  - camera2         │   │
│  └─────────────────────┘   │
│           │                 │
│           ↓                 │
│  ┌─────────────────────┐   │
│  │  OpenCV             │   │
│  │  Video Capture      │   │
│  └─────────────────────┘   │
└─────────────┬───────────────┘
              │
              ↓
      Physical Cameras
      (/dev/video0, etc.)
```

### Components

1. **Camera Class**: Manages individual camera lifecycle, frame capture, and test pattern generation
2. **FastAPI Application**: Provides REST API and streaming endpoints
3. **Frame Generator**: Yields MJPEG frames for continuous streaming
4. **Error Handler**: Generates visual error frames when cameras fail

## Setup & Installation

### Prerequisites

- Python 3.8+
- Physical webcam (optional, test patterns available)
- Docker (for containerized deployment)

### Local Development

```bash
# Navigate to service directory
cd services/video-stream

# Install dependencies
pip install -r requirements.txt

# Run service
uvicorn app:app --host 0.0.0.0 --port 8000 --reload
```

### Docker Deployment

The service is automatically deployed via `docker-compose.yml`:

```bash
# Start all services
docker-compose up -d

# View logs
docker-compose logs -f video-stream-service

# Access service
curl http://localhost:8001/status
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `PYTHONPATH` | `/app` | Python module search path |

### Camera Configuration

Cameras are configured in `app.py`:

```python
cameras: Dict[str, Camera] = {
    "webcam": Camera("Live Webcam", "webcam", 0, use_test_pattern=True),
    "test_pattern": Camera("Test Pattern Demo", "test_pattern", None, use_test_pattern=True),
    "camera1": Camera("Camera 1", "camera1", None),
    "camera2": Camera("Camera 2", "camera2", None),
}
```

**Camera Parameters:**
- `name`: Display name for the camera
- `camera_id`: Unique identifier for API access
- `source`: Camera index (0, 1, 2) or None for test pattern
- `use_test_pattern`: If True, shows test pattern when camera unavailable

### Docker Volume Mounts

```yaml
devices:
  - /dev/video0:/dev/video0  # Primary webcam
  - /dev/video1:/dev/video1  # Secondary webcam
privileged: true              # Required for camera access
volumes:
  - /dev:/dev                 # Device file access
```

## API/Endpoints

### GET /cameras

List all available cameras with their status.

**Response:**
```json
{
  "cameras": {
    "webcam": {
      "name": "Live Webcam",
      "status": "active",
      "stream_url": "/stream/webcam",
      "type": "real"
    },
    "test_pattern": {
      "name": "Test Pattern Demo",
      "status": "test_pattern",
      "stream_url": "/stream/test_pattern",
      "type": "test_pattern"
    }
  }
}
```

### GET /stream/{camera_id}

Stream live video from specified camera.

**Parameters:**
- `camera_id` (path): Camera identifier (webcam, test_pattern, camera1, camera2)

**Response:** MJPEG stream (`multipart/x-mixed-replace; boundary=frame`)

**Example:**
```html
<img src="http://localhost:8001/stream/webcam" />
```

### GET /still/{camera_id}

Capture a single still image from camera.

**Parameters:**
- `camera_id` (path): Camera identifier

**Response:** JPEG image

**Example:**
```bash
curl http://localhost:8001/still/webcam -o snapshot.jpg
```

### GET /status

Get service health and camera status.

**Response:**
```json
{
  "status": "operational",
  "cameras": {
    "webcam": {
      "name": "Live Webcam",
      "active": true,
      "type": "real"
    }
  },
  "message": "Video streaming service with test patterns for development"
}
```

### GET /debug

Detailed debug information for troubleshooting camera issues.

**Response:**
```json
{
  "video_devices": ["/dev/video0", "/dev/video1"],
  "opencv_version": "4.8.1",
  "camera_tests": {
    "camera_0": {
      "available": true,
      "can_read": true,
      "frame_shape": [480, 640, 3]
    }
  },
  "system_info": {
    "/dev/video0": "readable"
  }
}
```

## Usage Examples

### Dashboard Integration

```javascript
// React component
<img 
  src="http://video-stream-service:8000/stream/webcam" 
  alt="Live Camera Feed"
  style={{ width: '100%', height: 'auto' }}
/>
```

### Python Client

```python
import requests

# Get camera list
response = requests.get("http://localhost:8001/cameras")
cameras = response.json()["cameras"]

# Capture still image
response = requests.get("http://localhost:8001/still/webcam")
with open("snapshot.jpg", "wb") as f:
    f.write(response.content)

# Check service status
response = requests.get("http://localhost:8001/status")
print(response.json())
```

### Testing Camera Access

```bash
# List cameras
curl http://localhost:8001/cameras | jq

# Test video stream
curl http://localhost:8001/stream/test_pattern

# Debug camera issues
curl http://localhost:8001/debug | jq
```

## Dependencies

### Core Dependencies

- **FastAPI** (0.104.1): Web framework for API endpoints
- **Uvicorn** (0.24.0): ASGI server for FastAPI
- **OpenCV** (4.8.1.78): Video capture and image processing
- **NumPy** (2.2.6): Array operations for image data

### Optional Dependencies

- **PyTorch** (2.8.0): Deep learning framework (for future CV features)
- **Ultralytics** (8.3.203): YOLO object detection (for future features)

### System Requirements

- Linux: `/dev/video*` device access
- Windows: DirectShow compatible webcams
- macOS: AVFoundation compatible cameras

## Integration Points

### Upstream Services (Consumers)

1. **Dashboard Service**
   - Embeds video streams in monitoring UI
   - Displays camera feeds on main dashboard
   - Uses still images for thumbnails

### Communication Pattern

- **Protocol**: HTTP/REST
- **Port**: 8001 (external), 8000 (internal)
- **Type**: Synchronous request/response
- **Format**: MJPEG streams, JPEG images, JSON metadata

### Network Configuration

```yaml
networks:
  - barns-network
ports:
  - "8001:8000"  # Host:Container mapping
```

## Troubleshooting

### Camera Not Detected

**Issue**: Camera shows error frame instead of live video

**Solutions:**
1. Check camera permissions:
   ```bash
   ls -l /dev/video*
   sudo chmod 666 /dev/video0
   ```

2. Verify camera in Docker:
   ```yaml
   devices:
     - /dev/video0:/dev/video0
   privileged: true
   ```

3. Test camera manually:
   ```bash
   docker exec -it barns-video-stream python -c "import cv2; print(cv2.VideoCapture(0).isOpened())"
   ```

### Test Pattern Displayed Instead of Camera

**Expected behavior** when:
- Running in Docker on Windows (camera passthrough limitations)
- Physical camera not connected
- Camera in use by another application

Use `/debug` endpoint to diagnose camera availability.

### Low Frame Rate

**Issue**: Video stream appears choppy

**Solutions:**
1. Reduce number of concurrent streams
2. Lower resolution in Camera initialization:
   ```python
   self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
   self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
   ```
3. Increase sleep interval in `gen_frames()`:
   ```python
   time.sleep(0.2)  # Reduce to 5 FPS
   ```

### CORS Errors

**Issue**: Dashboard cannot load video streams

**Solution**: Add dashboard origin to CORS middleware:
```python
allow_origins=[
    "http://localhost:3000",
    "http://your-dashboard-domain.com"
]
```

## Performance Considerations

- **FPS**: Default 10 FPS per stream (configurable)
- **Resolution**: 640x480 (configurable)
- **Encoding**: JPEG compression for bandwidth efficiency
- **CPU Usage**: ~5-10% per active camera stream
- **Memory**: ~50MB per service instance

## Security Notes

- Service runs in privileged Docker mode for device access
- No authentication on endpoints (internal network only)
- CORS restricted to known dashboard origins
- No data persistence or logging of video content

## Future Enhancements

- Motion detection alerts
- Recording and playback functionality
- RTSP stream support
- Multi-camera synchronized capture
- Hardware-accelerated encoding
- Authentication and authorization
