# Video Stream Service

## Brief Overview

The Video Stream Service provides on-demand MJPEG video streaming over HTTP for the BARNS system. It connects to real camera sources (typically RTSP), captures frames continuously in the background, and serves the latest frames to clients.

## Key Features

- On-demand MJPEG video streaming over HTTP
- Still image capture per camera
- Per-camera stream session tracking (multiple viewers)
- Cooldown controls to avoid rapid reconnect churn
- RTSP latency/buffering optimizations via OpenCV + FFMPEG backend
- CORS allowlist for dashboard integration
- Status and debug endpoints for troubleshooting

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
│  │  - ceiling (RTSP)  │   │
│  │  - add more...     │   │
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
      RTSP Cameras (primary)
      or local camera devices (optional)
```

### Components

1. **Camera Class**: Manages camera lifecycle, background frame capture, and reconnection
2. **StreamManager**: Tracks active stream sessions and per-camera cooldown windows
3. **FastAPI Application**: Exposes REST endpoints and MJPEG streaming endpoint
4. **Frame Generator**: Yields multipart MJPEG frames to clients while a stream session is active

## Setup & Installation

### Prerequisites

- Python 3.10+
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

The container runs Uvicorn on port `8000`. Host port mapping depends on your Compose / Kubernetes configuration.

```bash
# Access service (adjust host/port to your deployment)
curl http://localhost:8000/status
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `PYTHONPATH` | `/app` | Python module search path |

### Camera Configuration

Cameras are configured in `app.py`:

```python
RTSP_URL = "rtsp://<user>:<pass>@<camera-host>:554/<path>"

cameras: Dict[str, Camera] = {
    "ceiling": Camera("Ceiling Camera", "ceiling", RTSP_URL),
    # Add more real cameras here as needed
}
```

**Camera Parameters:**
- `name`: Display name for the camera
- `camera_id`: Unique identifier for API access
- `source`: RTSP URL (recommended) or an OpenCV-supported local camera source (device index/path)

### Docker Volume Mounts

Only required if you use local camera devices (for example `/dev/video0`) instead of RTSP sources.

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
    "ceiling": {
      "name": "Ceiling Camera",
      "status": "active",
      "stream_url": "/stream/ceiling",
      "type": "RTSP Stream",
      "streaming": false,
      "cooldown": {
        "in_cooldown": false,
        "remaining_seconds": 0
      }
    }
  }
}
```

### GET /stream/{camera_id}

Stream live video from specified camera.

**Parameters:**
- `camera_id` (path): Camera identifier (for example `ceiling`)

**Response:** MJPEG stream (`multipart/x-mixed-replace; boundary=frame`)

**Errors:**
- `404`: Camera not found
- `503`: Camera cooling down (includes `Retry-After` header)
- `503`: Camera offline (includes `Retry-After: 1` header)

**Example:**
```html
<img src="http://localhost:8000/stream/ceiling" />
```

### GET /still/{camera_id}

Capture a single still image from camera.

**Parameters:**
- `camera_id` (path): Camera identifier

**Response:** JPEG image

**Example:**
```bash
curl http://localhost:8000/still/ceiling -o snapshot.jpg
```

### POST /stream/start/{camera_id}

Register a logical stream session for a camera (intended for pre-warming).

**Response (shape):**
```json
{ "status": "success", "message": "Stream started for ceiling", "camera": "ceiling" }
```

### POST /stream/stop/{camera_id}

Stop the logical stream session for a camera and apply a cooldown window.

**Response (shape):**
```json
{ "status": "success", "message": "Stream stopped for ceiling", "camera": "ceiling" }
```

### POST /stream/stop-all

Stop all active streams across all cameras and apply cooldown to each.

**Response (shape):**
```json
{
  "status": "success",
  "message": "All streams stopped",
  "stopped_cameras": ["ceiling"],
  "cooldown_seconds": 2.0,
  "ready_at": 1730000000.0
}
```

### GET /stream/active

List currently active streams and active session counts per camera.

**Response (shape):**
```json
{
  "status": "success",
  "active_camera_count": 1,
  "active_cameras": ["ceiling"],
  "details": { "ceiling": { "camera_name": "Ceiling Camera", "active_sessions": 2 } }
}
```

### GET /stream/cooldown/all

Get cooldown status for all cameras.

**Response (shape):**
```json
{ "status": "success", "cooldowns": { "ceiling": { "in_cooldown": false, "remaining_seconds": 0 } } }
```

### GET /stream/cooldown/{camera_id}

Get cooldown status for a specific camera.

**Response (shape):**
```json
{ "status": "success", "camera_id": "ceiling", "cooldown": { "in_cooldown": false, "remaining_seconds": 0 } }
```

### GET /status

Get service health and camera status.

**Response:**
```json
{
  "status": "operational",
  "cameras": {
    "ceiling": {
      "name": "Ceiling Camera",
      "active": true,
      "type": "RTSP Stream",
      "streaming": false
    }
  },
  "active_streams": 0,
  "message": "Video streaming service with on-demand streaming"
}
```

### GET /debug

Detailed debug information for troubleshooting OpenCV and camera availability. This endpoint includes OpenCV build information and probes local camera indices (0..2).

**Response:**
```json
{
  "video_devices": ["/dev/video0"],
  "opencv_version": "4.8.1",
  "camera_tests": { "camera_0": { "available": true, "can_read": true, "frame_shape": [480, 640, 3] } },
  "system_info": { "/dev/video0": "readable" }
}
```

## Usage Examples

### Dashboard Integration

```javascript
// React component
<img 
  src="http://video-stream-service:8000/stream/ceiling" 
  alt="Ceiling Camera Feed"
  style={{ width: '100%', height: 'auto' }}
/>
```

### Python Client

```python
import requests

# Get camera list
response = requests.get("http://localhost:8000/cameras")
cameras = response.json()["cameras"]

# Capture still image
response = requests.get("http://localhost:8000/still/ceiling")
with open("snapshot.jpg", "wb") as f:
    f.write(response.content)

# Check service status
response = requests.get("http://localhost:8000/status")
print(response.json())
```

### Testing Camera Access

```bash
# List cameras
curl http://localhost:8000/cameras | jq

# Test video stream
curl http://localhost:8000/stream/ceiling

# Debug camera issues
curl http://localhost:8000/debug | jq

# Stop all streams (applies cooldown)
curl -X POST http://localhost:8000/stream/stop-all | jq

# Check cooldown status
curl http://localhost:8000/stream/cooldown/all | jq
```

## Dependencies

### Core Dependencies

- **FastAPI** (0.104.1): Web framework for API endpoints
- **Uvicorn** (0.24.0): ASGI server for FastAPI
- **OpenCV** (4.8.1.78): Video capture and image processing
- **NumPy** (`>=1.21.0,<2.0.0`): Array operations for image data
- **Requests** (2.31.0): HTTP client (used by example tooling and available for future integrations)
- **Urllib3** (2.0.0): HTTP client dependency

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

### Downstream Dependencies (Outbound)

1. **RTSP Camera Endpoints**
   - The service opens RTSP connections defined in `app.py` (camera `source` values that start with `rtsp://`)
   - No other outbound service calls are made in the current implementation

### Communication Pattern

- **Protocol**: HTTP/REST
- **Port**: 8000 (container). Host/service port depends on deployment mapping.
- **Type**: Synchronous request/response
- **Format**: MJPEG streams, JPEG images, JSON metadata

### Network Configuration

Example Compose mapping (adjust as needed):

```yaml
networks:
  - barns-network
ports:
  - "8001:8000"  # Host:Container mapping (example)
```

## Troubleshooting

### GET `/stream/{camera_id}` returns 503 (cooldown)

This can happen after explicit stop operations (for example `POST /stream/stop-all` or `POST /stream/stop/{camera_id}`).

- Check cooldown: `GET /stream/cooldown/{camera_id}` or `GET /stream/cooldown/all`
- Respect the `Retry-After` response header in clients before retrying

### GET `/stream/{camera_id}` returns 503 (offline)

- Verify the camera is reachable from the service network (RTSP host, port, routing)
- Confirm RTSP credentials and path
- Use `GET /still/{camera_id}` to quickly validate frame capture
- Check `GET /status` and container logs for reconnect attempts

### RTSP stalls / high latency

The service reduces buffering and will force reconnect if frames become stale. If you still see stalls:

- Ensure the RTSP stream is stable (camera CPU/bitrate, network)
- Reduce camera-side bitrate/FPS if available
- Avoid many concurrent viewers per camera

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

- **Capture rate**: ~15 FPS target per camera (implementation detail)
- **Resolution**: 640x480 target (implementation detail)
- **Encoding**: JPEG (quality 80) for bandwidth efficiency
- **CPU usage**: depends heavily on codec, resolution, and number of concurrent viewers
- **Memory**: bounded (stores latest JPEG per camera)

## Security Notes

- No authentication on endpoints (assume internal network). If exposed beyond a trusted segment, add auth at an API gateway or inside the service.
- CORS is allowlisted, but it is not a security boundary; it only controls browser access.
- RTSP credentials should not be committed. The current implementation configures the RTSP URL in code (`app.py`); consider moving this to environment variables and secret management.

## Future Enhancements

- Motion detection alerts
- Recording and playback functionality
- Environment-based camera configuration (remove hard-coded RTSP URLs)
- Multi-camera synchronized capture
- Hardware-accelerated encoding
- Authentication and authorization
