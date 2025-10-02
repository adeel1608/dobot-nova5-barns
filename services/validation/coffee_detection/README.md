# Coffee Detection System

A production-ready coffee detection system that monitors a camera feed and detects coffee presence using computer vision techniques.

## Features

- **Real-time coffee detection** from camera feed
- **Configurable ROI (Region of Interest)** selection
- **Production-ready** with proper error handling and logging
- **Interactive ROI selector** tool for easy setup
- **Configurable parameters** via JSON configuration

## Files

### Core Production Files
- `camera_worker_production.py` - Main detection engine
- `production_usage_example.py` - Example usage script
- `detection_config.json` - Configuration file
- `roi_selector.py` - Interactive ROI selection tool

### Generated Files
- `coffee_detection.log` - Error logs
- `debug_frames/` - Debug images (when enabled)

## Installation

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

2. Configure your camera settings in `detection_config.json`

## Configuration

Edit `detection_config.json` to configure:

- **Camera settings**: URL, timeouts, retry logic
- **Detection parameters**: Threshold, frame sampling
- **ROI coordinates**: Region of interest points
- **Debug options**: Enable/disable debug output

### Example Configuration
```json
{
  "snapshot_url": "http://username:password@192.168.1.100/cgi-bin/snapshot.cgi",
  "frame_width": 1920,
  "frame_height": 1080,
  "coffee_threshold": 5.0,
  "roi_points": [
    [121, 481],
    [953, 411],
    [1873, 420],
    [1867, 712]
  ]
}
```

## Usage

### 1. Setup ROI (Region of Interest)

First, select the area where coffee detection should occur:

```bash
python roi_selector.py
```

**Instructions:**
- Click on the camera image to add ROI points
- Press `r` to reset all points
- Press `s` to save points to config
- Press `q` to quit without saving
- You need at least 3 points to form a polygon

### 2. Run Detection

Run the coffee detection system:

```bash
python production_usage_example.py
```

This will:
- Load configuration from `detection_config.json`
- Connect to the camera
- Run detection every 10 minutes (configurable)
- Log results and errors

## Detection Algorithm

The system uses the following approach:

1. **Capture frames** from the camera
2. **Apply ROI mask** to focus on the region of interest
3. **Color filtering** to identify coffee-colored pixels
4. **Morphological operations** to clean up the mask
5. **Threshold comparison** to determine coffee presence

## Configuration Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `snapshot_url` | Camera snapshot URL | Required |
| `frame_width` | Frame width | 1920 |
| `frame_height` | Frame height | 1080 |
| `coffee_threshold` | Detection threshold | 5.0 |
| `roi_points` | ROI polygon points | Required |
| `timeout` | Request timeout | 1.0s |
| `max_retries` | Max retry attempts | 3 |
| `enable_debug` | Enable debug output | true |

## Troubleshooting

### Camera Connection Issues
- Verify camera URL and credentials
- Check network connectivity
- Adjust timeout values in config

### Detection Issues
- Use ROI selector to adjust detection area
- Adjust `coffee_threshold` value
- Enable debug mode to see detection masks

### Performance Issues
- Reduce `frames_to_sample` value
- Increase `target_fps` for faster processing
- Disable debug mode in production

## Logs

Error logs are written to `coffee_detection.log`. Check this file for:
- Camera connection errors
- Detection failures
- System errors

## Expected Returns

### Detection Results

The `detect_coffee()` method returns a dictionary with the following structure:

```python
{
    "coffee": bool,        # True if coffee is detected, False otherwise
    "percentage": float,   # Coffee coverage percentage (0.0-100.0)
    "error": str | None    # Error message if detection failed, None if successful
}
```

**Example successful detection:**
```python
{
    "coffee": True,
    "percentage": 15.67,
    "error": None
}
```

**Example failed detection:**
```python
{
    "coffee": False,
    "percentage": 0.0,
    "error": "Camera request failed: Connection timeout"
}
```

### Health Check Results

The `health_check()` method returns a dictionary with system status:

```python
{
    "status": str,              # "healthy" or "unhealthy"
    "camera_accessible": bool,  # True if camera is reachable
    "roi_initialized": bool,    # True if ROI mask is ready
    "health_check_time": float, # Time taken for health check (seconds)
    "timestamp": float,         # Unix timestamp of check
    "error": str | None         # Error message if unhealthy, None if healthy
}
```

**Example healthy system:**
```python
{
    "status": "healthy",
    "camera_accessible": True,
    "roi_initialized": True,
    "health_check_time": 0.234,
    "timestamp": 1703123456.789,
    "error": None
}
```

**Example unhealthy system:**
```python
{
    "status": "unhealthy",
    "camera_accessible": False,
    "roi_initialized": False,
    "health_check_time": 0.0,
    "timestamp": 1703123456.789,
    "error": "Camera request failed: Connection refused"
}
```

### Console Output

When running the production example, you'll see:

```
{'coffee': True, 'percentage': 12.45, 'error': None}
COFFEE DETECTED!
Detection time: 1.234s
```

Or for no coffee detected:
```
{'coffee': False, 'percentage': 2.1, 'error': None}
Detection time: 0.987s
```

### Debug Output

When `enable_debug: true` in configuration, the system generates:

- **Debug frames directory**: `debug_frames/`
- **ROI mask**: `roi_mask.png` - Shows the region of interest
- **Per-frame outputs** for each sampled frame:
  - `frame_XXXX_scaled_roi.jpg` - Original frame with ROI overlay
  - `frame_XXXX_coffee_mask.png` - Raw coffee color detection mask
  - `frame_XXXX_final_mask.png` - Final mask after ROI application
  - `frame_XXXX_overlay.jpg` - Color-coded overlay showing detected areas

### Log Files

Error logs are written to `coffee_detection.log` with format:
```
2024-01-15 10:30:45,123 - __main__ - ERROR - Camera request failed: Connection timeout
2024-01-15 10:30:45,124 - __main__ - ERROR - Detection failed (ID: 1703123456123): Camera request failed
```

### Return Value Interpretation

- **`coffee: true`**: Coffee is present above the threshold
- **`coffee: false`**: No coffee detected or below threshold
- **`percentage`**: Coffee coverage as percentage of ROI area (0.0-100.0)
- **`error`**: Indicates system failure (camera issues, processing errors, etc.)

### Threshold Behavior

- Coffee is considered "detected" when `percentage > coffee_threshold`
- Default threshold is `5.0` (5% of ROI area)
- Threshold can be adjusted in `detection_config.json`

## Development

The system is designed for production use with:
- Proper error handling and logging
- Resource management and cleanup
- Configurable parameters
- Thread-safe operations

For development or testing, you can modify the detection parameters in the code or configuration file.