# ArUco Perception

## Overview

This node detects ArUco markers from the Orbbec camera RGB/depth streams and publishes each detected marker as a TF frame.

It uses RGB images for marker detection, depth images for 3D position estimation, and a sliding-window average to smooth marker poses.

## Run Command

```bash
ros2 run pickn_place aruco_perception
```

## Prerequisites

Start the Orbbec camera before running this node:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

The node expects these camera topics:

```bash
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_raw
/camera/depth/camera_info
/camera/depth_to_color
```

## Required Config Files

The node loads these files from the `pickn_place` package:

```text
axab_calibration.yaml
arucoID_name_config.yaml
```

### `axab_calibration.yaml`

Used to publish the static calibration TF:

```text
Link6 -> calibrated_camera_link
```

### `arucoID_name_config.yaml`

Maps ArUco marker IDs to readable TF frame names.

If a marker ID is not found in the config, the frame name defaults to:

```text
ID_<marker_id>
```

## TF Frames

The node publishes:

```text
Link6 -> calibrated_camera_link
calibrated_camera_link -> <aruco_marker_name>
```

The calibration TF is published at 100 Hz.

Marker TFs are published after enough samples are collected for averaging.

## Parameters

| Parameter | Default | Description |
|---|---:|---|
| `visualize` | `false` | Shows RGB/depth visualization window |
| `sample_window_size` | `6` | Number of marker samples used for smoothing |

Example with visualization enabled:

```bash
ros2 run pickn_place aruco_perception --ros-args -p visualize:=true
```

Example with a smaller averaging window:

```bash
ros2 run pickn_place aruco_perception --ros-args -p sample_window_size:=3
```

## ArUco Settings

The node uses:

```text
DICT_5X5_50
```

Marker size is fixed to:

```text
100 mm
```

## Useful Debug Commands

Check camera topics:

```bash
ros2 topic list | grep camera
```

Check TF output:

```bash
ros2 run tf2_tools view_frames
```

Echo camera image topic:

```bash
ros2 topic echo /camera/color/image_raw
```

Echo depth topic:

```bash
ros2 topic echo /camera/depth/image_raw
```

## Handover Notes

- The Orbbec camera must be running before this node.
- The node waits for RGB image, RGB camera info, depth image, and depth camera info before processing detections.
- Marker poses are smoothed using a sliding-window average.
- Marker transforms are published relative to `calibrated_camera_link`.
- `calibrated_camera_link` is published relative to `Link6` using the hand-eye calibration file.
- Enable `visualize:=true` only when debugging, as it opens an OpenCV display window.
