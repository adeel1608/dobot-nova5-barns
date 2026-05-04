# Orbbec Gemini 330 Camera

## Overview

This service launches the Orbbec Gemini 330 series camera ROS 2 driver.

It starts the Orbbec camera node under the default namespace:

```bash
/camera
```

## Launch Command

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

## Default Behavior

By default, the launch enables:

- Color stream
- Depth stream
- Depth registration
- TF publishing
- Laser
- LDP
- Soft filter
- Noise removal filter
- Frame sync

Point cloud, IR, accelerometer, and gyro are disabled by default.

## Common Launch Options

### Enable point cloud

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enable_point_cloud:=true
```

### Enable colored point cloud

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enable_point_cloud:=true enable_colored_point_cloud:=true
```

### Set camera namespace

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py camera_name:=camera
```

### Launch using a config file

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py config_file_path:=/path/to/config.yaml
```

## Important Parameters

| Parameter | Default | Description |
|---|---:|---|
| `camera_name` | `camera` | Camera namespace |
| `enable_color` | `true` | Enables RGB stream |
| `enable_depth` | `true` | Enables depth stream |
| `depth_registration` | `true` | Aligns depth with color |
| `enable_point_cloud` | `false` | Enables point cloud output |
| `enable_colored_point_cloud` | `false` | Enables RGB-colored point cloud |
| `enable_left_ir` | `false` | Enables left IR stream |
| `enable_right_ir` | `false` | Enables right IR stream |
| `enable_accel` | `false` | Enables accelerometer |
| `enable_gyro` | `false` | Enables gyroscope |
| `publish_tf` | `true` | Publishes camera TF frames |

## Useful Debug Commands

List camera topics:

```bash
ros2 topic list | grep camera
```

View color image:

```bash
ros2 topic echo /camera/color/image_raw
```

View depth image:

```bash
ros2 topic echo /camera/depth/image_raw
```

Check camera node:

```bash
ros2 node list | grep camera
```

## Handover Notes

- The default namespace is `/camera`.
- The launch file supports many runtime parameters through ROS 2 launch arguments.
- For ROS 2 Foxy, it launches `orbbec_camera_node`.
- For newer ROS 2 distros such as Humble, it launches the camera as a composable node using `rclcpp_components`.
- Use `config_file_path` if camera settings need to be managed through a YAML file.
- Enable point cloud only when needed, as it can increase CPU and bandwidth usage.
