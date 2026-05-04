# Teaching Offsets

## Overview

This package provides two teaching workflows:

1. **Tool Teach**
2. **Machine Teach**

Both workflows use TF frames generated from ArUco perception and save the trained offsets into YAML files for later execution.

---

## Prerequisites

Before teaching, make sure the full system is running:

```bash
ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py
ros2 launch dobot_moveit dobot_moveit.launch.py
ros2 run servo_action action_move_server_reality
ros2 launch orbbec_camera gemini_330_series.launch.py
ros2 run pickn_place aruco_perception
```

The ArUco marker must be visible to the camera, and the required TF frames must be available.

Useful TF check:

```bash
ros2 run tf2_tools view_frames
```

---

# Tool Teach

## Purpose

Tool Teach is used to train a tool pickup pose.

It maps the selected tool ArUco frame to:

```text
approach_pose
grab_pose
```

These offsets are saved and later used when the robot needs to approach and grab that tool.

## Run Command

```bash
ros2 run pickn_place tool_mount_teach
```

## Teaching Steps

1. Mount or grab the tool with the robot.
2. Make sure the tool ArUco marker is clearly visible.
3. Use the ArUco perception GUI to confirm the marker is being detected.
4. Find the marker name from `arucoID_name_config.yaml`.
5. Run:

```bash
ros2 run pickn_place tool_mount_teach
```

6. When prompted, enter the tool TF name.

Example:

```text
Enter tool tf name: single_portafilter
```

7. The script samples and saves:
   - `approach_pose`
   - `grab_pose`

## Required TF Frames

Tool Teach requires:

```text
Link6
tool_link
calibrated_camera_link
<tool_marker_name>
```

## Output File

The trained tool offsets are saved to:

```text
tool_offset_points.yaml
```

The file is written to both:

```text
~/barns_ws/src/pickn_place/share/tool_offset_points.yaml
install/pickn_place/share/pickn_place/tool_offset_points.yaml
```

## Output Format

Example structure:

```yaml
single_portafilter:
  approach_pose:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0
  grab_pose:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0
```

## Notes

- The tool marker name must match the name published by ArUco perception.
- The marker must stay visible during sampling.
- The script averages multiple TF samples before saving.
- The script applies a small fixed offset to the approach pose.

---

# Machine Teach

## Purpose

Machine Teach is used to train poses relative to a machine or station marker.

It creates a saved relation between:

```text
base_link
<machine_marker>
```

Then it records named point offsets under that machine.

Each point has:

```text
approach_pose
mount_pose
```

These are used later during execution.

## Run Command

```bash
ros2 run pickn_place machine_mount_teach
```

## Teaching Steps

1. Move the robot close enough so the machine marker is visible.
2. Confirm the machine marker TF is being published by ArUco perception.
3. Run:

```bash
ros2 run pickn_place machine_mount_teach
```

4. Enter the machine TF name.

Example:

```text
Enter machine TF name: espresso_grinder
```

5. Move the robot to the desired approach pose.
6. Enter a point name.

Example:

```text
Enter point name: grinder
```

7. When prompted for the approach TCP frame, enter:

```text
Link6
```

8. Move the robot to the desired mount pose.
9. When prompted for the mount TCP frame, enter:

```text
Link6
```

10. Repeat for more points if needed.
11. Press Enter on a blank point name to finish and save.

## No Separate Approach Case

If there is no separate approach pose, teach the same desired pose twice:

```text
approach_pose = same point
mount_pose = same point
```

## More Than Two Points

If a machine needs more than two poses, create multiple point names.

Example:

```text
grinder
angled_grinder
tamper
angled_tamper
```

During execution, call the required point by name.

## Required TF Frames

Machine Teach requires:

```text
base_link
Link6
<machine_marker_name>
```

## Output File

The trained machine offsets are saved to:

```text
machine_offset_points.yaml
```

The file is written to both:

```text
~/barns_ws/src/pickn_place/share/machine_offset_points.yaml
install/pickn_place/share/pickn_place/machine_offset_points.yaml
```

## Output Format

Example structure:

```yaml
espresso_grinder:
  grinder:
    approach_pose:
      TCP: Link6
      translation:
        x: 0.0
        y: 0.0
        z: 0.0
      rotation:
        w: 1.0
        x: 0.0
        y: 0.0
        z: 0.0
    mount_pose:
      TCP: Link6
      translation:
        x: 0.0
        y: 0.0
        z: 0.0
      rotation:
        w: 1.0
        x: 0.0
        y: 0.0
        z: 0.0
```

## Notes

- The machine marker must stay visible while the machine frame is sampled.
- The script samples the marker relative to `base_link`.
- The robot TCP pose is recorded relative to the saved machine frame.
- The script averages marker samples and filters translation outliers.
- Some targets use tighter stability thresholds for higher accuracy.
- Existing YAML entries are preserved; new points are merged into the file.

---

# ArUco Name Mapping

Marker names come from:

```text
arucoID_name_config.yaml
```

Example:

```yaml
aruco_id:
  - id: 11
    name: single_portafilter
  - id: 12
    name: double_portafilter
  - id: 31
    name: espresso_grinder
  - id: 41
    name: three_group_espresso
```

Use the `name`, not the numeric ID, when teaching.

---

# Debug Commands

Check TF frames:

```bash
ros2 run tf2_tools view_frames
```

Check live robot joint states:

```bash
ros2 topic echo /joint_states_robot
```

Check camera topics:

```bash
ros2 topic list | grep camera
```

Run ArUco perception with visualization:

```bash
ros2 run pickn_place aruco_perception --ros-args -p visualize:=true
```

---

# Handover Notes

- Use **Tool Teach** for tools that the robot needs to approach and grab.
- Use **Machine Teach** for machine/station targets where the robot needs approach and mount poses.
- Always use the ArUco marker name from `arucoID_name_config.yaml`.
- Keep the marker visible and stable while teaching.
- For machine teaching, use `Link6` unless a different TCP frame is intentionally required.
- For points with no separate approach, teach the same pose for both `approach_pose` and `mount_pose`.
- For machines with multiple required poses, create multiple point names and call the required one during execution.
