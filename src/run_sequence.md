# Project Launch Handover

## Overview

This project starts the full Dobot + MoveIt + Orbbec + ArUco perception pipeline.

The system includes:

- Dobot robot bringup
- MoveIt planning interface
- Servo action bridge for real robot execution
- Orbbec Gemini 330 camera
- ArUco marker perception and TF publishing

---

## Required Environment Variables

Set these before launching the system:

```bash
export IP_address=<ROBOT_IP_ADDRESS>
export DOBOT_TYPE=nova5
source install/setup.bash
```

`IP_address` is used by Dobot bringup.

`DOBOT_TYPE` is used by MoveIt and `servo_action` to generate the correct controller names.

For the current setup:

```bash
export DOBOT_TYPE=nova5
```

---

## Full Launch Sequence

The normal launch sequence is:

1. Kill any old running nodes
2. Launch Dobot bringup
3. Start `action_move_client_reality` briefly to sync robot joint position
4. Launch MoveIt
5. Kill the temporary action client
6. Start `action_move_server_reality`
7. Launch Orbbec camera
8. Launch ArUco perception

---

## Main Startup Script

Use this script to restart the full system:

```bash
#!/bin/bash
# Single script to kill all nodes and restart them in separate terminals

echo "Killing any existing nodes..."
pkill -f "ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py"
pkill -f "ros2 run servo_action action_move_client_reality"
pkill -f "ros2 launch dobot_moveit dobot_moveit.launch.py"
pkill -f "ros2 run servo_action action_move_server_reality"
pkill -f "ros2 launch orbbec_camera gemini_330_series.launch.py"
pkill -f "ros2 run pickn_place aruco_perception"

echo "Waiting for processes to terminate..."
sleep 1.0

# Step 1: Launch dobot bringup
echo "Launching dobot bringup..."
gnome-terminal -- bash -c "echo 'Launching dobot bringup...'; ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py; exec bash"
sleep 1.0

# Step 2: Run servo action client
echo "Starting servo action client..."
gnome-terminal -- bash -c "echo 'Starting servo action client...'; ros2 run servo_action action_move_client_reality; exec bash"
sleep 1.0

# Step 3: Launch dobot_moveit
echo "Launching dobot_moveit..."
gnome-terminal -- bash -c "echo 'Launching dobot_moveit...'; ros2 launch dobot_moveit dobot_moveit.launch.py; exec bash"
sleep 1.0

# Step 4: Terminate servo action client and start server
echo "Terminating servo action client..."
pkill -f "ros2 run servo_action action_move_client_reality"
sleep 1.0

echo "Starting servo action server..."
gnome-terminal -- bash -c "echo 'Starting servo action server...'; ros2 run servo_action action_move_server_reality; exec bash"
sleep 1.0

# Step 5: Launch the camera
echo "Launching gemini 335..."
gnome-terminal -- bash -c "echo 'Launching gemini 335...'; ros2 launch orbbec_camera gemini_330_series.launch.py; exec bash"
sleep 10.0

# Step 6: Launch the pose generator
echo "running pose generator..."
gnome-terminal -- bash -c "echo 'running aruco_perception...'; ros2 run pickn_place aruco_perception; exec bash"
```

---

## Components

### 1. Dobot Bringup

Command:

```bash
ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py
```

Purpose:

Starts the Dobot ROS 2 interface.

It launches:

- Dobot command/service node
- Dobot feedback node

Important outputs:

```bash
/joint_states_robot
/dobot_bringup_v3/srv/ServoJ
/dobot_bringup_v3/srv/EnableRobot
```

The feedback node publishes the real robot joint states to:

```bash
/joint_states_robot
```

---

### 2. MoveIt + Servo Action

MoveIt launch command:

```bash
ros2 launch dobot_moveit dobot_moveit.launch.py
```

Temporary joint sync command:

```bash
ros2 run servo_action action_move_client_reality
```

Real execution server:

```bash
ros2 run servo_action action_move_server_reality
```

Purpose:

- `action_move_client_reality` sends the current real robot joint state to the MoveIt controller.
- `action_move_server_reality` bridges MoveIt planned trajectories to the real Dobot using `ServoJ`.

Controller for Nova 5:

```bash
nova5_group_controller/follow_joint_trajectory
```

Controlled joints:

```bash
joint1
joint2
joint3
joint4
joint5
joint6
```

Important topics:

```bash
/joint_states_robot
/display_planned_path
/servo_controller_status
/max_points
/sleep_timing
```

Important services:

```bash
/get_servo_status
/get_max_points
/get_sleep_timing
/emergency_reset
```

---

### 3. Orbbec Gemini 330 Camera

Command:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

Purpose:

Starts the Orbbec Gemini camera driver.

Default namespace:

```bash
/camera
```

Important topics:

```bash
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_raw
/camera/depth/camera_info
/camera/depth_to_color
```

Useful options:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enable_point_cloud:=true
```

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enable_point_cloud:=true enable_colored_point_cloud:=true
```

---

### 4. ArUco Perception

Command:

```bash
ros2 run pickn_place aruco_perception
```

Purpose:

Detects ArUco markers using the Orbbec RGB/depth streams and publishes marker poses as TF frames.

Required config files inside `pickn_place`:

```text
axab_calibration.yaml
arucoID_name_config.yaml
```

TF frames published:

```text
Link6 -> calibrated_camera_link
calibrated_camera_link -> <aruco_marker_name>
```

Default ArUco dictionary:

```text
DICT_5X5_50
```

Marker size:

```text
100 mm
```

Optional visualization:

```bash
ros2 run pickn_place aruco_perception --ros-args -p visualize:=true
```

---

## Useful Debug Commands

Check Dobot joint states:

```bash
ros2 topic echo /joint_states_robot
```

Print current robot joint positions:

```bash
ros2 run servo_action Joint_Position
```

Check Servo Action status:

```bash
ros2 service call /get_servo_status std_srvs/srv/Trigger
```

Reset Servo Action if stuck:

```bash
ros2 service call /emergency_reset std_srvs/srv/Trigger
```

Check Orbbec camera topics:

```bash
ros2 topic list | grep camera
```

Check camera node:

```bash
ros2 node list | grep camera
```

Check TF frames:

```bash
ros2 run tf2_tools view_frames
```

---

## Handover Notes

- Always start Dobot bringup before MoveIt or Servo Action.
- `action_move_client_reality` is used briefly during startup to sync the current robot joint state.
- After MoveIt launches, kill `action_move_client_reality`.
- Keep `action_move_server_reality` running during real robot execution.
- Start the Orbbec camera before starting ArUco perception.
- ArUco perception waits for RGB image, RGB camera info, depth image, and depth camera info before publishing marker TFs.
- If MoveIt shows the wrong starting position, rerun `action_move_client_reality` briefly.
- If Servo Action gets stuck, call `/emergency_reset`.
- Camera startup can take a few seconds, so the script waits `10s` before starting ArUco perception.
