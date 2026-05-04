# Dobot MoveIt + Servo Action

## Overview

This service connects MoveIt motion planning to the real Dobot robot.

MoveIt plans the trajectory, and `servo_action` sends the planned joint trajectory to the real robot using the Dobot bringup `ServoJ` service.

## Prerequisites

Start Dobot bringup first:

```bash
ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py
```

Set the robot type:

```bash
export DOBOT_TYPE=nova5
```

Source the workspace:

```bash
source install/setup.bash
```

## Run Order

### 1. Start Servo Action Server

Keep this running:

```bash
ros2 run servo_action action_move_server_reality
```

This node listens to MoveIt planned paths and sends robot commands through:

```bash
/dobot_bringup_v3/srv/ServoJ
```

### 2. Sync Current Robot Joint Position

Run:

```bash
ros2 run servo_action action_move_client_reality
```

This reads the current robot joints from:

```bash
/joint_states_robot
```

and sends them to the MoveIt trajectory controller.

After launching MoveIt and confirming the robot position is synced, terminate this after about 1 second.

### 3. Launch MoveIt

Run:

```bash
ros2 launch dobot_moveit dobot_moveit.launch
```

### 4. Send MoveIt Commands to Robot

After planning in MoveIt, run:

```bash
ros2 run servo_action action_move_client_reality
```

## Main Nodes

| Node | Purpose |
|---|---|
| `action_move_server_reality` | Executes MoveIt planned trajectories on the real Dobot |
| `action_move_client_reality` | Sends current robot joint positions to the trajectory controller |
| `Joint_Position` | Debug script to print current robot joint positions |
| `dobot_moveit.launch.py` | Launches the MoveIt setup based on `DOBOT_TYPE` |

## Important Topics

```bash
/joint_states_robot
/display_planned_path
/servo_controller_status
/max_points
/sleep_timing
```

## Important Services

```bash
/dobot_bringup_v3/srv/ServoJ
/dobot_bringup_v3/srv/EnableRobot
/get_servo_status
/get_max_points
/get_sleep_timing
/emergency_reset
```

## Controller

For Nova 5, the MoveIt trajectory controller is:

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

## Debug Commands

Check robot joint states:

```bash
ros2 topic echo /joint_states_robot
```

Print joint positions:

```bash
ros2 run servo_action Joint_Position
```

Check servo status:

```bash
ros2 service call /get_servo_status std_srvs/srv/Trigger
```

Reset if stuck:

```bash
ros2 service call /emergency_reset std_srvs/srv/Trigger
```

## Handover Notes

- Dobot bringup must be running before MoveIt execution.
- `DOBOT_TYPE` must be set before launching MoveIt or `servo_action`.
- `action_move_server_reality` should stay running during robot execution.
- `action_move_client_reality` is used to sync/send joint positions.
- If MoveIt shows the wrong starting position, run `action_move_client_reality` briefly again.
- If execution gets stuck, call `/emergency_reset`.
