#!/usr/bin/env bash
set -e

# -----------------------------------------------------------------------------
# Source your ROS 2 overlay
# -----------------------------------------------------------------------------
source /root/ros_ws/install/setup.bash

# -----------------------------------------------------------------------------
# Helper function to wait for a service
# -----------------------------------------------------------------------------
wait_for_service() {
  local srv_name="$1"
  echo "Waiting for service ${srv_name} ..."
  until ros2 service type "${srv_name}" > /dev/null 2>&1; do
    sleep 0.1
  done
  echo "  ↳ ${srv_name} is now available."
}

# -----------------------------------------------------------------------------
# 1) Launch dobot_bringup_v3 (logs go to console)
# -----------------------------------------------------------------------------
echo "=== Launching dobot_bringup_v3 (log‐level=warn) ==="
ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py __log_level:=warn &
DOBOT_BRINGUP_PID=$!
sleep 10

wait_for_service "/dobot_bringup_v3/srv/ClearError"
echo "Calling ClearError ..."
ros2 service call /dobot_bringup_v3/srv/ClearError dobot_msgs_v3/srv/ClearError "{}"

wait_for_service "/dobot_bringup_v3/srv/DisableRobot"
echo "Calling DisableRobot ..."
ros2 service call /dobot_bringup_v3/srv/DisableRobot dobot_msgs_v3/srv/DisableRobot "{}"

wait_for_service "/dobot_bringup_v3/srv/EnableRobot"
echo "Calling EnableRobot (load: 2.0) ..."
ros2 service call /dobot_bringup_v3/srv/EnableRobot dobot_msgs_v3/srv/EnableRobot "{load: 2.0}"

wait_for_service "/dobot_bringup_v3/srv/StartDrag"
echo "Calling StartDrag ..."
ros2 service call /dobot_bringup_v3/srv/StartDrag dobot_msgs_v3/srv/StartDrag "{}"

wait_for_service "/dobot_bringup_v3/srv/StopDrag"
echo "Calling StopDrag ..."
ros2 service call /dobot_bringup_v3/srv/StopDrag dobot_msgs_v3/srv/StopDrag "{}"

wait_for_service "/dobot_bringup_v3/srv/ModbusClose"
echo "Calling ModbusClose (index: 0) ..."
ros2 service call /dobot_bringup_v3/srv/ModbusClose dobot_msgs_v3/srv/ModbusClose "{index: 0}"

wait_for_service "/dobot_bringup_v3/srv/ModbusCreate"
echo "Calling ModbusCreate (ip: 127.0.0.1, port: 60000, slave_id: 9, is_rtu: 1) ..."
ros2 service call /dobot_bringup_v3/srv/ModbusCreate dobot_msgs_v3/srv/ModbusCreate \
  "{ip: \"127.0.0.1\", port: 60000, slave_id: 9, is_rtu: 1}"

wait_for_service "/dobot_bringup_v3/srv/SetHoldRegs"
echo "Calling SetHoldRegs (zero‐out) ..."
ros2 service call /dobot_bringup_v3/srv/SetHoldRegs dobot_msgs_v3/srv/SetHoldRegs \
  "{index: 0, addr: 1000, count: 3, val_tab: \"0,0,0\", val_type: \"int\"}"

echo "Calling SetHoldRegs (load=256) ..."
ros2 service call /dobot_bringup_v3/srv/SetHoldRegs dobot_msgs_v3/srv/SetHoldRegs \
  "{index: 0, addr: 1000, count: 3, val_tab: \"256,0,0\", val_type: \"int\"}"
sleep 10

# -----------------------------------------------------------------------------
# 2) Launch MoveIt (but redirect its output to /dev/null)
# -----------------------------------------------------------------------------
echo "=== Launching dobot_moveit (silenced) ==="
ros2 launch dobot_moveit dobot_moveit.launch.py __log_level:=fatal &> /dev/null &
MOVEIT_PID=$!
sleep 10

# -----------------------------------------------------------------------------
# 3) Launch servo_action server (logs go to console)
# -----------------------------------------------------------------------------
echo "=== Launching servo_action server (log‐level=error) ==="
ros2 run servo_action action_move_server_reality __log_level:=error &
ACTION_SERVER_PID=$!
sleep 5

# -----------------------------------------------------------------------------
# 4) Launch Orbbec camera (silence everything)
#/dev/null ensures no logs appear in this terminal.
# -----------------------------------------------------------------------------
echo "=== Launching Orbbec camera (silenced) ==="
ros2 launch orbbec_camera gemini_330_series.launch.py __log_level:=fatal &> /dev/null &
CAMERA_PID=$!
sleep 10

# -----------------------------------------------------------------------------
# 5) Launch perception nodes (pose_generator, obstacle_generator) silently
# -----------------------------------------------------------------------------
echo "=== Spinning up pose_generator (silenced) ==="
ros2 run pickn_place pose_generator __log_level:=fatal &> /dev/null &
POSE_GEN_PID=$!
sleep 5

echo "=== Spinning up obstacle_generator (silenced) ==="
ros2 run pickn_place obstacle_generator __log_level:=fatal &> /dev/null &
OBSTACLE_GEN_PID=$!
sleep 5

# -----------------------------------------------------------------------------
# 6) Finally, leave the container alive (so dobot_bringup_v3 & servo_action keep running)
# -----------------------------------------------------------------------------
echo "=== All subsystems launched. Only dobot_bringup_v3 and servo_action will print logs. ==="
echo "=== All subsystems should be ready.  (CLI has been disabled in this script.) ==="
echo "    To start the CLI, open a second shell and run:"
echo "        docker exec -it <container-name> bash"
echo "        python3 /root/ros_ws/src/oms_v1/oms_v1/cli.py"
echo "        source /root/ros_ws/install/setup.bash"
tail -f /dev/null

# -----------------------------------------------------------------------------
# When the container stops, shut down all background processes
# -----------------------------------------------------------------------------
trap "kill \
  $DOBOT_BRINGUP_PID \
  $MOVEIT_PID \
  $ACTION_SERVER_PID \
  $CAMERA_PID \
  $POSE_GEN_PID \
  $OBSTACLE_GEN_PID" EXIT