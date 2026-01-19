#!/bin/bash
# Single script to kill all nodes and restart them in separate terminals

echo "Killing any existing nodes..."
pkill -f "ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py"
pkill -f "ros2 run servo_action action_move_client_reality"
pkill -f "ros2 launch dobot_moveit dobot_moveit.launch.py"
pkill -f "ros2 run servo_action action_move_server_reality"
pkill -f "ros2 launch orbbec_camera gemini_330_series.launch.py"
pkill -f "ros2 run pickn_place pose_generator"
pkill -f "ros2 run pickn_place obstacle_generator"

echo "Waiting for processes to terminate..."
sleep 1.5

# Step 1: Launch dobot bringup
echo "Launching dobot bringup..."
gnome-terminal -- bash -c "echo 'Launching dobot bringup...'; ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py; exec bash"
sleep 1.5  # Adjust the sleep if needed for initialization
echo "Calling initialization services..."
gnome-terminal -- bash -c "
  echo 'Calling ClearError...'; ros2 service call /dobot_bringup_v3/srv/ClearError dobot_msgs_v3/srv/ClearError '{}';
  echo 'Disabling Robot...'; ros2 service call /dobot_bringup_v3/srv/DisableRobot dobot_msgs_v3/srv/DisableRobot '{}';
  echo 'Enabling Robot...'; ros2 service call /dobot_bringup_v3/srv/EnableRobot dobot_msgs_v3/srv/EnableRobot '{load: 2.0}';
  echo 'Starting Drag...'; ros2 service call /dobot_bringup_v3/srv/StartDrag dobot_msgs_v3/srv/StartDrag '{}';
  echo 'Stopping Drag...'; ros2 service call /dobot_bringup_v3/srv/StopDrag dobot_msgs_v3/srv/StopDrag '{}';
  echo 'Closing Modbus...'; ros2 service call /dobot_bringup_v3/srv/ModbusClose dobot_msgs_v3/srv/ModbusClose '{index: 0}';
  echo 'Creating Modbus...'; ros2 service call /dobot_bringup_v3/srv/ModbusCreate dobot_msgs_v3/srv/ModbusCreate '{ip: \"127.0.0.1\", port: 60000, slave_id: 9, is_rtu: 1}';
  echo 'Setting HoldRegs to 0...'; ros2 service call /dobot_bringup_v3/srv/SetHoldRegs dobot_msgs_v3/srv/SetHoldRegs '{index: 0, addr: 1000, count: 3, val_tab: \"0,0,0\", val_type: \"int\"}';
  echo 'Setting HoldRegs to 256...'; ros2 service call /dobot_bringup_v3/srv/SetHoldRegs dobot_msgs_v3/srv/SetHoldRegs '{index: 0, addr: 1000, count: 3, val_tab: \"256,0,0\", val_type: \"int\"}';
  exec bash"

# Step 2: Run servo action client
echo "Starting servo action client..."
gnome-terminal -- bash -c "echo 'Starting servo action client...'; ros2 run servo_action action_move_client_reality; exec bash"
sleep 1.5  # Wait for the client to run

# Step 3: Launch dobot_moveit
echo "Launching dobot_moveit..."
gnome-terminal -- bash -c "echo 'Launching dobot_moveit...'; ros2 launch dobot_moveit dobot_moveit.launch.py; exec bash"
sleep 1.5  # Wait for moveit to initialize

# Step 4: Terminate servo action client and start server
echo "Terminating servo action client..."
pkill -f "ros2 run servo_action action_move_client_reality"
sleep 1.5
echo "Starting servo action server..."
gnome-terminal -- bash -c "echo 'Starting servo action server...'; ros2 run servo_action action_move_server_reality; exec bash"
sleep 1.5

# Step 5: Launch the camera
echo "Launching gemini 335..."
gnome-terminal -- bash -c "echo 'Launching gemini 335...'; ros2 launch orbbec_camera gemini_330_series.launch.py; exec bash"
sleep 1.5

# Step 6: Launch the pose generator
echo "running pose generator..."
gnome-terminal -- bash -c "echo 'running pose generator...'; ros2 run pickn_place pose_generator; exec bash"
sleep 1.5

# Step 7: Launch the base obstacle
echo "Launch base obstacle..."
gnome-terminal -- bash -c "echo 'launchin obstacle...'; ros2 run pickn_place obstacle_generator; exec bash"
sleep 1.5

# Step 8: Run the python cli
echo "Running the python cli..."
gnome-terminal -- bash -c "echo 'Running the python cli...'; python3 -m oms_v1.cli; exec bash"
