#!/bin/bash
# Single script to kill all nodes and restart them in separate terminals

echo "Killing any existing nodes..."
pkill -f "ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py"
pkill -f "ros2 run servo_action action_move_client_reality"
pkill -f "ros2 launch dobot_moveit dobot_moveit.launch.py"
pkill -f "ros2 run servo_action action_move_server_reality"
pkill -f "ros2 launch orbbec_camera gemini_330_series.launch.py"
pkill -f "ros2 run pickn_place aruco_perception"
pkill -f "ros2 run pickn_place obstacle_generator"

echo "Waiting for processes to terminate..."
sleep 1.0

# Step 1: Launch dobot bringup
echo "Launching dobot bringup..."
gnome-terminal -- bash -c "echo 'Launching dobot bringup...'; ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py; exec bash"
sleep 1.0  # Adjust the sleep if needed for initialization

# Step 2: Run servo action client
echo "Starting servo action client..."
gnome-terminal -- bash -c "echo 'Starting servo action client...'; ros2 run servo_action action_move_client_reality; exec bash"
sleep 1.0  # Wait for the client to run

# Step 3: Launch dobot_moveit
echo "Launching dobot_moveit..."
gnome-terminal -- bash -c "echo 'Launching dobot_moveit...'; ros2 launch dobot_moveit dobot_moveit.launch.py; exec bash"
sleep 1.0  # Wait for moveit to initialize

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

# Step 7: Launch the base obstacle
echo "Launch base obstacle..."
gnome-terminal -- bash -c "echo 'launchin obstacle...'; ros2 run pickn_place obstacle_generator; exec bash"
