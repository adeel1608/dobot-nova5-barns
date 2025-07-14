#!/usr/bin/env bash
set -e

# -----------------------------------------------------------------------------
# Clean up Orbbec device lock (prevents camera conflicts)
# -----------------------------------------------------------------------------
echo "=== Cleaning up Orbbec device lock ==="
if [ -f "/dev/shm/orbbec_device_lock" ]; then
    echo "Removing existing Orbbec device lock..."
    rm -f /dev/shm/orbbec_device_lock 2>/dev/null || {
        echo "WARNING: Could not remove Orbbec device lock (may not exist or permission issue)"
    }
else
    echo "No Orbbec device lock found (this is normal on first run)"
fi

# -----------------------------------------------------------------------------
# Environment variables for headless and optimized operation
# -----------------------------------------------------------------------------
export DISPLAY=${DISPLAY:-:99}  # Use virtual display if no display available
export QT_QPA_PLATFORM=offscreen  # Run Qt applications headlessly
export ROS_LOG_LEVEL=WARN  # Global log level optimization
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=WARN  # Additional logging optimization
export ROS_DISABLE_LOANED_MESSAGES=1  # Disable loaned messages for lower memory usage

# Set DOBOT_TYPE for MoveIt configuration (critical for launch file selection)
export DOBOT_TYPE=${DOBOT_TYPE:-nova5}

# Disable GUI components
export MOVEIT_DISABLE_GUI=1
export RVIZ_DISABLE=1

echo "=== Starting BARNS Robot Stack in Optimized Headless Mode ==="

# -----------------------------------------------------------------------------
# Source your ROS 2 overlay
# -----------------------------------------------------------------------------
source /root/ros_ws/install/setup.bash

# -----------------------------------------------------------------------------
# Helper function to wait for a service
# -----------------------------------------------------------------------------
wait_for_service() {
  local srv_name="$1"
  local max_wait="${2:-30}"  # Default 30 second timeout
  local count=0
  echo "Waiting for service ${srv_name} ..."
  until ros2 service type "${srv_name}" > /dev/null 2>&1; do
    sleep 0.2
    count=$((count + 1))
    if [ $count -gt $((max_wait * 5)) ]; then
      echo "ERROR: Service ${srv_name} not available after ${max_wait} seconds"
      return 1
    fi
  done
  echo "  ↳ ${srv_name} is now available."
}

# -----------------------------------------------------------------------------
# 1) Launch dobot_bringup_v3 (optimized logging)
# -----------------------------------------------------------------------------
echo "=== Launching dobot_bringup_v3 (optimized) ==="
ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py __log_level:=error &
DOBOT_BRINGUP_PID=$!

sleep 1

wait_for_service "/dobot_bringup_v3/srv/ClearError"
echo "Calling ClearError ..."
ros2 service call /dobot_bringup_v3/srv/ClearError dobot_msgs_v3/srv/ClearError "{}" > /dev/null

wait_for_service "/dobot_bringup_v3/srv/DisableRobot"
echo "Calling DisableRobot ..."
ros2 service call /dobot_bringup_v3/srv/DisableRobot dobot_msgs_v3/srv/DisableRobot "{}" > /dev/null

wait_for_service "/dobot_bringup_v3/srv/EnableRobot"
echo "Calling EnableRobot (load: 2.0) ..."
ros2 service call /dobot_bringup_v3/srv/EnableRobot dobot_msgs_v3/srv/EnableRobot "{load: 2.0}" > /dev/null

ros2 service call /dobot_bringup_v3/srv/SetGripperPosition dobot_msgs_v3/srv/SetGripperPosition "{position: 0, speed: 255, force: 255}"

wait_for_service "/dobot_bringup_v3/srv/StartDrag"
echo "Calling StartDrag ..."
ros2 service call /dobot_bringup_v3/srv/StartDrag dobot_msgs_v3/srv/StartDrag "{}" > /dev/null

sleep 5

wait_for_service "/dobot_bringup_v3/srv/StopDrag"
echo "Calling StopDrag ..."
ros2 service call /dobot_bringup_v3/srv/StopDrag dobot_msgs_v3/srv/StopDrag "{}" > /dev/null

# -----------------------------------------------------------------------------
# 1.a) Get current angle1, pick 45° bin, then call ServoJ (optimized)
# -----------------------------------------------------------------------------
wait_for_service "/dobot_bringup_v3/srv/GetAngle"
echo "Calling GetAngle to read joint-1…"
# call GetAngle and capture the raw response
angle_raw=$(ros2 service call /dobot_bringup_v3/srv/GetAngle dobot_msgs_v3/srv/GetAngle 2>/dev/null)
# extract the comma-separated list inside the braces from the robot response
angle_list=$(echo "$angle_raw" \
  | grep -oP '\{[^}]+\}' \
  | tr -d '{}' )
# grab the first field (joint-1)
a1=$(echo "$angle_list" | cut -d',' -f1)
echo "  ↳ joint-1 = ${a1}°"

# determine which 45° bin center (multiples of 45) lies within ±22.49°
# default to skip if somehow outside all ranges
j1_val=
if   awk "BEGIN {exit !($a1 >= -22.49 && $a1 <=  22.49)}"; then j1_val=0.0
elif awk "BEGIN {exit !($a1 >=  22.51 && $a1 <=  67.49)}"; then j1_val=45.0
elif awk "BEGIN {exit !($a1 >=  67.51 && $a1 <= 112.49)}"; then j1_val=90.0
elif awk "BEGIN {exit !($a1 >= 112.51 && $a1 <= 157.49)}"; then j1_val=135.0
elif awk "BEGIN {exit !($a1 >= 157.51 && $a1 <= 202.49)}"; then j1_val=180.0
elif awk "BEGIN {exit !($a1 <= -22.51 && $a1 >= -67.49)}"; then j1_val=-45.0
elif awk "BEGIN {exit !($a1 <= -67.51 && $a1 >= -112.49)}"; then j1_val=-90.0
elif awk "BEGIN {exit !($a1 <= -112.51 && $a1 >= -157.49)}"; then j1_val=-135.0
elif awk "BEGIN {exit !($a1 <= -157.51 && $a1 >= -202.49)}"; then j1_val=-180.0
else
  echo "WARNING: angle ${a1}° is outside all bins; skipping ServoJ."
fi

# if we got a valid j1, dispatch the ServoJ call
if [[ -n "$j1_val" ]]; then
  echo "Dispatching ServoJ → j1=${j1_val}"
  wait_for_service "/dobot_bringup_v3/srv/ServoJ"
  ros2 service call /dobot_bringup_v3/srv/ServoJ dobot_msgs_v3/srv/ServoJ \
    "{j1: ${j1_val}, j2: 30.0, j3: -130.0, j4: -100.0, j5: -90.0, j6: 0.0, t: 2.0}" > /dev/null
fi

wait_for_service "/dobot_bringup_v3/srv/StartDrag"
echo "Calling StartDrag ..."
ros2 service call /dobot_bringup_v3/srv/StartDrag dobot_msgs_v3/srv/StartDrag "{}" > /dev/null

wait_for_service "/dobot_bringup_v3/srv/StopDrag"
echo "Calling StopDrag ..."
ros2 service call /dobot_bringup_v3/srv/StopDrag dobot_msgs_v3/srv/StopDrag "{}" > /dev/null

wait_for_service "/dobot_bringup_v3/srv/ModbusClose"
echo "Calling ModbusClose (index: 0) ..."
ros2 service call /dobot_bringup_v3/srv/ModbusClose dobot_msgs_v3/srv/ModbusClose "{index: 0}" > /dev/null

wait_for_service "/dobot_bringup_v3/srv/ModbusCreate"
echo "Calling ModbusCreate (ip: 127.0.0.1, port: 60000, slave_id: 9, is_rtu: 1) ..."
ros2 service call /dobot_bringup_v3/srv/ModbusCreate dobot_msgs_v3/srv/ModbusCreate \
  "{ip: \"127.0.0.1\", port: 60000, slave_id: 9, is_rtu: 1}" > /dev/null

wait_for_service "/dobot_bringup_v3/srv/SetHoldRegs"
echo "Calling SetHoldRegs (zero‐out) ..."
ros2 service call /dobot_bringup_v3/srv/SetHoldRegs dobot_msgs_v3/srv/SetHoldRegs \
  "{index: 0, addr: 1000, count: 3, val_tab: \"0,0,0\", val_type: \"int\"}" > /dev/null

echo "Calling SetHoldRegs (load=256) ..."
ros2 service call /dobot_bringup_v3/srv/SetHoldRegs dobot_msgs_v3/srv/SetHoldRegs \
  "{index: 0, addr: 1000, count: 3, val_tab: \"256,0,0\", val_type: \"int\"}" > /dev/null

# -----------------------------------------------------------------------------
# 2) Launch MoveIt HEADLESS (no RVIZ, no GUI)
# -----------------------------------------------------------------------------
echo "=== Launching dobot_moveit (headless, no RVIZ) ==="
echo "Using DOBOT_TYPE: ${DOBOT_TYPE}"

# Verify the MoveIt package exists
MOVEIT_PACKAGE="${DOBOT_TYPE}_moveit"
if ! ros2 pkg list | grep -q "^${MOVEIT_PACKAGE}$"; then
    echo "ERROR: MoveIt package '${MOVEIT_PACKAGE}' not found!"
    echo "Available MoveIt packages:"
    ros2 pkg list | grep moveit | head -5
    exit 1
fi

echo "✅ Found MoveIt package: ${MOVEIT_PACKAGE}"
ros2 launch dobot_moveit dobot_moveit.launch.py \
  use_rviz:=false \
  debug:=false \
  __log_level:=fatal &> /dev/null &
MOVEIT_PID=$!

sleep 2
# -----------------------------------------------------------------------------
# 3) Launch servo_action server (optimized logging)
# -----------------------------------------------------------------------------
echo "=== Launching servo_action server (optimized) ==="
ros2 run servo_action action_move_server_reality __log_level:=fatal &
ACTION_SERVER_PID=$!

# -----------------------------------------------------------------------------
# 4) Launch Orbbec camera (silence everything)
# -----------------------------------------------------------------------------
echo "=== Launching Orbbec camera (silenced) ==="

# Validate and set default environment variables to prevent expansion issues
if [ -z "${CAM_NAME}" ]; then
    CAM_NAME="camera"
fi
if [ -z "${CAMERA_SERIAL_NUMBER}" ]; then
    CAMERA_SERIAL_NUMBER=""
fi
if [ -z "${USB_PORT}" ]; then
    USB_PORT=""
fi
if [ -z "${DEVICE_NUM}" ]; then
    DEVICE_NUM="1"
fi

echo "Camera configuration:"
echo "  - Camera Name: ${CAM_NAME}"
echo "  - Serial Number: ${CAMERA_SERIAL_NUMBER}"
echo "  - USB Port: ${USB_PORT}"
echo "  - Device Num: ${DEVICE_NUM}"

# List available USB devices for debugging
echo "Available USB devices:"
lsusb 2>/dev/null || echo "lsusb not available"

# List Orbbec devices and their UIDs
echo "Orbbec devices and UIDs:"
ros2 run orbbec_camera list_devices_node 2>/dev/null || echo "list_devices_node not available"

# Verify the target device exists and get its index
echo "Searching for target device with serial number: ${CAMERA_SERIAL_NUMBER}"
DEVICE_LIST=$(ros2 run orbbec_camera list_devices_node 2>/dev/null || echo "")

if [ -z "$DEVICE_LIST" ]; then
  echo "ERROR: Could not enumerate Orbbec devices!"
  exit 1
fi

# Check if target device exists
DEVICE_FOUND=$(echo "$DEVICE_LIST" | grep -c "${CAMERA_SERIAL_NUMBER}" || echo "0")

if [ "$DEVICE_FOUND" -eq "0" ]; then
  echo "ERROR: Target camera with serial number ${CAMERA_SERIAL_NUMBER} not found!"
  echo "Available devices:"
  echo "$DEVICE_LIST"
  exit 1
fi

# Try to determine correct device index based on enumeration
echo "Device enumeration:"
echo "$DEVICE_LIST"

# Extract device index for our target serial number if possible
TARGET_DEVICE_INDEX=$(echo "$DEVICE_LIST" | grep -n "${CAMERA_SERIAL_NUMBER}" | head -1 | cut -d: -f1)
if [ -n "$TARGET_DEVICE_INDEX" ]; then
  echo "Target device appears to be at index: $TARGET_DEVICE_INDEX"
  # Adjust DEVICE_NUM if needed (1-based indexing)
  DEVICE_NUM=$TARGET_DEVICE_INDEX
  echo "Using device_num: $DEVICE_NUM"
fi

# Launch camera with validated parameters and force serial number selection
if [ -n "${CAMERA_SERIAL_NUMBER}" ]; then
  echo "Attempting to connect to camera with serial number: ${CAMERA_SERIAL_NUMBER}"
  echo "Camera launch parameters:"
  echo "  - camera_name: ${CAM_NAME}"
  echo "  - serial_number: ${CAMERA_SERIAL_NUMBER}"
  echo "  - usb_port: ${USB_PORT}"
  echo "  - device_num: ${DEVICE_NUM}"
  
  ros2 launch orbbec_camera gemini_330_series.launch.py \
    camera_name:="${CAM_NAME}" \
    serial_number:="${CAMERA_SERIAL_NUMBER}" \
    usb_port:="${USB_PORT}" \
    device_num:="${DEVICE_NUM}" \
    connection_delay:=1000 \
    device_index:="${DEVICE_NUM}" \
    enable_sync_output_accel_gyro:=false \
    enable_noise_removal_filter:=false \
    enable_spatial_filter:=false \
    enable_temporal_filter:=false \
    enable_hole_filling_filter:=false \
    enable_decimation_filter:=false \
    enable_threshold_filter:=false \
    enable_sequence_id_filter:=false \
    enable_hdr_merge:=false \
    __log_level:=info &
  CAMERA_PID=$!
else
  echo "ERROR: CAMERA_SERIAL_NUMBER not set! Cannot launch camera."
  exit 1
fi

sleep 2
# Wait for camera to initialize and verify topics
echo "Waiting for camera to initialize..."
MAX_RETRIES=30
RETRY_COUNT=0

# Function to check if camera info is being published
check_camera_info() {
    # Try to get the latest message from the camera info topic using dynamic camera name
    MSG_COUNT=$(timeout 2 ros2 topic echo --once /${CAM_NAME}/color/camera_info 2>/dev/null | wc -l)
    if [ $MSG_COUNT -gt 0 ]; then
        return 0  # Success
    else
        return 1  # Failure
    fi
}

while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    if check_camera_info; then
        echo "Camera info topic is actively publishing!"
        break
    fi
    echo "Waiting for camera info messages... ($(( RETRY_COUNT + 1 ))/$MAX_RETRIES)"
    sleep 1
    RETRY_COUNT=$(( RETRY_COUNT + 1 ))
done

if [ $RETRY_COUNT -eq $MAX_RETRIES ]; then
    echo "WARNING: Camera info not publishing after $MAX_RETRIES seconds"
else
    # Verify the correct camera device is connected
    echo "Verifying camera device connection..."
    sleep 3
    
    # Check if the camera service is available and get device info
    if ros2 service list | grep -q "/${CAM_NAME}/get_device_info"; then
        echo "Checking connected device serial number..."
        CONNECTED_SERIAL=$(ros2 service call /${CAM_NAME}/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo {} 2>/dev/null | grep -o "serial_number: '[^']*'" | cut -d"'" -f2 || echo "")
        
        if [ -n "$CONNECTED_SERIAL" ]; then
            echo "Connected device serial number: $CONNECTED_SERIAL"
            if [ "$CONNECTED_SERIAL" = "$CAMERA_SERIAL_NUMBER" ]; then
                echo "✅ SUCCESS: Correct camera device connected!"
            else
                echo "❌ ERROR: Wrong camera device connected!"
                echo "  Expected: $CAMERA_SERIAL_NUMBER"
                echo "  Actual: $CONNECTED_SERIAL"
                echo "  This may cause camera assignment issues between robots."
            fi
        else
            echo "WARNING: Could not retrieve device serial number"
        fi
    else
        echo "WARNING: Camera device info service not available"
    fi
fi
# ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090 -p address:=0.0.0.0

# -----------------------------------------------------------------------------
# 5) Launch perception nodes (pose_generator, obstacle_generator) silently
# -----------------------------------------------------------------------------
echo "=== Spinning up pose_generator (silenced) ==="
ros2 run pickn_place pose_generator __log_level:=fatal &> /dev/null &
POSE_GEN_PID=$!

echo "=== Spinning up obstacle_generator (silenced) ==="
ros2 run pickn_place obstacle_generator __log_level:=fatal &> /dev/null &
OBSTACLE_GEN_PID=$!

# -----------------------------------------------------------------------------
# 6) Launch oms_v1.app service in separate background process
# -----------------------------------------------------------------------------
echo "=== Launching oms_v1.app service (log-level=info) ==="
cd /root/ros_ws/src/oms_v1 && python -m oms_v1.app --service &
OMS_APP_PID=$!

# -----------------------------------------------------------------------------
# 7) Finally, leave the container alive (so dobot_bringup_v3 & servo_action keep running)
# -----------------------------------------------------------------------------
echo "=== All subsystems launched. Only dobot_bringup_v3 and servo_action will print logs. ==="
echo "=== All subsystems should be ready. ==="
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
  $OBSTACLE_GEN_PID \
  $OMS_APP_PID" EXIT