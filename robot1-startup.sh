#!/usr/bin/env bash
###############################
# BARNS Robot 1 Startup Script
# For separate PC running both Docker services and Robot 1
###############################

set -euo pipefail

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR=${WORKSPACE_DIR:-${SCRIPT_DIR}/services/robot_container/ros_ws}
DOCKER_HOST_IP=${DOCKER_HOST_IP:-$(hostname -I | awk '{print $1}')}
ROBOT_ID=1
DOBOT_TYPE=${DOBOT_TYPE:-nova5}
IP_ADDRESS=${IP_ADDRESS:-192.168.200.249}
ROS_DOMAIN_ID=0
CAMERA_SERIAL_NUMBER=${CAMERA_SERIAL_NUMBER:-CP1Z842000YW}
CAM_NAME=cam0
DEVICE_ID=1
DEVICE_NUM=1
ORBBEC_CONNECTION_DELAY=3000
USBFS_MEMORY_MB=128

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Global variables for process tracking
CAMERA_PID=""
POSE_GEN_PID=""
RESTART_IN_PROGRESS=false

log() {
    echo -e "${GREEN}[ROBOT1]${NC} $*"
}

warn() {
    echo -e "${YELLOW}[WARNING]${NC} $*"
}

error() {
    echo -e "${RED}[ERROR]${NC} $*"
    exit 1
}

info() {
    echo -e "${BLUE}[INFO]${NC} $*"
}

# Function to reset USB device (helps with Orbbec camera initialization)
reset_usb_device() {
    log "Attempting USB device reset for Orbbec camera..."
    
    # Find the Orbbec camera USB device
    local orbbec_device=$(lsusb | grep -i "2bc5" | head -1)
    
    if [ -z "$orbbec_device" ]; then
        warn "Could not find Orbbec camera USB device, skipping USB reset"
        return 1
    fi
    
    # Extract bus and device numbers
    local bus=$(echo "$orbbec_device" | grep -oP "Bus \K[0-9]+")
    local dev=$(echo "$orbbec_device" | grep -oP "Device \K[0-9]+")
    
    if [ -n "$bus" ] && [ -n "$dev" ]; then
        log "Found Orbbec camera at Bus $bus Device $dev"
        
        # Unbind and rebind the USB device
        local usb_path="/sys/bus/usb/devices/$bus-*"
        for device_path in $usb_path; do
            if [ -e "$device_path/authorized" ]; then
                log "Resetting USB device at $device_path"
                echo 0 | sudo tee "$device_path/authorized" > /dev/null 2>&1 || true
                sleep 1
                echo 1 | sudo tee "$device_path/authorized" > /dev/null 2>&1 || true
                sleep 2
                log "USB device reset complete"
                return 0
            fi
        done
    fi
    
    warn "Could not reset USB device"
    return 1
}

# Function to restart camera and perception nodes
restart_camera_and_perception() {
    if [ "$RESTART_IN_PROGRESS" = true ]; then
        warn "Restart already in progress, skipping duplicate restart request"
        return
    fi
    
    RESTART_IN_PROGRESS=true
    warn "Restarting camera and perception nodes due to camera info error..."
    
    # Kill camera and perception processes by name
    warn "Terminating camera and perception processes..."
    pkill -TERM -f "orbbec_camera" &>/dev/null || true
    pkill -TERM -f "aruco_perception" &>/dev/null || true
    
    # Wait a moment then force kill if needed
    sleep 2
    pkill -KILL -f "orbbec_camera" &>/dev/null || true
    pkill -KILL -f "aruco_perception" &>/dev/null || true
    
    # Try USB reset to help with camera initialization
    reset_usb_device || true
    
    # Clear any device locks
    rm -f /dev/shm/orbbec_device_lock 2>/dev/null || true
    
    log "Waiting 5 seconds before restarting camera..."
    sleep 5
    
    # Restart camera and track PID
    log "Restarting Orbbec camera..."
    ros2 launch orbbec_camera gemini_330_series.launch.py __log_level:=info &
    local NEW_CAMERA_PID=$!
    log "Camera restarted with PID: $NEW_CAMERA_PID"
    
    # Wait and validate camera started
    log "Waiting 10 seconds for camera to initialize..."
    sleep 10
    
    # Validate camera is publishing topics
    local camera_ok=false
    local color_topic=$(ros2 topic list 2>/dev/null | grep "/camera/color/camera_info" || true)
    local depth_topic=$(ros2 topic list 2>/dev/null | grep "/camera/depth/camera_info" || true)
    
    if [ -n "$color_topic" ] && [ -n "$depth_topic" ]; then
        log "✓ Camera validation passed - both topics available"
        camera_ok=true
    else
        warn "⚠ Camera topics not fully available yet, but continuing"
    fi
    
    # Restart perception and track PID
    log "Restarting ArUco perception..."
    ros2 run pickn_place aruco_perception __log_level:=fatal &
    local NEW_PERCEPTION_PID=$!
    log "Perception restarted with PID: $NEW_PERCEPTION_PID"
    
    # Give perception time to start
    sleep 3
    
    # Check if processes are still running
    if kill -0 $NEW_CAMERA_PID 2>/dev/null && kill -0 $NEW_PERCEPTION_PID 2>/dev/null; then
        log "✓ Camera and perception nodes restarted successfully"
    else
        warn "⚠ Some processes may have died after restart"
    fi
    
    # Reset flag AFTER a delay to prevent rapid restart loops
    log "Waiting 30 seconds before allowing another restart..."
    sleep 30
    RESTART_IN_PROGRESS=false
    log "Restart cooldown completed, monitoring resumed"
}

# Function to wait for robot stack to be fully initialized
wait_for_robot_stack_ready() {
    log "Waiting for robot stack to be fully initialized..."
    local max_wait=120  # 2 minutes maximum wait
    local count=0
    
    while [ $count -lt $max_wait ]; do
        # Check for essential nodes and topics
        local camera_node=$(ros2 node list 2>/dev/null | grep "/camera/camera" || true)
        local aruco_node=$(ros2 node list 2>/dev/null | grep "aruco_perception_node" || true)
        local color_topic=$(ros2 topic list 2>/dev/null | grep "/camera/color/image_raw" || true)
        local depth_topic=$(ros2 topic list 2>/dev/null | grep "/camera/depth/image_raw" || true)
        local color_info_topic=$(ros2 topic list 2>/dev/null | grep "/camera/color/camera_info" || true)
        local depth_info_topic=$(ros2 topic list 2>/dev/null | grep "/camera/depth/camera_info" || true)
        
        if [ -n "$camera_node" ] && [ -n "$aruco_node" ] && [ -n "$color_topic" ] && [ -n "$depth_topic" ] && [ -n "$color_info_topic" ] && [ -n "$depth_info_topic" ]; then
            log "Robot stack fully initialized with color and depth cameras ready"
            return 0
        fi
        
        sleep 5
        count=$((count + 5))
        echo -n "."
    done
    
    warn "Robot stack initialization timeout after $max_wait seconds, starting monitoring anyway..."
    return 1
}

# Function to monitor perception node logs for errors  
monitor_perception_errors() {
    # First wait for the robot stack to be ready
    local stack_ready_result
    wait_for_robot_stack_ready
    stack_ready_result=$?
    
    # If stack didn't initialize properly (depth camera issue), restart immediately
    if [ $stack_ready_result -ne 0 ]; then
        warn "Robot stack failed to initialize properly, restarting camera and perception..."
        restart_camera_and_perception
        # Wait again after restart
        wait_for_robot_stack_ready
    fi
    
    log "Starting camera error monitoring..."
    log "Monitoring for: 'No color camera info received' and 'Waiting for depth camera intrinsics'"
    
    local warning_count=0
    local last_warning_time=0
    local depth_warning_count=0
    local last_depth_warning_time=0
    
    while true; do
        sleep 10
        
        # Check if processes are still running
        if ! pgrep -f "aruco_perception" >/dev/null 2>&1; then
            warn "ArUco perception process died, restarting..."
            restart_camera_and_perception
            warning_count=0
            depth_warning_count=0
            continue
        fi
        
        if ! pgrep -f "orbbec_camera" >/dev/null 2>&1; then
            warn "Camera process died, restarting..."
            restart_camera_and_perception
            warning_count=0
            depth_warning_count=0
            continue
        fi
        
        local current_time=$(date +%s)
        
        # Check for "Waiting for depth camera intrinsics" warnings
        local found_depth_warning="false"
        if journalctl --since "30 seconds ago" 2>/dev/null | grep -q "Waiting for depth camera intrinsics"; then
            found_depth_warning="true"
        fi
        
        if [ "$found_depth_warning" = "false" ] && [ -d "$HOME/.ros/log" ]; then
            if find "$HOME/.ros/log" -name "*.log" -newermt "30 seconds ago" -exec grep -l "Waiting for depth camera intrinsics" {} \; 2>/dev/null | head -1 | grep -q .; then
                found_depth_warning="true"
            fi
        fi
        
        if [ "$found_depth_warning" = "true" ]; then
            # For depth warnings, restart after 4 consecutive warnings within 2 minutes
            # (increased from 2 to reduce false positives during normal operation)
            if [ $((current_time - last_depth_warning_time)) -lt 120 ]; then
                depth_warning_count=$((depth_warning_count + 1))
            else
                depth_warning_count=1
            fi
            
            last_depth_warning_time=$current_time
            warn "Detected 'Waiting for depth camera intrinsics' warning ($depth_warning_count/4)"
            
            if [ $depth_warning_count -ge 4 ]; then
                warn "Depth camera not initializing properly after $depth_warning_count warnings, restarting..."
                restart_camera_and_perception
                depth_warning_count=0
                warning_count=0
                continue
            fi
        else
            # Reset depth warning count if no warnings for a while
            if [ $depth_warning_count -gt 0 ] && [ $((current_time - last_depth_warning_time)) -gt 180 ]; then
                depth_warning_count=0
                log "No depth camera warnings for 3+ minutes, count reset"
            fi
        fi
        
        # Check for "No color camera info received" warnings
        local found_color_warning="false"
        if journalctl --since "30 seconds ago" 2>/dev/null | grep -q "No color camera info received"; then
            found_color_warning="true"
        fi
        
        if [ "$found_color_warning" = "false" ] && [ -d "$HOME/.ros/log" ]; then
            if find "$HOME/.ros/log" -name "*.log" -newermt "30 seconds ago" -exec grep -l "No color camera info received" {} \; 2>/dev/null | head -1 | grep -q .; then
                found_color_warning="true"
            fi
        fi
        
        if [ "$found_color_warning" = "true" ]; then
            if [ $((current_time - last_warning_time)) -lt 60 ]; then
                warning_count=$((warning_count + 1))
            else
                warning_count=1
            fi
            
            last_warning_time=$current_time
            warn "Detected 'No color camera info received' warning ($warning_count/3)"
            
            if [ $warning_count -ge 3 ]; then
                warn "Multiple color camera info warnings detected, restarting camera and perception..."
                restart_camera_and_perception
                warning_count=0
                depth_warning_count=0
            fi
        else
            # Reset color warning count if no warnings for a while
            if [ $warning_count -gt 0 ] && [ $((current_time - last_warning_time)) -gt 120 ]; then
                warning_count=0
                log "No color camera warnings for 2+ minutes, count reset"
            fi
        fi
    done
}

# Check if workspace exists
check_workspace() {
    if [ ! -d "$WORKSPACE_DIR" ]; then
        error "Workspace not found at $WORKSPACE_DIR. Please run install-robot-dependencies.sh first."
    fi
    
    if [ ! -f "$WORKSPACE_DIR/setup_robot_env.sh" ]; then
        error "Environment setup script not found. Please run install-robot-dependencies.sh first."
    fi
}

# Clean up RabbitMQ queues to prevent resource lock issues
cleanup_rabbitmq_queues() {
    log "Cleaning up RabbitMQ queues to prevent resource lock issues..."
    
    # Check if RabbitMQ container is running
    if ! docker ps | grep -q barns-rabbitmq; then
        warn "RabbitMQ container not running, skipping queue cleanup"
        return
    fi
    
    # Delete problematic queues if they exist
    local queues_to_delete=(
        "robot_container_1_responses"
        "robot_container_1_requests"
        "robot_container_2_responses"
        "robot_container_2_requests"
    )
    
    for queue in "${queues_to_delete[@]}"; do
        if docker exec barns-rabbitmq rabbitmqctl list_queues name | grep -q "^${queue}$"; then
            log "Deleting queue: $queue"
            docker exec barns-rabbitmq rabbitmqctl delete_queue "$queue" 2>/dev/null || true
        fi
    done
    
    # Also kill any lingering Python processes that might reconnect with old settings
    pkill -f "python.*oms_v1" &>/dev/null || true
    
    log "RabbitMQ queue cleanup completed"
}

# Clean up ROS2 environment to prevent runtime errors
cleanup_ros2_environment() {
    log "Cleaning up ROS2 environment..."
    
    # Kill any existing ROS2 processes
    pkill -f "ros2" &>/dev/null || true
    pkill -f "_ros2_daemon" &>/dev/null || true
    
    # Clean up ROS2 daemon
    ros2 daemon stop &>/dev/null || true
    
    # Remove any stale ROS2 runtime files
    rm -rf /tmp/.ros* 2>/dev/null || true
    rm -rf ~/.ros/log/* 2>/dev/null || true
    
    # Wait a moment for cleanup
    sleep 2
    
    log "ROS2 environment cleanup completed"
}

# Start Docker services
start_docker_services() {
    log "Starting Docker services..."
    
    # Check if Docker is installed and running
    if ! command -v docker &> /dev/null; then
        error "Docker is not installed. Please install Docker first."
    fi
    
    if ! docker info &> /dev/null; then
        error "Docker is not running. Please start Docker service."
    fi
    
    # Check if docker-compose file exists
    if [ ! -f "docker-compose.arms.yml" ]; then
        error "docker-compose.arms.yml not found. Please run this script from the BARNS project directory."
    fi
    
    # Set up X11 forwarding
    log "Setting up X11 forwarding for Docker containers..."
    xhost +local:root 2>/dev/null || warn "Could not set X11 forwarding (xhost not available)"
    
    # Stop any existing services
    log "Stopping existing BARNS services..."
    docker compose -f docker-compose.arms.yml down 2>/dev/null || true
    
    # Start Docker services
    log "Starting BARNS Docker services..."
    docker compose -f docker-compose.arms.yml up -d --build
    
    # Wait for RabbitMQ to be ready
    log "Waiting for RabbitMQ to be ready..."
    local max_wait=60
    local count=0
    while ! docker exec barns-rabbitmq rabbitmq-diagnostics ping &>/dev/null; do
        sleep 1
        count=$((count + 1))
        if [ $count -gt $max_wait ]; then
            error "RabbitMQ failed to start within $max_wait seconds"
        fi
        echo -n "."
    done
    echo
    
    log "Docker services started successfully!"
    info "Dashboard: http://localhost:3000"
    info "RabbitMQ Management: http://localhost:15672 (admin/admin123)"
}

# Wait for services to be ready
wait_for_services() {
    log "Waiting for core services to be ready..."
    
    # Wait a bit longer for all services to initialize
    sleep 10
    
    # Check if routine service is responding
    local max_wait=30
    local count=0
    while ! docker exec barns-routine python -c "import socket; socket.create_connection(('rabbitmq', 5672), timeout=5)" &>/dev/null; do
        sleep 2
        count=$((count + 2))
        if [ $count -gt $max_wait ]; then
            warn "Routine service may not be fully ready, but continuing..."
            break
        fi
    done
    
    log "Services are ready for robot connection"
}

# Start robot process
start_robot() {
    log "Starting Robot 1 process..."
    
    # Source the environment
    source "$WORKSPACE_DIR/setup_robot_env.sh"
    
    # Set robot-specific environment variables
    export DOBOT_TYPE="$DOBOT_TYPE"
    export IP_address="$IP_ADDRESS"
    export ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
    export ROBOT_ID="$ROBOT_ID"
    export RABBITMQ_URL="amqp://admin:admin123@${DOCKER_HOST_IP}:5672/"
    export USBFS_MEMORY_MB="$USBFS_MEMORY_MB"
    export ORBBEC_CONNECTION_DELAY="$ORBBEC_CONNECTION_DELAY"
    export CAMERA_SERIAL_NUMBER="$CAMERA_SERIAL_NUMBER"
    export CAM_NAME="$CAM_NAME"
    export DEVICE_ID="$DEVICE_ID"
    export DEVICE_NUM="$DEVICE_NUM"
    
    log "Robot 1 Configuration:"
    info "  - Robot ID: $ROBOT_ID"
    info "  - Dobot IP: $IP_ADDRESS"
    info "  - ROS Domain: $ROS_DOMAIN_ID"
    info "  - Camera Serial: $CAMERA_SERIAL_NUMBER"
    info "  - RabbitMQ URL: $RABBITMQ_URL"
    
    # Change to workspace directory
    cd "$WORKSPACE_DIR"
    
    # Clean up any existing device locks
    log "Cleaning up Orbbec device lock..."
    rm -f /dev/shm/orbbec_device_lock 2>/dev/null || true
    
    # Start the robot stack using the run script logic
    log "Starting full robot stack..."
    
    # Store PIDs for cleanup
    declare -a PIDS=()
    
    # Function to kill all processes
    cleanup_processes() {
        log "Cleaning up robot processes..."
        
        # Kill all child processes more aggressively
        if [ ${#PIDS[@]} -gt 0 ]; then
            for pid in "${PIDS[@]}"; do
                if kill -0 "$pid" 2>/dev/null; then
                    # Send SIGTERM first, then SIGKILL if needed
                    kill -TERM "$pid" 2>/dev/null || true
                    sleep 0.5
                    kill -KILL "$pid" 2>/dev/null || true
                fi
            done
        fi
        
        # Kill by process name as backup (more aggressive)
        pkill -KILL -f "dobot_bringup_v3" &>/dev/null || true
        pkill -KILL -f "orbbec_camera" &>/dev/null || true
        pkill -KILL -f "dobot_moveit" &>/dev/null || true
        pkill -KILL -f "servo_action" &>/dev/null || true
        pkill -KILL -f "pickn_place" &>/dev/null || true
        pkill -KILL -f "oms_v1.app" &>/dev/null || true
        pkill -KILL -f "monitor_perception_errors" &>/dev/null || true
        
        # Clean up any remaining ros2 processes
        pkill -KILL -f "ros2" &>/dev/null || true
        
        log "Robot processes cleanup completed."
    }
    
    # Set up signal handler
    trap cleanup_processes EXIT INT TERM
    
    bash -c '
        # Import the robot startup logic
        source ./setup_robot_env.sh
        
        # Robot startup script (adapted from run_full_stack.sh)
        log() { echo -e "\033[1;32m[ROBOT1]\033[0m $*"; }
        
        # Store PIDs for cleanup
        declare -a PIDS=()
        
        # Function to kill all processes in inner shell
        cleanup_inner() {
            # Suppress output during cleanup to avoid noise
            exec 2>/dev/null
            
            # Kill tracked processes
            for pid in "${PIDS[@]}"; do
                kill -KILL "$pid" 2>/dev/null || true
            done
            
            # Kill by process group
            kill -KILL 0 2>/dev/null || true
        }
        
        # Set up signal handler for inner shell
        trap cleanup_inner EXIT INT TERM
        
        log "=== Starting BARNS Robot 1 Stack ==="
        
        # Launch dobot_bringup_v3
        log "=== Launching dobot_bringup_v3 ==="
        ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py __log_level:=error &
        DOBOT_BRINGUP_PID=$!
        PIDS+=($DOBOT_BRINGUP_PID)
        
        sleep 5

        # Launch Orbbec camera with validation
        log "=== Launching Orbbec camera ==="
        
        # Function to check if camera is working properly
        validate_camera() {
            sleep 10  # Give camera time to initialize
            
            local color_info=$(ros2 topic list 2>/dev/null | grep "/camera/color/camera_info" || true)
            local depth_info=$(ros2 topic list 2>/dev/null | grep "/camera/depth/camera_info" || true)
            
            if [ -n "$color_info" ] && [ -n "$depth_info" ]; then
                log "Camera validation passed - both color and depth info topics available"
                return 0
            else
                log "Camera validation failed - missing topics (color: $color_info, depth: $depth_info)"
                return 1
            fi
        }
        
        # Try launching camera up to 3 times
        camera_launch_attempts=0
        camera_success=false
        
        while [ $camera_launch_attempts -lt 3 ] && [ "$camera_success" = false ]; do
            camera_launch_attempts=$((camera_launch_attempts + 1))
            
            if [ $camera_launch_attempts -gt 1 ]; then
                log "Camera launch attempt $camera_launch_attempts of 3..."
                
                # Kill previous camera process
                pkill -KILL -f "orbbec_camera" &>/dev/null || true
                sleep 2
                
                # Try USB reset
                log "Attempting USB reset before retry..."
                echo 0 | sudo tee /sys/bus/usb/devices/*/authorized > /dev/null 2>&1 || true
                sleep 1
                echo 1 | sudo tee /sys/bus/usb/devices/*/authorized > /dev/null 2>&1 || true
                sleep 3
            fi
            
            ros2 launch orbbec_camera gemini_330_series.launch.py __log_level:=info &
            CAMERA_PID=$!
            PIDS+=($CAMERA_PID)
            
            if validate_camera; then
                camera_success=true
                log "Camera launched successfully"
            else
                log "Camera failed to initialize properly"
            fi
        done
        
        if [ "$camera_success" = false ]; then
            log "WARNING: Camera failed to initialize after 3 attempts, but continuing..."
        fi

        # Check if ROS2 is working properly
        log "Checking ROS2 connectivity..."
        if ! ros2 topic list &>/dev/null; then
            log "ROS2 daemon not ready, restarting..."
            ros2 daemon stop &>/dev/null || true
            sleep 2
            ros2 daemon start &>/dev/null || true
            sleep 3
        fi
        
        # Try topic list again (optional, for verification)
        ros2 topic list &>/dev/null || log "Warning: ROS2 topic list failed, but continuing..."

        sleep 5
        
        # Initialize robot
        sleep 5  # Allow services to start
        ros2 service call /dobot_bringup_v3/srv/ClearError dobot_msgs_v3/srv/ClearError "{}" > /dev/null
        
        sleep 1 # Stagger service calls
        ros2 service call /dobot_bringup_v3/srv/DisableRobot dobot_msgs_v3/srv/DisableRobot "{}" > /dev/null
        
        sleep 1 # Stagger service calls
        ros2 service call /dobot_bringup_v3/srv/EnableRobot dobot_msgs_v3/srv/EnableRobot "{load: 2.0}" > /dev/null

        sleep 1 # Stagger service calls
        ros2 service call /dobot_bringup_v3/srv/CP dobot_msgs_v3/srv/CP "{ r: 100 }" > /dev/null

        sleep 1 # Stagger service calls
        ros2 service call /dobot_bringup_v3/srv/SetGripperPosition dobot_msgs_v3/srv/SetGripperPosition "{position: 0, speed: 255, force: 255}" > /dev/null
        
        # Drag operations
        sleep 1 # Stagger service calls
        ros2 service call /dobot_bringup_v3/srv/StartDrag dobot_msgs_v3/srv/StartDrag "{}" > /dev/null
        
        sleep 5
        
        sleep 1 # Stagger service calls
        ros2 service call /dobot_bringup_v3/srv/StopDrag dobot_msgs_v3/srv/StopDrag "{}" > /dev/null
        
        # Get current angle and set position
        sleep 1 # Stagger service calls
        angle_raw=$(ros2 service call /dobot_bringup_v3/srv/GetAngle dobot_msgs_v3/srv/GetAngle 2>/dev/null)
        angle_list=$(echo "$angle_raw" | grep -oP "\{[^}]+\}" | tr -d "{}")
        a1=$(echo "$angle_list" | cut -d"," -f1)
        
        # Determine appropriate position
        j1_val=
        if   awk "BEGIN {exit !($a1 >= -22.49 && $a1 <=  22.49)}"; then j1_val=0.0
        elif awk "BEGIN {exit !($a1 >=  22.51 && $a1 <=  67.49)}"; then j1_val=45.0
        elif awk "BEGIN {exit !($a1 >=  67.51 && $a1 <= 112.49)}"; then j1_val=90.0
        elif awk "BEGIN {exit !($a1 >= 112.51 && $a1 <= 157.49)}"; then j1_val=135.0
        elif awk "BEGIN {exit !($a1 >= 157.51 && $a1 <= 202.49)}"; then j1_val=180.0
        elif awk "BEGIN {exit !($a1 >= 202.51 && $a1 <= 247.49)}"; then j1_val=-135.0
        elif awk "BEGIN {exit !($a1 >= 247.51 && $a1 <= 292.49)}"; then j1_val=-90.0
        elif awk "BEGIN {exit !($a1 >= 292.51 && $a1 <= 337.49)}"; then j1_val=-45.0
        elif awk "BEGIN {exit !($a1 >= 337.51 && $a1 <= 360.00)}"; then j1_val=0.0
        elif awk "BEGIN {exit !($a1 <= -22.51 && $a1 >= -67.49)}"; then j1_val=-45.0
        elif awk "BEGIN {exit !($a1 <= -67.51 && $a1 >= -112.49)}"; then j1_val=-90.0
        elif awk "BEGIN {exit !($a1 <= -112.51 && $a1 >= -157.49)}"; then j1_val=-135.0
        elif awk "BEGIN {exit !($a1 <= -157.51 && $a1 >= -202.49)}"; then j1_val=-180.0
        elif awk "BEGIN {exit !($a1 <= -202.51 && $a1 >= -247.49)}"; then j1_val=135.0
        elif awk "BEGIN {exit !($a1 <= -247.51 && $a1 >= -292.49)}"; then j1_val=90.0
        elif awk "BEGIN {exit !($a1 <= -292.51 && $a1 >= -337.49)}"; then j1_val=45.0
        elif awk "BEGIN {exit !($a1 <= -337.51 && $a1 >= -360.00)}"; then j1_val=0.0
        fi
        
        if [[ -n "$j1_val" ]]; then
            sleep 1 # Stagger service calls
            ros2 service call /dobot_bringup_v3/srv/JointMovJ dobot_msgs_v3/srv/JointMovJ \
                "{j1: ${j1_val}, j2: 30.0, j3: -130.0, j4: -100.0, j5: -90.0, j6: 0.0}" > /dev/null
        fi
        
        # Modbus setup
        sleep 1 # Stagger service calls
        ros2 service call /dobot_bringup_v3/srv/ModbusClose dobot_msgs_v3/srv/ModbusClose "{index: 0}" > /dev/null
        
        sleep 1 # Stagger service calls
        ros2 service call /dobot_bringup_v3/srv/ModbusCreate dobot_msgs_v3/srv/ModbusCreate \
            "{ip: \"127.0.0.1\", port: 60000, slave_id: 9, is_rtu: 1}" > /dev/null
        
        sleep 1 # Stagger service calls
        ros2 service call /dobot_bringup_v3/srv/SetHoldRegs dobot_msgs_v3/srv/SetHoldRegs \
            "{index: 0, addr: 1000, count: 3, val_tab: \"0,0,0\", val_type: \"int\"}" > /dev/null
        
        ros2 service call /dobot_bringup_v3/srv/SetHoldRegs dobot_msgs_v3/srv/SetHoldRegs \
            "{index: 0, addr: 1000, count: 3, val_tab: \"256,0,0\", val_type: \"int\"}" > /dev/null
        
        sleep 5
        
        # Launch MoveIt
        log "=== Launching MoveIt (headless) ==="
        ros2 launch dobot_moveit dobot_moveit.launch.py \
            use_rviz:=false debug:=false __log_level:=fatal &> /dev/null &
        MOVEIT_PID=$!
        PIDS+=($MOVEIT_PID)
        
        sleep 5
        
        # Launch servo action server
        log "=== Launching servo_action server ==="
        ros2 run servo_action action_move_server_reality __log_level:=fatal &
        ACTION_SERVER_PID=$!
        PIDS+=($ACTION_SERVER_PID)
        
        sleep 5

        # Launch perception nodes
        log "=== Launching perception nodes ==="
        ros2 run pickn_place aruco_perception __log_level:=fatal &
        POSE_GEN_PID=$!
        PIDS+=($POSE_GEN_PID)
        
        sleep 5
        
        ros2 run pickn_place obstacle_generator __log_level:=fatal &> /dev/null &
        OBSTACLE_GEN_PID=$!
        PIDS+=($OBSTACLE_GEN_PID)
        
        sleep 5

        # Launch OMS service
        log "=== Launching oms_v1.app service ==="
        cd src/oms_v1 && python -m oms_v1.app --service &
        OMS_APP_PID=$!
        PIDS+=($OMS_APP_PID)
        
        cd ../..
        
        log "=== Robot 1 stack started successfully ==="
        log "Robot 1 is now ready and connected to RabbitMQ at ${RABBITMQ_URL}"
        
        # Keep the process running and wait for signals
        wait
    ' &
    
    # Store the bash subprocess PID
    BASH_PID=$!
    PIDS+=($BASH_PID)
    
    # Wait a moment for the robot stack to start launching
    sleep 10
    
    # Start error monitoring in background (after all nodes are launched)
    log "Starting camera and perception error monitoring..."
    monitor_perception_errors &
    MONITOR_PID=$!
    PIDS+=($MONITOR_PID)
    
    # Wait for the bash subprocess
    wait $BASH_PID
}

# Handle shutdown
cleanup() {
    log "Shutting down Robot 1..."
    # Kill any background processes using specific patterns
    pkill -KILL -f "dobot_bringup_v3" &>/dev/null || true
    pkill -KILL -f "orbbec_camera" &>/dev/null || true
    pkill -KILL -f "dobot_moveit" &>/dev/null || true
    pkill -KILL -f "servo_action" &>/dev/null || true
    pkill -KILL -f "pickn_place" &>/dev/null || true
    pkill -KILL -f "oms_v1.app" &>/dev/null || true
    pkill -KILL -f "ros2" &>/dev/null || true
    log "Robot 1 processes stopped."
}

# Main function
main() {
    log "Starting BARNS Robot 1 (Separate PC Mode)"
    log "Docker Host IP: $DOCKER_HOST_IP"
    log "Robot IP: $IP_ADDRESS"
    echo
    
    trap cleanup EXIT
    
    check_workspace
    start_docker_services
    wait_for_services
    start_robot
}

# Handle command line arguments
case "${1:-start}" in
    start)
        main
        ;;
    docker-only)
        log "Starting only Docker services..."
        start_docker_services
        log "Docker services started. Use '$0 robot-only' to start robot."
        ;;
    robot-only)
        log "Starting only Robot 1..."
        trap cleanup EXIT
        check_workspace
        cleanup_rabbitmq_queues
        cleanup_ros2_environment
        start_robot
        ;;
    stop)
        log "Stopping Robot 1 and Docker services..."
        cleanup
        docker compose -f docker-compose.arms.yml down --volumes
        log "Stopped successfully"
        ;;
    --help|-h)
        echo "BARNS Robot 1 Startup Script"
        echo
        echo "Usage: $0 [start|docker-only|robot-only|stop|--help]"
        echo
        echo "Commands:"
        echo "  start        Start Docker services and Robot 1 (default)"
        echo "  docker-only  Start only Docker services"
        echo "  robot-only   Start only Robot 1 (assumes Docker is running)"
        echo "  stop         Stop Robot 1 and Docker services"
        echo "  --help       Show this help"
        echo
        echo "Environment variables:"
        echo "  WORKSPACE_DIR           Robot workspace (default: ./services/robot_container/ros_ws)"
        echo "  DOCKER_HOST_IP          IP for RabbitMQ connection (auto-detected)"
        echo "  IP_ADDRESS              Robot IP address (default: 192.168.200.249)"
        echo "  CAMERA_SERIAL_NUMBER    Camera serial (default: CP1Z842000YW)"
        ;;
    *)
        error "Unknown command: $1. Use --help for usage information."
        ;;
esac 