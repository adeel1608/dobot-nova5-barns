#!/usr/bin/env bash
###############################
# BARNS Robot 2 Startup Script
# For Docker host PC (connects to existing Docker services)
###############################

set -euo pipefail

# Configuration
WORKSPACE_DIR=${WORKSPACE_DIR:-$HOME/barns_robot_ws}
DOCKER_HOST_IP=${DOCKER_HOST_IP:-127.0.0.1}
ROBOT_ID=2
DOBOT_TYPE=${DOBOT_TYPE:-nova5}
IP_ADDRESS=${IP_ADDRESS:-192.168.200.248}
ROS_DOMAIN_ID=1
CAMERA_SERIAL_NUMBER=${CAMERA_SERIAL_NUMBER:-CP1Z842000F6}
CAM_NAME=cam1
DEVICE_ID=2
USB_PORT=${USB_PORT:-2-7-6}
DEVICE_NUM=2
ORBBEC_CONNECTION_DELAY=5000
USBFS_MEMORY_MB=128
ROBOT_STARTUP_DELAY=${ROBOT_STARTUP_DELAY:-15}

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log() {
    echo -e "${GREEN}[ROBOT2]${NC} $*"
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

# Check if workspace exists
check_workspace() {
    if [ ! -d "$WORKSPACE_DIR" ]; then
        error "Workspace not found at $WORKSPACE_DIR. Please run install-robot-dependencies.sh first."
    fi
    
    if [ ! -f "$WORKSPACE_DIR/setup_robot_env.sh" ]; then
        error "Environment setup script not found. Please run install-robot-dependencies.sh first."
    fi
}

# Check if Docker services are running
check_docker_services() {
    log "Checking Docker services..."
    
    # Check if RabbitMQ is accessible
    if ! nc -z localhost 5672 2>/dev/null; then
        error "RabbitMQ is not accessible on localhost:5672. Please start Docker services first."
    fi
    
    # Check if RabbitMQ container is running
    if ! docker ps | grep -q barns-rabbitmq; then
        error "RabbitMQ container not found. Please start Docker services first."
    fi
    
    log "Docker services are running and accessible"
}

# Wait for Docker services to be ready
wait_for_docker_services() {
    log "Waiting for Docker services to be ready..."
    
    # Wait for RabbitMQ
    local max_wait=60
    local count=0
    while ! docker exec barns-rabbitmq rabbitmq-diagnostics ping &>/dev/null; do
        sleep 1
        count=$((count + 1))
        if [ $count -gt $max_wait ]; then
            error "RabbitMQ not ready within $max_wait seconds"
        fi
        echo -n "."
    done
    echo
    
    # Wait for routine service
    count=0
    while ! docker exec barns-routine python -c "import socket; socket.create_connection(('rabbitmq', 5672), timeout=5)" &>/dev/null; do
        sleep 2
        count=$((count + 2))
        if [ $count -gt 30 ]; then
            warn "Routine service may not be fully ready, but continuing..."
            break
        fi
    done
    
    log "Docker services are ready for robot connection"
}

# Start robot process
start_robot() {
    log "Starting Robot 2 process (after ${ROBOT_STARTUP_DELAY}s delay)..."
    
    # Startup delay to avoid USB conflicts with Robot 1
    if [ "$ROBOT_STARTUP_DELAY" -gt 0 ]; then
        log "Waiting ${ROBOT_STARTUP_DELAY} seconds to avoid USB conflicts with Robot 1..."
        sleep "$ROBOT_STARTUP_DELAY"
    fi
    
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
    export USB_PORT="$USB_PORT"
    export DEVICE_NUM="$DEVICE_NUM"
    
    log "Robot 2 Configuration:"
    info "  - Robot ID: $ROBOT_ID"
    info "  - Dobot IP: $IP_ADDRESS"
    info "  - ROS Domain: $ROS_DOMAIN_ID"
    info "  - Camera Serial: $CAMERA_SERIAL_NUMBER"
    info "  - RabbitMQ URL: $RABBITMQ_URL"
    info "  - Startup Delay: ${ROBOT_STARTUP_DELAY}s"
    
    # Change to workspace directory
    cd "$WORKSPACE_DIR"
    
    # Clean up any existing device locks
    log "Cleaning up Orbbec device lock..."
    rm -f /dev/shm/orbbec_device_lock 2>/dev/null || true
    
    # Start the robot stack
    log "Starting full robot stack..."
    exec bash -c '
        # Import the robot startup logic
        source ./setup_robot_env.sh
        
        # Robot startup script (adapted from run_full_stack.sh)
        log() { echo -e "\033[1;32m[ROBOT2]\033[0m $*"; }
        
        log "=== Starting BARNS Robot 2 Stack ==="
        
        # Helper function to wait for a service
        wait_for_service() {
            local srv_name="$1"
            local max_wait="${2:-30}"
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
        
        # Launch dobot_bringup_v3
        log "=== Launching dobot_bringup_v3 ==="
        ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py __log_level:=error &
        DOBOT_BRINGUP_PID=$!
        
        sleep 1
        
        # Initialize robot
        wait_for_service "/dobot_bringup_v3/srv/ClearError"
        ros2 service call /dobot_bringup_v3/srv/ClearError dobot_msgs_v3/srv/ClearError "{}" > /dev/null
        
        wait_for_service "/dobot_bringup_v3/srv/DisableRobot"
        ros2 service call /dobot_bringup_v3/srv/DisableRobot dobot_msgs_v3/srv/DisableRobot "{}" > /dev/null
        
        wait_for_service "/dobot_bringup_v3/srv/EnableRobot"
        ros2 service call /dobot_bringup_v3/srv/EnableRobot dobot_msgs_v3/srv/EnableRobot "{load: 2.0}" > /dev/null
        
        ros2 service call /dobot_bringup_v3/srv/SetGripperPosition dobot_msgs_v3/srv/SetGripperPosition "{position: 0, speed: 255, force: 255}" > /dev/null
        
        # Drag operations
        wait_for_service "/dobot_bringup_v3/srv/StartDrag"
        ros2 service call /dobot_bringup_v3/srv/StartDrag dobot_msgs_v3/srv/StartDrag "{}" > /dev/null
        
        sleep 5
        
        wait_for_service "/dobot_bringup_v3/srv/StopDrag"
        ros2 service call /dobot_bringup_v3/srv/StopDrag dobot_msgs_v3/srv/StopDrag "{}" > /dev/null
        
        # Get current angle and set position
        wait_for_service "/dobot_bringup_v3/srv/GetAngle"
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
        elif awk "BEGIN {exit !($a1 <= -22.51 && $a1 >= -67.49)}"; then j1_val=-45.0
        elif awk "BEGIN {exit !($a1 <= -67.51 && $a1 >= -112.49)}"; then j1_val=-90.0
        elif awk "BEGIN {exit !($a1 <= -112.51 && $a1 >= -157.49)}"; then j1_val=-135.0
        elif awk "BEGIN {exit !($a1 <= -157.51 && $a1 >= -202.49)}"; then j1_val=-180.0
        fi
        
        if [[ -n "$j1_val" ]]; then
            wait_for_service "/dobot_bringup_v3/srv/ServoJ"
            ros2 service call /dobot_bringup_v3/srv/ServoJ dobot_msgs_v3/srv/ServoJ \
                "{j1: ${j1_val}, j2: 30.0, j3: -130.0, j4: -100.0, j5: -90.0, j6: 0.0, t: 2.0}" > /dev/null
        fi
        
        # More drag operations
        ros2 service call /dobot_bringup_v3/srv/StartDrag dobot_msgs_v3/srv/StartDrag "{}" > /dev/null
        ros2 service call /dobot_bringup_v3/srv/StopDrag dobot_msgs_v3/srv/StopDrag "{}" > /dev/null
        
        # Modbus setup
        wait_for_service "/dobot_bringup_v3/srv/ModbusClose"
        ros2 service call /dobot_bringup_v3/srv/ModbusClose dobot_msgs_v3/srv/ModbusClose "{index: 0}" > /dev/null
        
        wait_for_service "/dobot_bringup_v3/srv/ModbusCreate"
        ros2 service call /dobot_bringup_v3/srv/ModbusCreate dobot_msgs_v3/srv/ModbusCreate \
            "{ip: \"127.0.0.1\", port: 60000, slave_id: 9, is_rtu: 1}" > /dev/null
        
        wait_for_service "/dobot_bringup_v3/srv/SetHoldRegs"
        ros2 service call /dobot_bringup_v3/srv/SetHoldRegs dobot_msgs_v3/srv/SetHoldRegs \
            "{index: 0, addr: 1000, count: 3, val_tab: \"0,0,0\", val_type: \"int\"}" > /dev/null
        
        ros2 service call /dobot_bringup_v3/srv/SetHoldRegs dobot_msgs_v3/srv/SetHoldRegs \
            "{index: 0, addr: 1000, count: 3, val_tab: \"256,0,0\", val_type: \"int\"}" > /dev/null
        
        # Launch MoveIt
        log "=== Launching MoveIt (headless) ==="
        ros2 launch dobot_moveit dobot_moveit.launch.py \
            use_rviz:=false debug:=false __log_level:=fatal &> /dev/null &
        MOVEIT_PID=$!
        
        sleep 2
        
        # Launch servo action server
        log "=== Launching servo_action server ==="
        ros2 run servo_action action_move_server_reality __log_level:=fatal &
        ACTION_SERVER_PID=$!
        
        # Launch Orbbec camera (with longer delay for Robot 2)
        log "=== Launching Orbbec camera (with delay) ==="
        ros2 launch orbbec_camera gemini_330_series.launch.py \
            camera_name:="${CAM_NAME}" \
            serial_number:="${CAMERA_SERIAL_NUMBER}" \
            usb_port:="${USB_PORT}" \
            device_num:="${DEVICE_NUM}" \
            connection_delay:="${ORBBEC_CONNECTION_DELAY}" \
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
        
        sleep 2
        
        # Launch perception nodes
        log "=== Launching perception nodes ==="
        ros2 run pickn_place pose_generator __log_level:=fatal &> /dev/null &
        POSE_GEN_PID=$!
        
        ros2 run pickn_place obstacle_generator __log_level:=fatal &> /dev/null &
        OBSTACLE_GEN_PID=$!
        
        # Launch OMS service
        log "=== Launching oms_v1.app service ==="
        cd src/oms_v1 && python -m oms_v1.app --service &
        OMS_APP_PID=$!
        
        cd ../..
        
        log "=== Robot 2 stack started successfully ==="
        log "Robot 2 is now ready and connected to RabbitMQ at ${RABBITMQ_URL}"
        
        # Keep the process running
        tail -f /dev/null
        
        # Cleanup on exit
        trap "kill $DOBOT_BRINGUP_PID $MOVEIT_PID $ACTION_SERVER_PID $CAMERA_PID $POSE_GEN_PID $OBSTACLE_GEN_PID $OMS_APP_PID" EXIT
    '
}

# Handle shutdown
cleanup() {
    log "Shutting down Robot 2..."
    # Kill any background processes
    pkill -f "ros2 launch" || true
    pkill -f "ros2 run" || true
    pkill -f "oms_v1.app" || true
}

# Main function
main() {
    log "Starting BARNS Robot 2 (Docker Host PC Mode)"
    log "Docker Host IP: $DOCKER_HOST_IP"
    log "Robot IP: $IP_ADDRESS"
    echo
    
    trap cleanup EXIT
    
    check_workspace
    check_docker_services
    wait_for_docker_services
    start_robot
}

# Handle command line arguments
case "${1:-start}" in
    start)
        main
        ;;
    check)
        log "Checking prerequisites..."
        check_workspace
        check_docker_services
        log "All prerequisites met!"
        ;;
    stop)
        log "Stopping Robot 2..."
        cleanup
        log "Robot 2 stopped successfully"
        ;;
    --help|-h)
        echo "BARNS Robot 2 Startup Script"
        echo
        echo "Usage: $0 [start|check|stop|--help]"
        echo
        echo "Commands:"
        echo "  start   Start Robot 2 (default)"
        echo "  check   Check if Docker services are ready"
        echo "  stop    Stop Robot 2"
        echo "  --help  Show this help"
        echo
        echo "Environment variables:"
        echo "  WORKSPACE_DIR           Robot workspace (default: \$HOME/barns_robot_ws)"
        echo "  DOCKER_HOST_IP          RabbitMQ connection IP (default: 127.0.0.1)"
        echo "  IP_ADDRESS              Robot IP address (default: 192.168.200.248)"
        echo "  CAMERA_SERIAL_NUMBER    Camera serial (default: CP1Z842000F6)"
        echo "  USB_PORT                Camera USB port (default: 2-7-6)"
        echo "  ROBOT_STARTUP_DELAY     Startup delay in seconds (default: 15)"
        echo
        echo "Prerequisites:"
        echo "  - Docker services must be running (start with start-barns.sh)"
        echo "  - Robot dependencies installed (run install-robot-dependencies.sh)"
        ;;
    *)
        error "Unknown command: $1. Use --help for usage information."
        ;;
esac 