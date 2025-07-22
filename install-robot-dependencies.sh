#!/usr/bin/env bash
###############################
# BARNS Robot Dependencies Installation Script
# Replicates the setup from dev.Dockerfile for host installation
###############################

set -euo pipefail

# Default values
ROS_DISTRO=${ROS_DISTRO:-humble}
ORBBEC_SDK_VERSION=${ORBBEC_SDK_VERSION:-2.4.8}
INSTALL_DIR=${INSTALL_DIR:-/opt/barns-robot}
WORKSPACE_DIR=${WORKSPACE_DIR:-$HOME/barns_robot_ws}

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log() {
    echo -e "${GREEN}[INSTALL]${NC} $*"
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

# Check if running as root
check_root() {
    if [[ $EUID -eq 0 ]]; then
        error "This script should not be run as root. Run as a regular user with sudo privileges."
    fi
}

# Check Ubuntu version
check_ubuntu() {
    if ! command -v lsb_release &> /dev/null; then
        error "lsb_release not found. Are you running Ubuntu?"
    fi
    
    local version=$(lsb_release -rs)
    if [[ "$version" != "22.04" ]]; then
        warn "This script is designed for Ubuntu 22.04. You are running $version. Proceed with caution."
        read -p "Continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
}

# Install ROS 2 if not already installed
install_ros2() {
    log "Checking ROS 2 installation..."
    
    if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        info "ROS 2 ${ROS_DISTRO} already installed"
        return 0
    fi
    
    log "Installing ROS 2 ${ROS_DISTRO}..."
    
    # Add ROS 2 repository
    sudo apt update
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    
    # Add ROS 2 GPG key
    sudo apt update && sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    # Add repository to sources list
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Install ROS 2
    sudo apt update
    sudo apt install -y ros-${ROS_DISTRO}-ros-base
    
    log "ROS 2 ${ROS_DISTRO} installed successfully"
}

# Install APT dependencies
install_apt_dependencies() {
    log "Installing APT dependencies..."
    
    sudo apt update
    sudo apt install -y --no-install-recommends \
        build-essential \
        git \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-image-transport-plugins \
        ros-${ROS_DISTRO}-compressed-image-transport \
        ros-${ROS_DISTRO}-camera-info-manager \
        ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-diagnostic-msgs \
        ros-${ROS_DISTRO}-statistics-msgs \
        ros-${ROS_DISTRO}-tf-transformations \
        ros-${ROS_DISTRO}-kinematics-interface-kdl \
        ros-${ROS_DISTRO}-ros-testing \
        ros-${ROS_DISTRO}-launch-testing \
        ros-${ROS_DISTRO}-launch-testing-ament-cmake \
        ros-${ROS_DISTRO}-image-publisher \
        ros-${ROS_DISTRO}-backward-ros \
        libgflags-dev \
        nlohmann-json3-dev \
        libdw-dev \
        libomp-dev \
        freeglut3-dev \
        libgoogle-glog-dev \
        curl \
        ca-certificates \
        udev \
        xvfb \
        libgtk-3-0 \
        libgl1-mesa-glx \
        libglib2.0-0 \
        libsm6 \
        libxrender1 \
        libxext6 \
        lsusb \
        usbutils
    
    log "APT dependencies installed successfully"
}

# Install Python dependencies
install_python_dependencies() {
    log "Installing Python dependencies..."
    
    # Upgrade pip first
    python3 -m pip install --upgrade pip --user
    
    # Install required packages
    python3 -m pip install --no-cache-dir --user \
        opencv-contrib-python==4.10.0.84 \
        numpy==1.23.5 \
        scipy==1.11.4 \
        transformations==2025.1.1 \
        aio-pika==9.4.3 \
        pika==1.3.2
    
    # Create python symlink if it doesn't exist
    if ! command -v python &> /dev/null; then
        sudo ln -s /usr/bin/python3 /usr/bin/python
    fi
    
    log "Python dependencies installed successfully"
}

# Install Orbbec SDK
install_orbbec_sdk() {
    log "Installing Orbbec SDK v${ORBBEC_SDK_VERSION}..."
    
    # Check if already installed
    if [ -d "/opt/OrbbecSDK_v${ORBBEC_SDK_VERSION}" ]; then
        info "Orbbec SDK v${ORBBEC_SDK_VERSION} already installed"
        return 0
    fi
    
    local temp_dir=$(mktemp -d)
    cd "$temp_dir"
    
    # Try primary URL first, then fallback
    local primary_url="https://github.com/orbbec/OrbbecSDK_v2/releases/download/v${ORBBEC_SDK_VERSION}/OrbbecSDK_v${ORBBEC_SDK_VERSION}_amd64.deb"
    local fallback_url="https://github.com/orbbec/OrbbecSDK_v2/releases/download/v${ORBBEC_SDK_VERSION}/OrbbecSDK_v${ORBBEC_SDK_VERSION}_Ubuntu22.04_amd64.deb"
    
    if ! curl -Lf --retry 3 --retry-delay 2 -o sdk.deb "$primary_url"; then
        log "Primary URL failed, trying fallback..."
        curl -Lf --retry 3 --retry-delay 2 -o sdk.deb "$fallback_url"
    fi
    
    # Verify package
    dpkg -I sdk.deb > /dev/null
    
    # Install package
    sudo apt install -y ./sdk.deb
    
    # Create compatibility symlink
    local libdir="/opt/OrbbecSDK_v${ORBBEC_SDK_VERSION}/lib"
    if [ -f "$libdir/libOrbbecSDK.so" ]; then
        sudo mv "$libdir/libOrbbecSDK.so" "$libdir/libOrbbecSDK_C.so"
    fi
    sudo ln -sf "$libdir/libobsensor.so" "$libdir/libOrbbecSDK.so"
    
    # Update ldconfig
    echo "/opt/OrbbecSDK_v${ORBBEC_SDK_VERSION}/lib" | sudo tee /etc/ld.so.conf.d/orbbec.conf
    sudo ldconfig
    
    # Cleanup
    cd - > /dev/null
    rm -rf "$temp_dir"
    
    log "Orbbec SDK v${ORBBEC_SDK_VERSION} installed successfully"
}

# Initialize rosdep
initialize_rosdep() {
    log "Initializing rosdep..."
    
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
    
    log "rosdep initialized successfully"
}

# Create workspace structure
create_workspace() {
    log "Creating workspace at ${WORKSPACE_DIR}..."
    
    mkdir -p "${WORKSPACE_DIR}/src"
    
    # Copy robot container source code if available
    if [ -d "$(dirname "$0")/services/robot_container/ros_ws/src" ]; then
        info "Copying robot source code from repository..."
        cp -r "$(dirname "$0")/services/robot_container/ros_ws/src"/* "${WORKSPACE_DIR}/src/"
    else
        warn "Robot source code not found. You'll need to copy it manually."
        info "Expected location: services/robot_container/ros_ws/src"
    fi
    
    log "Workspace created at ${WORKSPACE_DIR}"
}

# Build workspace
build_workspace() {
    log "Building ROS workspace..."
    
    cd "${WORKSPACE_DIR}"
    
    # Source ROS 2
    source /opt/ros/${ROS_DISTRO}/setup.bash
    
    # Install dependencies
    rosdep install --from-paths src --ignore-src -y -q --rosdistro "${ROS_DISTRO}" || true
    
    # Build workspace
    colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DORBBEC_SDK_ROOT=/opt/OrbbecSDK_v${ORBBEC_SDK_VERSION} \
        --continue-on-error \
        --parallel-workers $(nproc)
    
    log "Workspace built successfully"
}

# Install udev rules
install_udev_rules() {
    log "Installing udev rules for Orbbec cameras..."
    
    local rules_file="99-obsensor-libusb.rules"
    local source_path="${WORKSPACE_DIR}/src/OrbbecSDK_ROS2/orbbec_camera/scripts/${rules_file}"
    
    if [ -f "$source_path" ]; then
        sudo cp "$source_path" /etc/udev/rules.d/
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        log "udev rules installed successfully"
    else
        warn "udev rules file not found at $source_path"
        info "You may need to install udev rules manually for camera access"
    fi
}

# Create environment script
create_environment_script() {
    log "Creating environment setup script..."
    
    local env_script="${WORKSPACE_DIR}/setup_robot_env.sh"
    
    cat > "$env_script" << EOF
#!/usr/bin/env bash
# BARNS Robot Environment Setup
# Source this script before running robot processes

# ROS 2 Environment
export ROS_DISTRO=${ROS_DISTRO}
source /opt/ros/\${ROS_DISTRO}/setup.bash

# Workspace
source ${WORKSPACE_DIR}/install/setup.bash

# Orbbec SDK
export LD_LIBRARY_PATH=/opt/OrbbecSDK_v${ORBBEC_SDK_VERSION}/lib:\${LD_LIBRARY_PATH}

# Python path
export PYTHONPATH=${WORKSPACE_DIR}/src:\${PYTHONPATH}

# Environment optimizations for headless operation
export DISPLAY=\${DISPLAY:-:99}
export QT_QPA_PLATFORM=offscreen
export ROS_LOG_LEVEL=WARN
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=WARN
export ROS_DISABLE_LOANED_MESSAGES=1
export MOVEIT_DISABLE_GUI=1
export RVIZ_DISABLE=1
export AMENT_TRACE_SETUP_FILES=0
export AMENT_PYTHON_EXECUTABLE=python3

echo "BARNS Robot environment loaded"
echo "ROS_DISTRO: \$ROS_DISTRO"
echo "Workspace: ${WORKSPACE_DIR}"
echo "Orbbec SDK: /opt/OrbbecSDK_v${ORBBEC_SDK_VERSION}"
EOF

    chmod +x "$env_script"
    
    log "Environment setup script created at $env_script"
    info "Source this script before running robot processes: source $env_script"
}

# Main installation function
main() {
    log "Starting BARNS Robot Dependencies Installation"
    log "ROS Distro: $ROS_DISTRO"
    log "Orbbec SDK Version: $ORBBEC_SDK_VERSION"
    log "Workspace Directory: $WORKSPACE_DIR"
    echo
    
    check_root
    check_ubuntu
    
    install_ros2
    install_apt_dependencies
    install_python_dependencies
    install_orbbec_sdk
    initialize_rosdep
    create_workspace
    build_workspace
    install_udev_rules
    create_environment_script
    
    log "Installation completed successfully!"
    echo
    info "Next steps:"
    info "1. Source the environment: source ${WORKSPACE_DIR}/setup_robot_env.sh"
    info "2. Use the robot startup scripts (robot1-startup.sh or robot2-startup.sh)"
    info "3. Make sure your robot hardware is connected"
    echo
    warn "You may need to reboot for udev rules to take full effect"
}

# Handle command line arguments
case "${1:-install}" in
    install)
        main
        ;;
    --help|-h)
        echo "BARNS Robot Dependencies Installation Script"
        echo
        echo "Usage: $0 [install|--help]"
        echo
        echo "Environment variables:"
        echo "  ROS_DISTRO          ROS 2 distribution (default: humble)"
        echo "  ORBBEC_SDK_VERSION  Orbbec SDK version (default: 2.4.8)"
        echo "  WORKSPACE_DIR       Workspace directory (default: \$HOME/barns_robot_ws)"
        echo
        echo "Example:"
        echo "  ROS_DISTRO=humble WORKSPACE_DIR=/opt/barns ./install-robot-dependencies.sh"
        ;;
    *)
        error "Unknown command: $1. Use --help for usage information."
        ;;
esac 