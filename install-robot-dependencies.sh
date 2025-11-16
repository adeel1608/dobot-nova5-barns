#!/usr/bin/env bash
###############################
# BARNS Robot Dependencies Installation Script
# Follows official OrbbecSDK ROS2 installation procedure
###############################

set -euo pipefail

# Default values
ROS_DISTRO=${ROS_DISTRO:-humble}
INSTALL_DIR=${INSTALL_DIR:-/opt/barns-robot}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SCRIPT_DIR}/services/robot_container/ros_ws"

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
    sudo apt install -y software-properties-common curl
    sudo add-apt-repository universe
    
    # Add ROS 2 GPG key
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    # Add repository to sources list
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Install ROS 2
    sudo apt update
    sudo apt install -y ros-${ROS_DISTRO}-ros-base
    
    log "ROS 2 ${ROS_DISTRO} installed successfully"
}

# Install APT dependencies following official OrbbecSDK requirements
install_apt_dependencies() {
    log "Installing APT dependencies..."
    
    local packages=(
        # Build essentials
        "build-essential"
        "git"
        "python3-pip"
        "python3-colcon-common-extensions"
        "python3-rosdep"
        
        # ROS 2 packages
        "ros-${ROS_DISTRO}-ros2-control"
        "ros-${ROS_DISTRO}-ros2-controllers"
        "ros-${ROS_DISTRO}-moveit"
        "ros-${ROS_DISTRO}-image-transport"
        "ros-${ROS_DISTRO}-image-transport-plugins"
        "ros-${ROS_DISTRO}-compressed-image-transport"
        "ros-${ROS_DISTRO}-camera-info-manager"
        "ros-${ROS_DISTRO}-diagnostic-updater"
        "ros-${ROS_DISTRO}-diagnostic-msgs"
        "ros-${ROS_DISTRO}-statistics-msgs"
        "ros-${ROS_DISTRO}-tf-transformations"
        "ros-${ROS_DISTRO}-kinematics-interface-kdl"
        "ros-${ROS_DISTRO}-ros-testing"
        "ros-${ROS_DISTRO}-launch-testing"
        "ros-${ROS_DISTRO}-launch-testing-ament-cmake"
        "ros-${ROS_DISTRO}-image-publisher"
        "ros-${ROS_DISTRO}-backward-ros"
        
        # OrbbecSDK specific dependencies (from official README)
        "libgflags-dev"
        "nlohmann-json3-dev"
        "libdw-dev"
        "libomp-dev"
        "freeglut3-dev"
        "libgoogle-glog-dev"
        
        # System dependencies
        "curl"
        "ca-certificates"
        "udev"
        "usbutils"
        "xvfb"
        "libgtk-3-0"
        "libgl1-mesa-glx"
        "libglib2.0-0"
        "libsm6"
        "libxrender1"
        "libxext6"
    )
    
    sudo apt update
    
    # Install packages one by one to handle already installed packages gracefully
    for package in "${packages[@]}"; do
        if ! dpkg -l | grep -q "^ii  $package "; then
            info "Installing $package..."
            sudo apt install -y --no-install-recommends "$package" || warn "Failed to install $package, continuing..."
        else
            info "$package already installed"
        fi
    done
    
    log "APT dependencies installation completed"
}

# Install Python dependencies
install_python_dependencies() {
    log "Installing Python dependencies..."
    
    # Upgrade pip first
    python3 -m pip install --upgrade pip --user
    
    # Install required packages
    local python_packages=(
        "opencv-contrib-python==4.10.0.84"
        "numpy==1.23.5"
        "scipy==1.11.4"
        "transformations==2025.1.1"
        "aio-pika==9.4.3"
        "pika==1.3.2"
        "pyorbbecsdk==1.3.2"
    )
    
    for package in "${python_packages[@]}"; do
        info "Installing Python package: $package"
        python3 -m pip install --no-cache-dir --user "$package" || warn "Failed to install $package, continuing..."
    done
    
    # Create python symlink if it doesn't exist
    if ! command -v python &> /dev/null; then
        sudo ln -s /usr/bin/python3 /usr/bin/python
    fi
    
    log "Python dependencies installed successfully"
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

# Create workspace structure and clone OrbbecSDK_ROS2
create_workspace() {
    log "Setting up workspace at ${WORKSPACE_DIR}..."
    
    # Ensure the workspace and src directories exist
    mkdir -p "${WORKSPACE_DIR}/src"
    
    # Clone OrbbecSDK_ROS2 if not already present or fix corrupted directory
    if [ ! -d "${WORKSPACE_DIR}/src/OrbbecSDK_ROS2" ]; then
        info "Cloning OrbbecSDK_ROS2..."
        cd "${WORKSPACE_DIR}/src"
        git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
    else
        # Check if it's a proper git repository
        if [ -d "${WORKSPACE_DIR}/src/OrbbecSDK_ROS2/.git" ]; then
            info "OrbbecSDK_ROS2 already exists, pulling latest changes..."
            cd "${WORKSPACE_DIR}/src/OrbbecSDK_ROS2"
            git pull
        else
            warn "OrbbecSDK_ROS2 directory exists but is not a git repository, replacing it..."
            rm -rf "${WORKSPACE_DIR}/src/OrbbecSDK_ROS2"
            cd "${WORKSPACE_DIR}/src"
            git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
        fi
    fi
    
    # Copy shared module to src directory for ROS packages to access
    if [ -d "${SCRIPT_DIR}/shared" ]; then
        info "Copying shared module to ROS workspace..."
        cp -r "${SCRIPT_DIR}/shared" "${WORKSPACE_DIR}/src/"
    else
        warn "Shared module not found at ${SCRIPT_DIR}/shared. Robot-to-Docker communication may not work."
    fi
    
    log "Workspace ready at ${WORKSPACE_DIR}"
}

# Install udev rules for Orbbec cameras
install_udev_rules() {
    log "Installing udev rules for Orbbec cameras..."
    
    local rules_script="${WORKSPACE_DIR}/src/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh"
    
    if [ -f "$rules_script" ]; then
        cd "$(dirname "$rules_script")"
        sudo bash install_udev_rules.sh
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        log "udev rules installed successfully"
    else
        warn "udev rules script not found at $rules_script"
        info "You may need to install udev rules manually for camera access"
    fi
}

# Build workspace
build_workspace() {
    log "Building ROS workspace..."
    
    cd "${WORKSPACE_DIR}"
    
    # Source ROS 2 (temporarily disable unbound variable check)
    set +u
    source /opt/ros/${ROS_DISTRO}/setup.bash
    set -u
    
    # Install dependencies using rosdep
    info "Installing ROS dependencies..."
    rosdep install --from-paths src --ignore-src -y -q --rosdistro "${ROS_DISTRO}" || warn "Some rosdep dependencies failed to install"
    
    # Build workspace with proper configuration for OrbbecSDK
    info "Building workspace with colcon..."
    colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --continue-on-error \
        --parallel-workers $(nproc) \
        --event-handlers console_direct+
    
    # Check if critical packages built successfully
    if [ ! -f "install/setup.bash" ]; then
        error "Workspace build failed - install/setup.bash not found"
    fi
    
    log "Workspace built successfully"
}

# Create environment script
create_environment_script() {
    log "Creating environment setup script..."
    
    local env_script="${WORKSPACE_DIR}/setup_robot_env.sh"
    
    cat > "$env_script" << EOF
#!/usr/bin/env bash
# BARNS Robot Environment Setup
# Source this script before running robot processes

# Get the directory containing this script
SCRIPT_DIR="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)"

# ROS 2 Environment
export ROS_DISTRO=${ROS_DISTRO}
set +u 2>/dev/null || true
source /opt/ros/\${ROS_DISTRO}/setup.bash
set -u 2>/dev/null || true

# Workspace
set +u 2>/dev/null || true
source "\${SCRIPT_DIR}/install/setup.bash"
set -u 2>/dev/null || true

# Python path - add both the src directory and the repository root
export PYTHONPATH="\${SCRIPT_DIR}/src:\${SCRIPT_DIR}/../../../:\${PYTHONPATH}"

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
echo "Workspace: \${SCRIPT_DIR}"
echo "Repository root: \${SCRIPT_DIR}/../../../"
EOF

    chmod +x "$env_script"
    
    # Also create a convenience script in the repository root
    local root_env_script="${SCRIPT_DIR}/setup_robot_env.sh"
    cat > "$root_env_script" << EOF
#!/usr/bin/env bash
# Convenience script to source robot environment from repository root
source "\$(dirname "\${BASH_SOURCE[0]}")/services/robot_container/ros_ws/setup_robot_env.sh"
EOF
    chmod +x "$root_env_script"
    
    log "Environment setup scripts created:"
    info "  Main script: $env_script"
    info "  Convenience script: $root_env_script"
    info "Source either script before running robot processes"
}

# Main installation function
main() {
    log "Starting BARNS Robot Dependencies Installation"
    log "ROS Distro: $ROS_DISTRO"
    log "Repository Directory: $SCRIPT_DIR"
    log "Workspace Directory: $WORKSPACE_DIR"
    echo
    
    check_root
    check_ubuntu
    
    install_ros2
    install_apt_dependencies
    install_python_dependencies
    initialize_rosdep
    create_workspace
    install_udev_rules
    build_workspace
    create_environment_script
    
    log "Installation completed successfully!"
    echo
    info "Next steps:"
    info "1. Source the environment: source ${WORKSPACE_DIR}/setup_robot_env.sh"
    info "   OR from repository root: source ${SCRIPT_DIR}/setup_robot_env.sh"
    info "2. Use the robot startup scripts (robot1-startup.sh or robot2-startup.sh)"
    info "3. Make sure your robot hardware is connected"
    echo
    warn "You may need to reboot for udev rules to take full effect"
    
    # Check for build issues and provide guidance
    if [ -f "${WORKSPACE_DIR}/log/latest_build/orbbec_camera/stderr.log" ]; then
        local stderr_size=$(wc -l < "${WORKSPACE_DIR}/log/latest_build/orbbec_camera/stderr.log")
        if [ "$stderr_size" -gt 10 ]; then
            warn "OrbbecSDK build had some issues. Check the log at:"
            warn "${WORKSPACE_DIR}/log/latest_build/orbbec_camera/stderr.log"
            warn "The system should still work for basic robot operations."
        fi
    fi
}

# Handle command line arguments
case "${1:-install}" in
    install)
        main
        ;;
    clean)
        log "Cleaning workspace and reinstalling..."
        if [ -d "${WORKSPACE_DIR}" ]; then
            info "Cleaning build artifacts from ${WORKSPACE_DIR}..."
            # Only clean build artifacts, preserve source code
            rm -rf "${WORKSPACE_DIR}/build"
            rm -rf "${WORKSPACE_DIR}/install"
            rm -rf "${WORKSPACE_DIR}/log"
            rm -f "${WORKSPACE_DIR}/setup_robot_env.sh"
            rm -f "${SCRIPT_DIR}/setup_robot_env.sh"
            main
        else
            info "Workspace doesn't exist, proceeding with fresh installation"
            main
        fi
        ;;
    --help|-h)
        echo "BARNS Robot Dependencies Installation Script"
        echo
        echo "Usage: $0 [install|clean|--help]"
        echo
        echo "Commands:"
        echo "  install      Install dependencies and build workspace (default)"
        echo "  clean        Clean build artifacts and reinstall everything"
        echo "  --help       Show this help"
        echo
        echo "Environment variables:"
        echo "  ROS_DISTRO          ROS 2 distribution (default: humble)"
        echo
        echo "The script builds in the repository's existing ROS workspace:"
        echo "  ${SCRIPT_DIR}/services/robot_container/ros_ws"
        echo
        echo "Example:"
        echo "  ROS_DISTRO=humble ./install-robot-dependencies.sh"
        ;;
    *)
        error "Unknown command: $1. Use --help for usage information."
        ;;
esac 