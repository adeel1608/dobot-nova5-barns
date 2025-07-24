#!/usr/bin/env bash
# BARNS Robot Environment Setup
# Source this script before running robot processes

# Get the directory containing this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS 2 Environment
export ROS_DISTRO=humble
set +u 2>/dev/null || true
source /opt/ros/${ROS_DISTRO}/setup.bash
set -u 2>/dev/null || true

# Workspace
set +u 2>/dev/null || true
source "${SCRIPT_DIR}/install/setup.bash"
set -u 2>/dev/null || true

# Python path - add both the src directory and the repository root
export PYTHONPATH="${SCRIPT_DIR}/src:${SCRIPT_DIR}/../../../:${PYTHONPATH}"

# Environment optimizations for headless operation
export DISPLAY=${DISPLAY:-:99}
export QT_QPA_PLATFORM=offscreen
export ROS_LOG_LEVEL=WARN
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=WARN
export ROS_DISABLE_LOANED_MESSAGES=1
export MOVEIT_DISABLE_GUI=1
export RVIZ_DISABLE=1
export AMENT_TRACE_SETUP_FILES=0
export AMENT_PYTHON_EXECUTABLE=python3

echo "BARNS Robot environment loaded"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "Workspace: ${SCRIPT_DIR}"
echo "Repository root: ${SCRIPT_DIR}/../../../"
