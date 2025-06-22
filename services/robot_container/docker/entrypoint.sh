#!/usr/bin/env bash
set -e

# ───────────────────────────────────────────────────────────────────
# 2) Source ROS 2 from the base image
# ───────────────────────────────────────────────────────────────────

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash

# ───────────────────────────────────────────────────────────────────
# 3) Change into your workspace and fix script permissions
# ───────────────────────────────────────────────────────────────────

# Fix permissions if needed
if [ ! -r /root/ros_ws ] || [ ! -x /root/ros_ws ]; then
    echo "Fixing permissions for /root/ros_ws..."
    chown -R root:root /root/ros_ws 2>/dev/null || true
    chmod -R 755 /root/ros_ws 2>/dev/null || true
fi

cd /root/ros_ws

# Fix permissions for the run_full_stack.sh script
if [ -f "/root/ros_ws/run_full_stack.sh" ]; then
    chmod +x /root/ros_ws/run_full_stack.sh
fi

# ───────────────────────────────────────────────────────────────────
# 4) Refresh apt so rosdep can install any missing deps
# ───────────────────────────────────────────────────────────────────

apt-get update

# ───────────────────────────────────────────────────────────────────
# 5) Update rosdep's database
# ───────────────────────────────────────────────────────────────────

rosdep update

# ───────────────────────────────────────────────────────────────────
# 6) Install all Debian dependencies in your workspace (if src exists)
#    Skip ament_python if it still shows up anywhere
# ───────────────────────────────────────────────────────────────────

if [ -d "src" ] && [ "$(ls -A src 2>/dev/null)" ]; then
    echo "Installing ROS dependencies from src directory..."
    rosdep install \
      --from-paths src \
      --ignore-src \
      -r \
      -y \
      --skip-keys ament_python
else
    echo "No src directory found or empty - skipping rosdep install"
fi

# ───────────────────────────────────────────────────────────────────
# 7) Build the entire ROS 2 workspace
# ───────────────────────────────────────────────────────────────────

if [ -d "src" ] && [ "$(ls -A src 2>/dev/null)" ]; then
    echo "Building ROS workspace..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
else
    echo "No src directory found or empty - skipping colcon build"
fi

# ───────────────────────────────────────────────────────────────────
# 8) Source the newly-built overlay so launches can see your packages
# ───────────────────────────────────────────────────────────────────

if [ -f "install/setup.bash" ]; then
    echo "Sourcing ROS workspace overlay"
    source install/setup.bash
else
    echo "No built overlay found, using base ROS installation"
fi

# ───────────────────────────────────────────────────────────────────
# 9) Finally, run the command passed into the container (e.g. "ros2 launch …")
# ───────────────────────────────────────────────────────────────────

exec "$@"
