#!/usr/bin/env bash
set -e

# ───────────────────────────────────────────────────────────────────
# 2) Source ROS 2 from the base image
# ───────────────────────────────────────────────────────────────────

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash

# ───────────────────────────────────────────────────────────────────
# 3) Change into your workspace
# ───────────────────────────────────────────────────────────────────

cd /root/ros_ws

# ───────────────────────────────────────────────────────────────────
# 4) Refresh apt so rosdep can install any missing deps
# ───────────────────────────────────────────────────────────────────

apt-get update

# ───────────────────────────────────────────────────────────────────
# 5) Update rosdep’s database
# ───────────────────────────────────────────────────────────────────

rosdep update

# ───────────────────────────────────────────────────────────────────
# 6) Install all Debian dependencies in your workspace
#    Skip ament_python if it still shows up anywhere
# ───────────────────────────────────────────────────────────────────

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y \
  --skip-keys ament_python

# ───────────────────────────────────────────────────────────────────
# 7) Build the entire ROS 2 workspace
# ───────────────────────────────────────────────────────────────────

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ───────────────────────────────────────────────────────────────────
# 8) Source the newly-built overlay so launches can see your packages
# ───────────────────────────────────────────────────────────────────

source install/setup.bash

# ───────────────────────────────────────────────────────────────────
# 9) Finally, run the command passed into the container (e.g. "ros2 launch …")
# ───────────────────────────────────────────────────────────────────

exec "$@"
