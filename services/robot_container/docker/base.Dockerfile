# ───────────────────────── docker/base.Dockerfile ─────────────────────────
FROM ubuntu:22.04

ARG ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ─── Core OS tools ────────────────────────────────────────────────────────
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        curl gnupg lsb-release software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN rm -f /etc/apt/sources.list.d/ros2.list
# ─── ROS 2 key & repo -----------------------------------------------------
RUN mkdir -p /usr/share/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros2-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] \
         http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list

# ─── ROS 2 desktop + colcon ------------------------------------------------
RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-desktop-full python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*
# ─── Create empty overlay workspace ---------------------------------------
RUN mkdir -p /root/ros_ws/src
WORKDIR /root/ros_ws

# Source ROS for every shell
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc
