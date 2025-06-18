###############################
# Development image for OMS-robot_2
###############################
ARG ROS_DISTRO=humble
FROM ros:humble-ros-base

# Make sure later RUN steps see $ROS_DISTRO
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ---- 0. Install ROS 2 apt source tool ----
RUN rm /etc/apt/sources.list.d/ros2-latest.list && \
    apt update && apt install curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ---- 1. Build & run-time dependencies (APT) ----
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        procps \
        psmisc \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-rospkg-modules \
        python3-ament-package \
        python3-vcstool \
        ros-${ROS_DISTRO}-tf-transformations \
        python3-transforms3d \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        libomp-dev \
        nlohmann-json3-dev \
        freeglut3-dev \
        libgoogle-glog-dev \
        libdw-dev \
        python3-pygraphviz \
        python3-coverage \
        xvfb \
        libgtk-3-0 \
        libgl1-mesa-glx \
        libglib2.0-0 \
        libsm6 \
        libxrender1 \
        libxext6 \
        openbox \
        tigervnc-standalone-server \
    && rm -rf /var/lib/apt/lists/*

# ---- 2. Install exactly the pip wheels you need ----
RUN pip3 install --no-cache-dir \
        opencv-contrib-python==4.11.0.86 \
        numpy==1.23.5 \
        scipy==1.15.2 \
        transformations==2025.1.1

# ---- 3. Initialise rosdep (so container can run as non-root) ----
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
 && rosdep init \
 && rosdep update


# ---- 4. Create an empty workspace; user mounts actual source via docker-compose ----
RUN mkdir -p /root/ros_ws/src
WORKDIR /root/ros_ws

# ---- 5. Copy in the custom entrypoint that builds & then launches ROS ----
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
