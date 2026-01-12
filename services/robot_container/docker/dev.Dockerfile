###############################
# --- BARNS Robot Container (Single-Stage Optimized) -----------
###############################
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

# Environment ------------------------------------------------------
ARG ROS_DISTRO
ENV \
  DEBIAN_FRONTEND=noninteractive \
  ROS_DISTRO=${ROS_DISTRO}

SHELL ["/bin/bash", "-c"]

# APT deps ---------------------------------------------------------
# ① keep one package per line (git diff friendly)
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential              \
        git                          \
        python3-pip                  \
        python3-colcon-common-extensions \
        python3-rosdep               \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-moveit      \
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
        libgflags-dev               \
        nlohmann-json3-dev          \
        libdw-dev                   \
        libomp-dev                  \
        freeglut3-dev               \
        libgoogle-glog-dev          \
        curl ca-certificates udev   \
        xvfb libgtk-3-0 libgl1-mesa-glx libglib2.0-0 libsm6 libxrender1 libxext6 \
        && rm -rf /var/lib/apt/lists/*

# PIP deps (pin only where it matters) -----------------------------
# First upgrade pip to latest version
RUN pip3 install --upgrade pip

# Then install required packages (versions compatible with Python 3.10)
RUN pip3 install --no-cache-dir \
        opencv-contrib-python==4.10.0.84 \
        numpy==1.23.5 \
        scipy==1.11.4 \
        transformations==2025.1.1 \
        aio-pika==9.4.3 \
        pika==1.3.2

# Create python symlink for legacy compatibility
RUN ln -s /usr/bin/python3 /usr/bin/python

# ---- Install Orbbec SDK BEFORE building orbbec_camera ----------
ARG  ORBBEC_SDK_VERSION=2.4.8
ARG  ORBBEC_SDK_URL_PRIMARY=https://github.com/orbbec/OrbbecSDK_v2/releases/download/v${ORBBEC_SDK_VERSION}/OrbbecSDK_v${ORBBEC_SDK_VERSION}_arm64.deb
ARG  ORBBEC_SDK_URL_FALLBACK=https://github.com/orbbec/OrbbecSDK_v2/releases/download/v${ORBBEC_SDK_VERSION}/OrbbecSDK_v${ORBBEC_SDK_VERSION}_Ubuntu22.04_arm64.deb

RUN set -e \
  && mkdir -p /etc/udev/rules.d \
  # ── download .deb (primary → fallback) ───────────────────────
  && (curl -Lf --retry 3 --retry-delay 2 -o /tmp/sdk.deb "$ORBBEC_SDK_URL_PRIMARY" \
      || curl -Lf --retry 3 --retry-delay 2 -o /tmp/sdk.deb "$ORBBEC_SDK_URL_FALLBACK") \
  && dpkg -I /tmp/sdk.deb > /dev/null        \
  && apt-get install -y /tmp/sdk.deb         \
  && rm /tmp/sdk.deb                         \
  # ── COMPAT SHIM  ─────────────────────────────────────────────
  # libobsensor.so contains all C++ symbols; many ROS wrappers
  # still link against -lOrbbecSDK.  Overwrite the thin C shim
  # with a symlink to libobsensor.so so linking succeeds everywhere.
  && LIBDIR=/opt/OrbbecSDK_v${ORBBEC_SDK_VERSION}/lib \
  && if [ -f $LIBDIR/libOrbbecSDK.so ]; then mv -f $LIBDIR/libOrbbecSDK.so $LIBDIR/libOrbbecSDK_C.so; fi \
  && ln -s $LIBDIR/libobsensor.so $LIBDIR/libOrbbecSDK.so \
  && ldconfig

ENV LD_LIBRARY_PATH=/opt/OrbbecSDK_v${ORBBEC_SDK_VERSION}/lib:${LD_LIBRARY_PATH}

# rosdep -----------------------------------------------------------
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
     fi && \
     rosdep update

# Create workspace structure --------------------------------------
RUN mkdir -p /root/ros_ws/src
WORKDIR /root/ros_ws

# Copy entrypoint early (rarely changes) --------------------------
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Copy run script (rarely changes) --------------------------------
COPY ros_ws/run_full_stack.sh /root/ros_ws/run_full_stack.sh
RUN chmod +x /root/ros_ws/run_full_stack.sh

# Install udev rules for Orbbec cameras (critical for USB device access)
# Copy only the specific file needed instead of entire source
COPY ros_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/99-obsensor-libusb.rules /tmp/99-obsensor-libusb.rules
RUN cp /tmp/99-obsensor-libusb.rules /etc/udev/rules.d/99-obsensor-libusb.rules && \
    rm /tmp/99-obsensor-libusb.rules && \
    echo "udev rules installed at /etc/udev/rules.d/99-obsensor-libusb.rules"

# Copy source code (this layer will be invalidated when source changes)
COPY ros_ws/src /root/ros_ws/src

# Install rosdep dependencies and build in one layer to reduce layers
RUN rosdep update --rosdistro="${ROS_DISTRO}" -q || true && \
    rosdep install --from-paths src --ignore-src -y -q --skip-keys " ament_clang_format ament_lint_auto ament_lint_common\ \
    --rosdistro "${ROS_DISTRO}" || true && \
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DORBBEC_SDK_ROOT=/opt/OrbbecSDK_v${ORBBEC_SDK_VERSION} \
    --continue-on-error \
    --parallel-workers $(nproc)"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
