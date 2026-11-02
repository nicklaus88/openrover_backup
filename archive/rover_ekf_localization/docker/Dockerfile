FROM ros:humble-perception-jammy

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=111

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_LOCALHOST_ONLY=0


# ============================================================
# Fix Ubuntu ARM mirror / caching issues
# ============================================================

RUN sed -i \
    's|http://ports.ubuntu.com/ubuntu-ports|https://ports.ubuntu.com/ubuntu-ports|g' \
    /etc/apt/sources.list \
    && printf 'Acquire::Retries "5";\nAcquire::https::No-Cache "true";\n' \
       > /etc/apt/apt.conf.d/80-retries


# ============================================================
# Clear existing apt lists before downloading fresh indexes
# ============================================================

RUN rm -rf /var/lib/apt/lists/* \
    && apt-get clean \
    && apt-get update


# ============================================================
# Development + ROS + TurtleBot 4 packages
# ============================================================

RUN apt-get install -y --no-install-recommends \
    sudo \
    git \
    wget \
    curl \
    nano \
    vim \
    bash-completion \
    build-essential \
    cmake \
    gdb \
    pkg-config \
    python3-pip \
    python3-dev \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    python3-colcon-common-extensions \
    iproute2 \
    net-tools \
    iputils-ping \
    dnsutils \
    usbutils \
    udev \
    libusb-1.0-0-dev \
    \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    \
    ros-humble-turtlebot4-desktop \
    ros-humble-turtlebot4-description \
    ros-humble-turtlebot4-msgs \
    ros-humble-turtlebot4-navigation \
    ros-humble-turtlebot4-node \
    \
    ros-humble-irobot-create-msgs \
    \
    ros-humble-depthai-ros \
    ros-humble-rplidar-ros \
    \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    \
    # robot_localization supplies the EKF that fuses the commanded-velocity
    # odometry with rf2o scan matching (odom_source:=fused). Genuinely new.
    #
    # laser_filters already resolved transitively (navigation2 pulls it in),
    # so this line changes nothing today -- but launch/localization.py has
    # called scan_to_scan_filter_chain directly since the rf2o path was
    # added, and no package.xml in either workspace declares it. Naming it
    # here means a future dependency shuffle upstream cannot silently
    # remove it and break the default launch.
    ros-humble-robot-localization \
    ros-humble-laser-filters \
    \
    ros-humble-tf2-tools \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    \
    ros-humble-image-transport-plugins \
    ros-humble-camera-info-manager \
    \
    ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy \
    ros-humble-joy \
    \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rosbag2-storage-mcap \
    \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# Create user matching Jetson host UID/GID
# ============================================================

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd \
        --uid ${USER_UID} \
        --gid ${USER_GID} \
        --create-home \
        --shell /bin/bash \
        ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" \
        > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && usermod -aG video ${USERNAME} \
    && groupmod -g 993 render
    # 'video' group owns /dev/nvmap and the /dev/nvhost-* nodes -- without
    # it, torch.cuda / any Tegra GPU access fails with
    # "NvRmMemInitNvmap failed: error Permission denied".
    #
    # The render renumber is cosmetic ONLY. What actually grants GPU
    # access is compose.yaml's `group_add: "993"`, which is numeric
    # because the kernel checks the number on the host's bind-mounted
    # /dev (same reasoning as dialout there). Jammy ships render at GID
    # 110, so an unnamed 993 makes every shell greet you with
    # "groups: cannot find name for group ID 993"; renumbering the
    # image's own render group to the host's 993 gives that GID a name
    # and silences it. Nothing in the image uses GID 110.


# ============================================================
# rosdep
# ============================================================

RUN rosdep init || true


# ============================================================
# Workspace
# ============================================================

RUN mkdir -p /workspace/ros2_ws/src \
    && mkdir -p /workspace/ROS2-WorkSpace \
    && chown -R ${USERNAME}:${USERNAME} /workspace


USER ${USERNAME}


# ============================================================
# Jetson GPU python stack (pose_detector_node: torch + ultralytics)
#
# Requires internet at build time (downloads from PyPI). Versions pinned
# to exactly what was validated working on this Orin/L4T R39 box,
# including running real YOLO GPU inference -- see
# ros2_ws/src/person_follower/README.md "Known environment gotchas".
#
# Order matters:
#   1. torch/torchvision/ultralytics first -- installing them upgrades
#      user-site setuptools as a side effect.
#   2. packaging upgraded to match that newer setuptools, or colcon's
#      ament_cmake_python egg-info step breaks
#      (canonicalize_version() unexpected kwarg 'strip_trailing_zero').
#   3. numpy pinned back to <2 LAST -- ultralytics/opencv-python pull in
#      numpy>=2, which breaks cv_bridge's compiled extension (built
#      against apt's numpy 1.x, raises "AttributeError: _ARRAY_API not
#      found" on import).
# ============================================================

RUN pip3 install --user --no-cache-dir \
        torch==2.13.0 \
        torchvision==0.28.0 \
    && pip3 install --user --no-cache-dir ultralytics==8.4.119 \
    && pip3 install --user --no-cache-dir --upgrade "packaging>=23" \
    && pip3 install --user --no-cache-dir "numpy<2"


# ============================================================
# Entrypoint
#
# Deliberately last: everything above it (notably the torch/ultralytics
# layer, a multi-GB download on ARM) stays cached when only the
# entrypoint changes.
# ============================================================

USER root

COPY entrypoint.sh /entrypoint.sh

COPY ros_env.sh /ros_env.sh

RUN chmod +x /entrypoint.sh /ros_env.sh

USER ${USERNAME}


# ============================================================
# Interactive shells
#
# `docker compose exec ros2 bash` does NOT run /entrypoint.sh, so an exec'd
# shell would otherwise start with no ROS on the PATH and no discovery
# config. Source the same /ros_env.sh the entrypoint uses -- one file, so
# the two paths cannot drift apart.
#
# ros_discovery_server_start is idempotent (a single pgrep when the server
# is already up). It is called here so a shell still gets a working server
# if the one from the entrypoint died.
# ============================================================

RUN printf '%s\n' \
    '' \
    '# --- ROS 2 environment (shared with /entrypoint.sh) ---' \
    'if [ -f /ros_env.sh ]; then' \
    '    source /ros_env.sh' \
    '    ros_discovery_server_start || true' \
    'fi' \
    >> /home/${USERNAME}/.bashrc


WORKDIR /workspace/ROS2-WorkSpace

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]