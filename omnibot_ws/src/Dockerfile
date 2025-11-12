# ===============================================================
# OmniBot Dockerfile — ROS 2 Jazzy (ARMv7 / Raspberry Pi 3B+)
# ===============================================================

# 1. Use ROS 2 Jazzy base image for ARMv7 (multi-arch compatible)
FROM ros:jazzy-ros-base

# 2. Set environment variables for noninteractive build
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Recife
ENV ROS_DISTRO=jazzy
ENV COLCON_HOME=/root/colcon_ws
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# 3. Update and install required system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-numpy \
    python3-opencv \
    libserial-dev \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-pluginlib \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-diff-drive-controller \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    ros-${ROS_DISTRO}-nav2-controller \
    ros-${ROS_DISTRO}-nav2-planner \
    ros-${ROS_DISTRO}-nav2-amcl \
    ros-${ROS_DISTRO}-nav2-behavior-tree \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \

    && rm -rf /var/lib/apt/lists/*

# 4. Upgrade pip and install Python packages used by OmniBot
RUN pip3 install --no-cache-dir \
    python3-numpy \
    python3-scipy \
    python3-serial \
    python3-pyyaml

# 5. Initialize rosdep
RUN rosdep init && rosdep update

# 6. Create workspace and clone OmniBot repository
WORKDIR /root/omnibot_ws/src
RUN git clone https://github.com/SmartFactoryLab-UFCG/OmniBot.git

# 7. Install dependencies via rosdep
WORKDIR /root/omnibot_ws
RUN rosdep install --from-paths src --ignore-src -r -y

# 8. Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 9. Source workspace on startup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/omnibot_ws/install/setup.bash" >> /root/.bashrc

# 10. Set default entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
