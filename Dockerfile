FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install colcon, rosdep, and known system deps
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    libsdl2-dev \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Copy workspace
COPY . .

# Build
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install

# Source setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /ros2_ws/install/setup.bash \
    && cd /ros2_ws/src/startup && ros2 launch bot-launch.xml"]
