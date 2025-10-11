FROM ros:jazzy-ros-base

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

# Install curl
RUN apt install -y --no-install-recommends curl \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Install ROS package signing keys
RUN rm /usr/share/keyrings/ros-archive-keyring.gpg || true
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN rm /etc/apt/sources.list.d/ros2.list || true
RUN rm /usr/share/keyrings/ros-archive-keyring.gpg || true

# Install dependencies
RUN apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    libsdl2-dev \
    python3-pip \
    python3.12-venv \
    ninja-build \
    xauth \
    openssh-client \
    wget \
    at \
    # Required developer tool
    software-properties-common \
    # Image handling
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Create and switch to user
RUN groupadd -g $GID $USERNAME \
    && useradd -lm -u $UID -g $USERNAME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USERNAME

# Create workspace so that user own this directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws

# Setup bashrc
# RUN echo "source /home/$USERNAME/ros2_ws/.venv/bin/activate" >> /home/$USERNAME/.bashrc
## Make ros2 public to the system via `source` command
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc

## Check if own build has been made; if yes, source it
RUN echo "if [ -f \"/home/$USERNAME/ros2_ws/install/setup.bash\" ]; then" >> /home/$USERNAME/.bashrc \
    && echo "    source /home/$USERNAME/ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "else" >> /home/$USERNAME/.bashrc \
    && echo "    echo \"/home/$USERNAME/ros2_ws/install/setup.bash has not yet been created.\"" >> /home/$USERNAME/.bashrc \
    && echo "    echo \"Make sure to build your package(s).\"" >> /home/$USERNAME/.bashrc \
    && echo "fi" >> /home/$USERNAME/.bashrc

# Setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
