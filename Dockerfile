FROM osrf/ros:jazzy-desktop AS base

# Add the OSRF Gazebo Harmonic apt repository
RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
         -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
         http://packages.osrfoundation.org/gazebo/ubuntu-stable \
         $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
         > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Jazzy + Gazebo Harmonic dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb3-sim \
    ros-jazzy-slam-toolbox \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-teleop-twist-keyboard \
    gz-harmonic \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Python deps for future swarm nodes (comms FEC, math)
RUN pip3 install --no-cache-dir --break-system-packages numpy scipy crcmod reedsolo

WORKDIR /ws

# Copy only src so Docker layer cache survives non-src changes
COPY src/ src/

# Build only packages that have manifests; swarm_msgs is currently empty
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build \
        --packages-select swarm_exploration

# Default shell for sourced entrypoints
SHELL ["/bin/bash", "-c"]
