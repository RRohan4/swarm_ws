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
    ros-jazzy-rmw-cyclonedds-cpp \
    gz-harmonic \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Python deps for swarm nodes
RUN pip3 install --no-cache-dir --break-system-packages numpy scipy

WORKDIR /ws

# ── Per-package COPY + build for fine-grained layer caching ──────────────────
# Each package gets its own layer: changing one package only invalidates
# that layer and any packages that depend on it downstream.

# swarm_msgs (ament_cmake — generates C++ headers, changes rarely)
COPY src/swarm_msgs/ src/swarm_msgs/
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-select swarm_msgs

# swarm_slam (ament_python — symlink-install means no recompile needed)
COPY src/swarm_slam/ src/swarm_slam/
RUN . /opt/ros/jazzy/setup.sh && . install/setup.sh \
    && colcon build --symlink-install --packages-select swarm_slam

# swarm_exploration (ament_python)
COPY src/swarm_exploration/ src/swarm_exploration/
RUN . /opt/ros/jazzy/setup.sh && . install/setup.sh \
    && colcon build --symlink-install --packages-select swarm_exploration

# swarm_bringup (ament_python — launch files and configs)
COPY src/swarm_bringup/ src/swarm_bringup/
RUN . /opt/ros/jazzy/setup.sh && . install/setup.sh \
    && colcon build --symlink-install --packages-select swarm_bringup

SHELL ["/bin/bash", "-c"]

# ── Regression-test stage ─────────────────────────────────────────────────────
# Run with: docker build --target test .
# Validates that every launch file parses cleanly and all ROS 2 entry points
# are importable.  Runs headlessly (no Gazebo process, no display needed) so
# it completes in seconds and is safe to use in CI.
FROM base AS test
RUN . /opt/ros/jazzy/setup.sh && . install/setup.sh \
    && ros2 launch swarm_bringup swarm.launch.py --show-args \
    && ros2 launch swarm_bringup robot_stack.launch.py --show-args \
    && ros2 launch swarm_bringup global.launch.py --show-args \
    && python3 -c "from swarm_slam.global_node import GlobalNode" \
    && python3 -c "from swarm_exploration.frontier_detector_node import FrontierDetectorNode" \
    && python3 -c "from swarm_exploration.robot_fsm_node import RobotFSMNode" \
    && echo "--- regression test passed ---"
