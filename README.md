# Swarm Simulator Headless Gazebo + Foxglove

This workspace includes a ROS 2 launch file that starts Gazebo in headless mode and exposes ROS topics to Foxglove.

## What this runs

The launch file `src/swarm_exploration/launch/single_robot.launch.py` starts:
- Gazebo Harmonic in headless/server mode
- TurtleBot3 Waffle spawn
- ROS <-> Gazebo topic bridges (`ros_gz_bridge`)
- SLAM Toolbox
- Foxglove bridge on `ws://localhost:8765`

## Prerequisites

Assumes a Linux machine with:
- ROS 2 (same distro used by your workspace)
- Gazebo Harmonic + ROS Gazebo bridge packages
- Workspace dependencies installed for `swarm_exploration`

Helpful packages used by this launch stack include:
- `ros_gz_sim`
- `ros_gz_bridge`
- `slam_toolbox`
- `foxglove_bridge`
- `nav2_minimal_tb3_sim` (for TurtleBot3 simulation assets)

If dependencies are missing, run from workspace root:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Build

From workspace root (`swarm_ws`):

```bash
colcon build --packages-select swarm_exploration
source install/setup.bash
```

## Run headless simulator for Foxglove

In terminal 1:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch swarm_exploration single_robot.launch.py
```

Expected behavior:
- Gazebo runs headless (no GUI)
- Foxglove bridge starts on TCP port `8765`
- SLAM lifecycle is configured and activated automatically

## Connect Foxglove

Use Foxglove Desktop or web app:
1. Open a new connection.
2. Select **Foxglove WebSocket**.
3. Use URL: `ws://localhost:8765`.
4. Set fixed frame to `map`.

Suggested panels/topics:
- `LaserScan`: `/scan`
- `PointCloud`: `/scan/points`
- `PointCloud`: `/depth/points`
- `Map`: `/map`
- `Image`: `/depth/image`

## Optional teleop

In terminal 2:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Quick checks

List active topics:

```bash
ros2 topic list
```

Confirm Foxglove bridge node:

```bash
ros2 node list | grep foxglove_bridge
```

## Troubleshooting

- Port in use (`8765`): stop any process on that port, then relaunch.
- No map updates: wait a few seconds after launch; SLAM starts with timed delays.
- No robot movement in map: send velocity commands (teleop) so SLAM receives motion and scan updates.
- Missing package errors: re-run `rosdep install` and rebuild.
