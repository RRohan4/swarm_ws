"""
swarm.launch.py

Top-level launch: Gazebo + N robot stacks + global nodes (map_merge, foxglove).
Global exploration nodes (frontier_detector, coordinator) are added in Phase 4–6.

Args:
  world      : path to .sdf world file (default: maze_world.sdf from swarm_bringup)
  num_robots : number of robots to spawn (default: 2)
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Spawn poses for up to 8 robots (extend as needed)
SPAWN_POSES = [
    {"x": "0.6", "y": "0.6", "z": "0.05", "yaw": "1.5708"},  # robot_0
    {"x": "1.8", "y": "0.6", "z": "0.05", "yaw": "1.5708"},  # robot_1
    {"x": "0.6", "y": "1.8", "z": "0.05", "yaw": "1.5708"},  # robot_2
    {"x": "1.8", "y": "1.8", "z": "0.05", "yaw": "1.5708"},  # robot_3
]


def launch_setup(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    world = LaunchConfiguration("world").perform(context)

    bringup_dir = get_package_share_directory("swarm_bringup")
    tb3_dir = get_package_share_directory("nav2_minimal_tb3_sim")
    robot_stack_launch = os.path.join(bringup_dir, "launch", "robot_stack.launch.py")

    robot_ids = [f"robot_{i}" for i in range(num_robots)]

    actions = [
        # ── Gazebo resource paths ──────────────────────────────────────────────
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", os.path.join(tb3_dir, "models")
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", str(Path(tb3_dir).parent.resolve())
        ),
        # ── Gazebo sim (headless) ──────────────────────────────────────────────
        ExecuteProcess(
            cmd=["gz", "sim", "-r", "-s", world],
            output="screen",
        ),
        # ── Clock bridge — must run exactly once; all sim-time nodes block
        # until /clock is published from Gazebo via this bridge.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_clock_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            parameters=[{"use_sim_time": False}],
            output="screen",
        ),
        # ── Foxglove bridge (singleton, global) ───────────────────────────────
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="foxglove_bridge",
                    executable="foxglove_bridge",
                    parameters=[{"use_sim_time": True, "port": 8765}],
                    output="screen",
                ),
            ],
        ),
    ]

    # ── Per-robot stacks ───────────────────────────────────────────────────────
    for i, robot_id in enumerate(robot_ids):
        pose = SPAWN_POSES[i] if i < len(SPAWN_POSES) else SPAWN_POSES[-1]
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_stack_launch),
                launch_arguments={
                    "robot_id": robot_id,
                    "robot_ids": ",".join(robot_ids),
                    "x": pose["x"],
                    "y": pose["y"],
                    "z": pose["z"],
                    "yaw": pose["yaw"],
                }.items(),
            )
        )

    # ── Global nodes (map merge + frontier detector) ───────────────────────────
    actions.append(
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package="swarm_slam",
                    executable="global_node",
                    name="global_node",
                    parameters=[
                        {
                            "use_sim_time": True,
                            "robot_ids": robot_ids,
                            "rate": 2.0,
                        }
                    ],
                    output="screen",
                ),
            ],
        )
    )

    actions.append(
        TimerAction(
            period=25.0,
            actions=[
                Node(
                    package="swarm_exploration",
                    executable="frontier_detector_node",
                    name="frontier_detector_node",
                    parameters=[
                        {
                            "use_sim_time": True,
                            "min_frontier_size": 5,
                            "detect_rate": 2.0,
                        }
                    ],
                    output="screen",
                ),
            ],
        )
    )

    return actions


def generate_launch_description():
    bringup_dir = get_package_share_directory("swarm_bringup")
    default_world = os.path.join(bringup_dir, "worlds", "maze_world.sdf")

    return LaunchDescription(
        [
            # Kill any stale Foxglove port
            ExecuteProcess(
                cmd=["bash", "-c", "fuser -k 8765/tcp 2>/dev/null; sleep 1; true"],
                output="screen",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Path to Gazebo world SDF",
            ),
            DeclareLaunchArgument(
                "num_robots",
                default_value="3",
                description="Number of robots to spawn",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
