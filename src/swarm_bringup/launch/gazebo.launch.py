"""
gazebo.launch.py

Starts Gazebo Harmonic headless with the configured world.
Run as its own service; robot stacks connect to it via Gazebo transport.

Args:
  world : path to .sdf world file (default: maze_world.sdf from swarm_bringup)
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("swarm_bringup")
    tb3_dir = get_package_share_directory("nav2_minimal_tb3_sim")
    default_world = os.path.join(bringup_dir, "worlds", "maze_world.sdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Path to Gazebo world SDF",
            ),
            # Gazebo needs these to find TB3 mesh assets when loading spawned models
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", os.path.join(tb3_dir, "models")
            ),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", str(Path(tb3_dir).parent.resolve())
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-r", "-s", LaunchConfiguration("world")],
                output="screen",
            ),
            # Single clock bridge — must only run once to avoid interleaved
            # timestamps that crash SLAM's tf2 buffer when /dev/shm is shared.
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_clock_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                parameters=[{"use_sim_time": False}],
                output="screen",
            ),
        ]
    )
