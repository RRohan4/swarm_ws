"""
gazebo.launch.py

Starts Gazebo Harmonic headless with the configured world.
Run as its own service; robot stacks connect to it via Gazebo transport.

Args:
  world : path to .sdf world file (default: robot_world.sdf from swarm_exploration)
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


def generate_launch_description():
    exploration_dir = get_package_share_directory("swarm_exploration")
    tb3_dir = get_package_share_directory("nav2_minimal_tb3_sim")
    default_world = os.path.join(exploration_dir, "worlds", "robot_world.sdf")

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
        ]
    )
