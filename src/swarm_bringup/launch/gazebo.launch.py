"""
gazebo.launch.py

Starts Gazebo Harmonic headless with the configured world.
Run as its own service; robot stacks connect to it via Gazebo transport.

Args:
  world : path to .sdf world file (default: maze_world.sdf from swarm_bringup)
"""

import os
import re
import tempfile
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


def _patched_world(source_sdf: str, rtf: float) -> str:
    """Return path to a temp SDF with real_time_factor replaced by *rtf*.

    The temp file persists for the lifetime of the process (delete=False) so
    gz sim can open it after this function returns.
    """
    text = Path(source_sdf).read_text()
    text = re.sub(
        r"<real_time_factor>[^<]*</real_time_factor>",
        f"<real_time_factor>{rtf}</real_time_factor>",
        text,
    )
    tmp = tempfile.NamedTemporaryFile(
        suffix=".sdf", prefix="maze_world_rtf_", delete=False
    )
    tmp.write(text.encode())
    tmp.flush()
    return tmp.name


def generate_launch_description():
    bringup_dir = get_package_share_directory("swarm_bringup")
    tb3_dir = get_package_share_directory("nav2_minimal_tb3_sim")
    source_world = os.path.join(bringup_dir, "worlds", "maze_world.sdf")

    rtf = float(os.environ.get("GZ_REAL_TIME_FACTOR", "1.0"))
    if rtf != 1.0:
        world_path = _patched_world(source_world, rtf)
    else:
        world_path = source_world

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=world_path,
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
