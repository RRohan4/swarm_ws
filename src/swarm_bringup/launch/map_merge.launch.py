"""
map_merge.launch.py

Launches the global map_merge_node singleton.
Accepts robot_ids and num_robots as arguments so compose.yaml can stay clean.

Args:
  robot_ids  : comma-separated robot IDs, e.g. "robot_0,robot_1" (default)
  rate       : merge publish rate Hz (default 2.0)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    ids_str = LaunchConfiguration("robot_ids").perform(context)
    rate = float(LaunchConfiguration("rate").perform(context))
    robot_ids = [r.strip() for r in ids_str.split(",")]

    return [
        Node(
            package="swarm_slam",
            executable="map_merge_node",
            name="map_merge_node",
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_ids": robot_ids,
                    "map_merge_rate": rate,
                }
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ids", default_value="robot_0,robot_1"),
            DeclareLaunchArgument("rate", default_value="2.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
