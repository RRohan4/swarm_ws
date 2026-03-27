"""
global.launch.py

Launches global singleton nodes: global_node (map merge) and
frontier_detector_node.  Run as a single compose service after the robot
stacks are up.

Args:
  robot_ids  : comma-separated robot IDs, e.g. "robot_0,robot_1" (default)
  rate       : map merge publish rate Hz (default 2.0)
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
            executable="global_node",
            name="global_node",
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_ids": robot_ids,
                    "rate": rate,
                }
            ],
            output="screen",
        ),
        Node(
            package="swarm_exploration",
            executable="frontier_detector_node",
            name="frontier_detector_node",
            parameters=[
                {
                    "use_sim_time": True,
                    "min_frontier_size": 5,
                    "detect_rate": 5.0,
                }
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ids", default_value="robot_0,robot_1,robot_2"),
            DeclareLaunchArgument("rate", default_value="2.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
