"""
global.launch.py

Launches global singleton nodes: global_node (map merge) and
frontier_detector_node.  Run as a single compose service after the robot
stacks are up.

Args:
  robot_ids  : comma-separated robot IDs, e.g. "robot_0,robot_1" (default)
  rate       : map merge publish rate Hz (default 2.0)
"""

import multiprocessing

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get_thread_count() -> int:
    """Get optimal thread count (1/2 available CPU cores, minimum 2)."""
    cpu_count = multiprocessing.cpu_count()
    return max(2, cpu_count // 2)


def launch_setup(context, *args, **kwargs):
    ids_str = LaunchConfiguration("robot_ids").perform(context)
    rate = float(LaunchConfiguration("rate").perform(context))
    robot_ids = [r.strip() for r in ids_str.split(",")]
    thread_count = _get_thread_count()

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
                    "robot_clear_radius": 0.55,
                    "_thread_count": thread_count,
                }
            ],
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="swarm_exploration",
            executable="frontier_detector_node",
            name="frontier_detector_node",
            parameters=[
                {
                    "use_sim_time": True,
                    "min_frontier_size": 3,
                    "min_unknown_backing": 4,
                    "detect_rate": 5.0,
                    "_thread_count": thread_count,
                }
            ],
            output="screen",
            emulate_tty=True,
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
