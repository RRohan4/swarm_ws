"""
map_merge.launch.py

Launches global singleton nodes: map_merge_node, frontier_detector_node,
coordinator_node.  Run as a single compose service after the robot stacks
are up.

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
        # ── Phase 3: map merge ─────────────────────────────────────────────────
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
        ),
        # ── Phase 4: frontier detector ─────────────────────────────────────────
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
        # ── Phase 6: coordinator ───────────────────────────────────────────────
        Node(
            package="swarm_exploration",
            executable="coordinator_node",
            name="coordinator_node",
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_ids": robot_ids,
                    "rate": 1.0,
                }
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ids", default_value="robot_0,robot_1"),
            DeclareLaunchArgument("rate", default_value="2.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
