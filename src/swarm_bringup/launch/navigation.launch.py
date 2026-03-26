"""
navigation_launch.py (swarm_bringup override)

Nav2 bringup for swarm exploration.  Excludes route_server, docking_server,
smoother_server, and collision_monitor.  The cmd_vel chain is:
controller → cmd_vel_nav → velocity_smoother → cmd_vel → Gazebo bridge.

Accepts the same arguments as nav2_bringup/launch/navigation_launch.py so
robot_stack.launch.py doesn't need to change.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "bt_navigator",
    ]

    # No TF remapping needed — robot frames are namespaced (robot_0/*, robot_1/*)
    # so all robots can safely share the global /tf topic without frame-name conflicts.
    remappings = []

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={"autostart": autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("params_file"),
            DeclareLaunchArgument("autostart", default_value="true"),
            # Unused but declared so callers can pass them without error
            DeclareLaunchArgument("use_namespace", default_value="False"),
            DeclareLaunchArgument("slam", default_value="False"),
            DeclareLaunchArgument(
                "map_subscribe_transient_local", default_value="False"
            ),
            DeclareLaunchArgument("use_composition", default_value="False"),
            DeclareLaunchArgument("use_respawn", default_value="False"),
            DeclareLaunchArgument("log_level", default_value="info"),
            GroupAction(
                actions=[
                    SetParameter("use_sim_time", use_sim_time),
                    Node(
                        package="nav2_controller",
                        executable="controller_server",
                        output="screen",
                        parameters=[configured_params],
                        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                    ),
                    Node(
                        package="nav2_planner",
                        executable="planner_server",
                        name="planner_server",
                        output="screen",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    Node(
                        package="nav2_behaviors",
                        executable="behavior_server",
                        name="behavior_server",
                        output="screen",
                        parameters=[configured_params],
                        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                    ),
                    Node(
                        package="nav2_bt_navigator",
                        executable="bt_navigator",
                        name="bt_navigator",
                        output="screen",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    # velocity_smoother subscribes cmd_vel_nav (from controller)
                    # and publishes directly to cmd_vel (picked up by ros_gz_bridge).
                    Node(
                        package="nav2_velocity_smoother",
                        executable="velocity_smoother",
                        name="velocity_smoother",
                        output="screen",
                        parameters=[configured_params],
                        remappings=remappings
                        + [
                            ("cmd_vel", "cmd_vel_nav"),
                            ("cmd_vel_smoothed", "cmd_vel"),
                        ],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",
                        output="screen",
                        parameters=[
                            {"autostart": autostart},
                            {"node_names": lifecycle_nodes},
                        ],
                    ),
                ]
            ),
        ]
    )
