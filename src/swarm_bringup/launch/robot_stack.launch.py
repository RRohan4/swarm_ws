"""
robot_stack.launch.py

Per-robot launch: RSP + Gazebo bridge + SLAM Toolbox + Nav2 + static world→map TF.

Args:
  robot_id  : e.g. robot_0
  x, y, z   : spawn position (default 0 0 0.05)
  yaw       : spawn yaw in radians (default 0)
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _write_bridge_config(robot_id: str) -> str:
    """Write a per-robot ros_gz_bridge YAML config to a tempfile and return its path."""
    cfg = [
        # Clock is bridged once in gazebo.launch.py — NOT per robot.
        # With shared /dev/shm, multiple clock bridges produce interleaved
        # timestamps that cause tf2 "jump back in time" crashes in SLAM.
        # Drive
        {
            "ros_topic_name": f"/{robot_id}/cmd_vel",
            "gz_topic_name": f"/{robot_id}/cmd_vel",
            "ros_type_name": "geometry_msgs/msg/Twist",
            "gz_type_name": "gz.msgs.Twist",
            "direction": "ROS_TO_GZ",
        },
        # Odometry
        {
            "ros_topic_name": f"/{robot_id}/odom",
            "gz_topic_name": f"/{robot_id}/odom",
            "ros_type_name": "nav_msgs/msg/Odometry",
            "gz_type_name": "gz.msgs.Odometry",
            "direction": "GZ_TO_ROS",
        },
        # LIDAR
        {
            "ros_topic_name": f"/{robot_id}/scan",
            "gz_topic_name": f"/{robot_id}/scan",
            "ros_type_name": "sensor_msgs/msg/LaserScan",
            "gz_type_name": "gz.msgs.LaserScan",
            "direction": "GZ_TO_ROS",
        },
        # TF — publish to global /tf so Foxglove and any other tool can read it.
        # Frame IDs are already namespaced (robot_N/odom, robot_N/base_footprint)
        # so multiple robots writing to /tf causes no conflicts.
        {
            "ros_topic_name": "/tf",
            "gz_topic_name": f"/{robot_id}/tf",
            "ros_type_name": "tf2_msgs/msg/TFMessage",
            "gz_type_name": "gz.msgs.Pose_V",
            "direction": "GZ_TO_ROS",
        },
        # Joint states
        {
            "ros_topic_name": f"/{robot_id}/joint_states",
            "gz_topic_name": f"/{robot_id}/joint_states",
            "ros_type_name": "sensor_msgs/msg/JointState",
            "gz_type_name": "gz.msgs.Model",
            "direction": "GZ_TO_ROS",
        },
    ]
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=f"_{robot_id}_bridge.yaml", delete=False
    )
    yaml.dump(cfg, tmp)
    tmp.flush()
    return tmp.name


def _make_slam_params(robot_id: str, slam_params_path: str) -> str:
    """Load slam_params.yaml, substitute robot_id_placeholder, write to tempfile."""
    with open(slam_params_path) as f:
        content = f.read()
    content = content.replace("robot_id_placeholder", robot_id)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=f"_{robot_id}_slam.yaml", delete=False
    )
    tmp.write(content)
    tmp.flush()
    return tmp.name


def _make_nav2_params(robot_id: str, nav2_params_path: str) -> str:
    """Load nav2_params.yaml, substitute robot_id_placeholder, write to tempfile."""
    with open(nav2_params_path) as f:
        content = f.read()
    content = content.replace("robot_id_placeholder", robot_id)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=f"_{robot_id}_nav2.yaml", delete=False
    )
    tmp.write(content)
    tmp.flush()
    return tmp.name


def launch_setup(context, *args, **kwargs):
    # SWARM_ROBOT_ID env var is the authoritative source for robot identity.
    # With network_mode: host, all containers share the same ROS2 DDS domain and
    # ros2 launch argument resolution gets cross-contaminated across containers.
    # Environment variables are set per-service in compose.yaml and are immune to this.
    import os

    robot_id = os.environ.get(
        "SWARM_ROBOT_ID", LaunchConfiguration("robot_id").perform(context)
    )
    x = os.environ.get("SWARM_X", LaunchConfiguration("x").perform(context))
    y = os.environ.get("SWARM_Y", LaunchConfiguration("y").perform(context))
    z = LaunchConfiguration("z").perform(context)
    yaw = os.environ.get("SWARM_YAW", LaunchConfiguration("yaw").perform(context))

    bringup_dir = get_package_share_directory("swarm_bringup")
    exploration_dir = get_package_share_directory("swarm_exploration")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    robot_xacro = os.path.join(exploration_dir, "urdf", "gz_waffle.sdf.xacro")
    robot_urdf = os.path.join(exploration_dir, "urdf", "turtlebot3_waffle.urdf")
    slam_params = _make_slam_params(
        robot_id, os.path.join(bringup_dir, "config", "slam_params.yaml")
    )
    nav2_params = _make_nav2_params(
        robot_id, os.path.join(bringup_dir, "config", "nav2_params.yaml")
    )
    bridge_cfg = _write_bridge_config(robot_id)

    with open(robot_urdf) as f:
        urdf_content = f.read()
    # frame_prefix on RSP handles the robot_N/ namespace — do NOT rename URDF links,
    # as "/" is not a valid character in URDF link names and crashes RSP.

    nodes = [
        # ── Gazebo bridge (per robot) ──────────────────────────────────────────
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"gz_bridge_{robot_id}",
            parameters=[
                {
                    "config_file": bridge_cfg,
                    "expand_gz_topic_names": False,
                    "use_sim_time": True,
                }
            ],
            output="screen",
        ),
        # ── Robot state publisher ──────────────────────────────────────────────
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=robot_id,
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_description": urdf_content,
                    "frame_prefix": f"{robot_id}/",
                }
            ],
            output="screen",
        ),
        # ── Static TF: world → robot_N/map (identity; co-located spawns) ──────
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"world_to_{robot_id}_map",
            arguments=[x, y, "0", yaw, "0", "0", "world", f"{robot_id}/map"],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        # ── Spawn robot in Gazebo ──────────────────────────────────────────────
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    name=f"spawn_{robot_id}",
                    arguments=[
                        "-name",
                        robot_id,
                        "-string",
                        Command(
                            [
                                FindExecutable(name="xacro"),
                                f" {robot_xacro} namespace:={robot_id}",
                            ]
                        ),
                        "-x",
                        x,
                        "-y",
                        y,
                        "-z",
                        z,
                        "-Y",
                        yaw,
                    ],
                    output="screen",
                ),
            ],
        ),
        # ── SLAM Toolbox ───────────────────────────────────────────────────────
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package="slam_toolbox",
                    executable="async_slam_toolbox_node",
                    name="slam_toolbox",
                    namespace=robot_id,
                    parameters=[slam_params, {"use_sim_time": True}],
                    # slam_toolbox hardcodes /map as an absolute topic;
                    # remap to /{robot_id}/map so Nav2 global_costmap finds it.
                    remappings=[
                        ("/map", f"/{robot_id}/map"),
                        ("/map_metadata", f"/{robot_id}/map_metadata"),
                    ],
                    output="screen",
                ),
            ],
        ),
        # ── SLAM lifecycle: configure then activate ────────────────────────────
        # async_slam_toolbox_node is a lifecycle node; must be explicitly activated.
        # Use ros2 service call instead of ros2 lifecycle set — the latter relies
        # on ros2 node list discovery which can fail with network_mode: host DDS
        # sharing, while direct service calls always work once the node is up.
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "bash",
                        "-c",
                        f"until ros2 service call /{robot_id}/slam_toolbox/change_state"
                        " lifecycle_msgs/srv/ChangeState '{transition: {id: 1}}'"
                        " 2>&1 | grep -q 'success=True';"
                        " do sleep 1; done"
                        " && sleep 2"
                        f" && until ros2 service call /{robot_id}/slam_toolbox/change_state"
                        " lifecycle_msgs/srv/ChangeState '{transition: {id: 3}}'"
                        " 2>&1 | grep -q 'success=True';"
                        " do sleep 1; done",
                    ],
                    output="screen",
                ),
            ],
        ),
        # ── Nav2 ───────────────────────────────────────────────────────────────
        # Nav2 Jazzy's navigation_launch.py removed PushRosNamespace internally,
        # so we must push the namespace here so nodes land at /robot_N/controller_server
        # and RewrittenYaml(root_key=namespace) can resolve their parameters.
        # Timer is 20s so SLAM has time to configure, activate, and publish the
        # initial map→odom TF before global_costmap tries to look it up.
        TimerAction(
            period=20.0,
            actions=[
                GroupAction(
                    actions=[
                        PushRosNamespace(robot_id),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(
                                    bringup_dir, "launch", "navigation.launch.py"
                                )
                            ),
                            launch_arguments={
                                "use_sim_time": "True",
                                "namespace": robot_id,
                                "use_namespace": "True",
                                "params_file": nav2_params,
                                "slam": "False",
                                "map_subscribe_transient_local": "True",
                                "autostart": "True",
                            }.items(),
                        ),
                    ]
                ),
            ],
        ),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="robot_0"),
            DeclareLaunchArgument("x", default_value="0.6"),
            DeclareLaunchArgument("y", default_value="0.6"),
            DeclareLaunchArgument("z", default_value="0.05"),
            DeclareLaunchArgument("yaw", default_value="1.5708"),
            OpaqueFunction(function=launch_setup),
        ]
    )
