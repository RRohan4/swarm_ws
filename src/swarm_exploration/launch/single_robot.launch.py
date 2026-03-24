"""
single_robot.launch.py

TurtleBot3 Waffle in Gazebo Harmonic (headless) with:
  - Lidar (LaserScan /scan, PointCloud2 /scan/points)
  - Depth camera (/depth/image, /depth/points, /depth/camera_info)
  - SLAM Toolbox (/map + map frame)
  - Foxglove bridge at ws://localhost:8765

Foxglove: fixed frame = "map". Add /scan, /map, /depth/points.
Control the robot via Foxglove's Teleop panel publishing to /cmd_vel.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    tb3_dir = get_package_share_directory("nav2_minimal_tb3_sim")
    swarm_dir = get_package_share_directory("swarm_exploration")
    bringup_dir = get_package_share_directory("swarm_bringup")

    world = os.path.join(bringup_dir, "worlds", "maze_world.sdf")
    robot_xacro = os.path.join(swarm_dir, "urdf", "gz_waffle.sdf.xacro")
    robot_urdf = os.path.join(swarm_dir, "urdf", "turtlebot3_waffle.urdf")
    bridge_yaml = os.path.join(tb3_dir, "configs", "turtlebot3_waffle_bridge.yaml")

    with open(robot_urdf) as f:
        urdf_content = f.read()

    cam_link = "/world/swarm_world/model/turtlebot3_waffle/link/camera_link/sensor"
    rgb_prefix = cam_link + "/rgb_camera"
    depth_prefix = cam_link + "/intel_realsense_r200_depth"

    return LaunchDescription(
        [
            # ── Cleanup ────────────────────────────────────────────────────────────
            ExecuteProcess(
                cmd=["bash", "-c", "fuser -k 8765/tcp 2>/dev/null; sleep 1; true"],
                output="screen",
            ),
            # ── Gazebo resource paths ──────────────────────────────────────────────
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", os.path.join(tb3_dir, "models")
            ),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", str(Path(tb3_dir).parent.resolve())
            ),
            # ── Gazebo (headless) ──────────────────────────────────────────────────
            ExecuteProcess(
                cmd=["gz", "sim", "-r", "-s", world],
                output="screen",
            ),
            # ── Bridge: standard TB3 topics ────────────────────────────────────────
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[
                    {
                        "config_file": bridge_yaml,
                        "expand_gz_topic_names": False,
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
            # ── Bridge: lidar PointCloud2 ──────────────────────────────────────────
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                ],
                output="screen",
            ),
            # ── Bridge: depth camera ───────────────────────────────────────────────
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    rgb_prefix + "/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    rgb_prefix
                    + "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    depth_prefix + "/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                    depth_prefix + "/depth_image/points"
                    "@sensor_msgs/msg/PointCloud2"
                    "[gz.msgs.PointCloudPacked",
                    depth_prefix
                    + "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                ],
                remappings=[
                    (rgb_prefix + "/image", "/camera/image_raw"),
                    (rgb_prefix + "/camera_info", "/camera/camera_info"),
                    (depth_prefix + "/depth_image", "/depth/image"),
                    (depth_prefix + "/depth_image/points", "/depth/points"),
                    (depth_prefix + "/camera_info", "/depth/camera_info"),
                ],
                output="screen",
            ),
            # ── Robot spawn + state publisher (delayed for /clock) ─────────────────
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="robot_state_publisher",
                        executable="robot_state_publisher",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "robot_description": urdf_content,
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        arguments=[
                            "-name",
                            "turtlebot3_waffle",
                            "-string",
                            Command([FindExecutable(name="xacro"), " ", robot_xacro]),
                            "-x",
                            "0",
                            "-y",
                            "0",
                            "-z",
                            "0.01",
                        ],
                        output="screen",
                    ),
                ],
            ),
            # ── SLAM + Foxglove (delayed for TF + scan) ───────────────────────────
            TimerAction(
                period=10.0,
                actions=[
                    LifecycleNode(
                        package="slam_toolbox",
                        executable="async_slam_toolbox_node",
                        name="slam_toolbox",
                        namespace="",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "scan_topic": "/scan",
                                "base_frame": "base_footprint",
                                "odom_frame": "odom",
                                "map_frame": "map",
                                "max_laser_range": 9.9,
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="foxglove_bridge",
                        executable="foxglove_bridge",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "port": 8765,
                            }
                        ],
                        output="screen",
                    ),
                ],
            ),
            # ── Activate SLAM lifecycle ────────────────────────────────────────────
            TimerAction(
                period=15.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "bash",
                            "-c",
                            "until ros2 lifecycle set /slam_toolbox configure;"
                            " do sleep 2; done"
                            " && sleep 2"
                            " && until ros2 lifecycle set /slam_toolbox activate;"
                            " do sleep 2; done",
                        ],
                        output="screen",
                    ),
                ],
            ),
        ]
    )
