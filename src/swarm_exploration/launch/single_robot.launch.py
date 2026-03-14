from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '/home/rraina/swarm_ws/robot_world.sdf'],
            output='screen'
        ),

        # Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            output='screen'
        ),

        # Static transform: base_link -> robot/lidar_link/lidar
        # Gazebo stamps LaserScan with the scoped sensor name: {model}/{link}/{sensor}
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'robot/lidar_link/lidar'],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # SLAM
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            parameters=[{
                'use_sim_time': True,
                'scan_topic': '/scan',
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'autostart': True,
            }],
            output='screen'
        ),
    ])
