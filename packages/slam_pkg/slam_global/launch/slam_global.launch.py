import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    livox_ros2_pkg = get_package_share_directory('livox_ros_driver2')
    livox_launch = os.path.join(livox_ros2_pkg, 'launch_ROS2', 'msg_MID360_launch.py')
    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch),
    )

    fast_lio_pkg = get_package_share_directory('fast_lio')
    fast_lio_launch = os.path.join(fast_lio_pkg, 'launch', 'map_local.launch.py')
    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fast_lio_launch),
    )

    map_publisher = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='map_publisher',
        parameters=[{
            'file_name': '/home/slam/data/map/dit.pcd',
            'tf_frame': '/map',
            'publish_rate': 5.0,  # Hz
        }],
        remappings=[
            ('cloud_pcd', 'map'),
        ],
    )

    localizer = Node(
        package='slam_global',
        executable='localizer_node',
        name='localizer_node',
        output='screen'
    )

    transform = Node(
        package='slam_global',
        executable='transform_node',
        name='transform_node',
        output='screen'
    )

    odom2pose = Node(
        package='slam_global',
        executable='odom2pose',
        name='odom2pose_bridge',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(livox)

    ld.add_action(
        TimerAction(
            period=2.0,
            actions=[fast_lio]
        )
    )

    ld.add_action(
        TimerAction(
            period=4.0,
            actions=[map_publisher]
        )
    )

    ld.add_action(
        TimerAction(
            period=5.0,
            actions=[localizer]
        )
    )

    ld.add_action(
        TimerAction(
            period=6.0,
            actions=[transform]
        )
    )

    ld.add_action(
        TimerAction(
            period=1.0,
            actions=[odom2pose]
        )
    )

    return ld
