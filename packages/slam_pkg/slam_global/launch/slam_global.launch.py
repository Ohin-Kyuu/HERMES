from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    map_publisher = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='map_publisher',
        parameters=[{
            'file_name': '/home/slam/data/map/sector_map.pcd',
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

    tf_pub_3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_pub_3',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init']
    )

    ld = LaunchDescription()
    ld.add_action(map_publisher)
    ld.add_action(localizer)
    ld.add_action(transform)

    ld.add_action(tf_pub_3)

    return ld
