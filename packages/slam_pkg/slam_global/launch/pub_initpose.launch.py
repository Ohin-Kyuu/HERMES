from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pub_initialpose = Node(
        package='slam_global',
        executable='pub_initpose.py',
        name='publish_initial_pose',
        output='screen',
        parameters=[{
            'x': -0.55,
            'y':  1.72,
            'z':  0.0,
            'roll':  0.0,
            'pitch': 0.0,
            'yaw':  1.5
        }]   
    )

    ld = LaunchDescription()
    ld.add_action(pub_initialpose)
    return ld