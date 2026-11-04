from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lms_starter',
            executable='lms_starter',
            output='log',
            name='lms_starter',
            parameters=[
                {
                    'use_binary_protocol': False,
                }
            ]
        )
    ])  