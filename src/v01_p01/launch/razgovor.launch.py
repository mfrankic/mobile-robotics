from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace='v01_p01',
            package='v01_p01',
            executable='talker',
            name='talker'
        ),
        Node(
            namespace='v01_p01',
            package='v01_p01',
            executable='listener',
            name='listener'
        )
    ])
