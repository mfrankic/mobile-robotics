import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    go_to_goal = Node(package="mfrankic_6", executable="go_to_goal", name="go_to_goal")
    
    rviz_config_file = os.path.join(
        get_package_share_directory('mfrankic_6'),
        'config.rviz'
    )

    rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )

    ld = LaunchDescription()

    ld.add_action(go_to_goal)
    ld.add_action(rviz_cmd)

    return ld
