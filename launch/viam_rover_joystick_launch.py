from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            # namespace='joy',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='teleop_twist_joy',
            # namespace='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            remappings=[('/joy_config', '/xd3')]
        ),
    ])