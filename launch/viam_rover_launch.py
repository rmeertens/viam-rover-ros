from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    bringup_dir = os.path.join(os.path.dirname(__file__), '..')    

    return LaunchDescription([
        Node(
            package='viam_rover_package',
            namespace='viam_rover',
            executable='viam_rover_control',
            name='viam_rover_control'
        ),
        Node(
            package='foxglove_bridge',
            namespace='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge'
        ),
        Node(
            package='usb_cam',
            namespace='usb_cam',
            executable='usb_cam_node_exe',
            arguments=['--params-file', os.path.join(bringup_dir, 'config', 'usb_cam.yaml')],
            name='usb_cam'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[os.path.join(bringup_dir, 'description', 'viam_rover_description.urdf')]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_footprint_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen',
        ),
         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
        ),
    ])