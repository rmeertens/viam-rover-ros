from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='usb_cam',
        #     namespace='usb_cam',
        #     executable='usb_cam_node_exe',
        #     name='usb_cam'
        # ),
        Node(
            package='viam_rover_package',
            namespace='viam_rover',
            executable='viam_rover_node',
            name='viam_rover'
        ),
    ])