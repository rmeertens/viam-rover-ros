from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='viam_rover_package',
            namespace='viam_rover',
            executable='viam_rover_encoder',
            name='viam_rover_enc'
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
            arguments=['--params-file', '/home/roland/ros2_ws/src/viam_rover_package/config/usb_cam.yaml'],
            name='usb_cam'
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0.2', '0.0', '0.3', '1.57', '3.14', '1.57', 'base_link', 'default_cam'],
        #     name='static_transform_publisher'
        # ),
        # Urdf 
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=['/home/roland/ros2_ws/src/viam-rover-ros/description/viam_rover_description.urdf']
        ),
    ])