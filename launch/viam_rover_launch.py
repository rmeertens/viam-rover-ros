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
        # Node(
        #     package='usb_cam',
        #     namespace='usb_cam',
        #     executable='usb_cam_node_exe',
        #     arguments=['--params-file', '/home/roland/ros2_ws/src/viam_rover_package/config/usb_cam.yaml'],
        #     name='usb_cam'
        # ),
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

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_pub',
        #     arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        #     output='screen',
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='some_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='some_publisher_2',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen',
        ),
         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='some_publisher_3',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='some_publisher_2',
        #     arguments=['0', '0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        #     output='screen',
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='some_publisher_3',
        #     arguments=['0', '0', '0', '0', '0', '0', '0', 'map', 'odom'],
        #     output='screen',
        # ),
        
#ros2 run nav2_map_server map_server /home/roland/ros2_ws/src/viam-rover-ros/map/map.yaml
        # Node(
        #     package='nav2_map_server',
        #     executable='map_server',
        #     name='map_server',
        #     output='screen',
        #     parameters=[
        #         {'yaml_filename': '/home/roland/ros2_ws/src/viam-rover-ros/map/map.yaml'}
        #     ]
        #     # arguments=['/home/roland/ros2_ws/src/viam-rover-ros/map/map.yaml']
        # )
    ])