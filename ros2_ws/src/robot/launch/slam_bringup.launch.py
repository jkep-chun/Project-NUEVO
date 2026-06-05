from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot',
            executable='nav_bridge',
            name='navigation_bridge'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1878', '0', '0', '0', '0', '0', 'base_link', 'laser']
        )
    ])