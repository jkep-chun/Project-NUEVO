import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = get_package_share_directory('robot')
    params_file = os.path.join(config_dir, 'config', 'mapper_params_online_async.yaml')
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
        ),
        Node(
            parameters=[params_file],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        )
    ])