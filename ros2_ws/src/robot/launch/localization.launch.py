import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robot')
    
    # Paths
    map_file = os.path.join(pkg_share, 'maps', 'arena_map.yaml')
    amcl_config = os.path.join(pkg_share, 'config', 'amcl_params.yaml')
    
    return LaunchDescription([
        # 1. Navigation Bridge (Odom + TF)
        Node(
            package='robot',
            executable='nav_bridge',
            name='navigation_bridge'
        ),

        # 2. Static TF for Lidar
        # Adjust these values if your lidar mount offset changes
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1878', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),

        # 3. Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}]
        ),

        # 4. Laser Filter (Remove scans outside FOV)
        Node(
            package='sensors',
            executable='laser_filter',
            name='laser_filter',
            output='screen',
            parameters=[{
                'fov_min_deg': -70.0,
                'fov_max_deg': 70.0,
                'range_min_m': 0.07,
                'range_max_m': 3.5
            }]
        ),

        # 5. AMCL (Localization)
        # Remap /amcl_pose to /pose so slam_bridge can pick it up
        # Remap scan to /scan_filtered to use the filtered lidar data
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config],
            remappings=[
                ('/amcl_pose', '/pose'),
                ('initialpose', '/initialpose'),
                ('scan', '/scan_filtered')
            ]
        ),

        # 6. SLAM Bridge (Converter: m -> mm)
        # Listens to /pose and publishes to /slam_pose_update
        Node(
            package='robot',
            executable='slam_bridge',
            name='slam_bridge',
            output='screen'
        ),

        # 7. Lifecycle Manager (to activate map_server and amcl)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        ),

        # 8. Foxglove Bridge for visualization
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge'
        )
    ])
