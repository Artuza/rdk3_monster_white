import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to package
    pkg_share = get_package_share_directory('s8_py_slam')

    # Path to YAML file
    map_yaml_file = os.path.join(pkg_share, 'maps', 'map_server_params.yaml')

    return LaunchDescription([
        # Static transform from map to odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),

        # Launch the map_server node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[map_yaml_file]
        ),

        # Launch the lifecycle manager to manage the map_server node
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),
    ])
