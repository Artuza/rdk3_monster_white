import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to package
    pkg_share = get_package_share_directory('s8_py_slam')

    # Path to the URDF file
    # amr_plugin or cylinder_plugin
    urdf_plugin_path = os.path.join(pkg_share, 'urdf', 'cylinder_plugin.urdf')

    # Launch file for Gazebo
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    
    # Path to your custom world file
    # robot_world or tec_warehouse
    world_path = os.path.join(pkg_share, 'worlds', 'tec_warehouse.world')

    return LaunchDescription([
        # 1) Start Gazebo with the specified world and verbosity
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': world_path,
                # 'verbose': 'true'
            }.items(),
        ),

        # 2) Spawn the robot model
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robot_lidar',
                '-file', urdf_plugin_path,
                '-x', '0.5',
                '-y', '0.5',
                '-z', '90.0'
            ],
            output='screen'
        ),
    ])
