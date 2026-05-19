import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_tray = get_package_share_directory('monster_trayectoria')
    pkg_sim  = get_package_share_directory('mecanum_sim')
    config   = os.path.join(pkg_tray, 'config', 'trajectories.yaml')

    return LaunchDescription([
        # Suppress Gazebo multicast spam (harmless but floods the terminal)
        SetEnvironmentVariable('GAZEBO_IP', '127.0.0.1'),
        SetEnvironmentVariable('GAZEBO_HOSTNAME', 'localhost'),

        DeclareLaunchArgument(
            'preset',
            default_value='cuadrado',
            description='cuadrado | triangulo | circulo | ocho  (used only if no goal is sent)',
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='false',
            description='Loop the preset trajectory',
        ),

        # Start Gazebo + robot (without mecanum_controller — trajectory_follower drives instead)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_sim, 'launch', 'mecanum_gazebo.launch.py')
            ),
            launch_arguments={'with_controller': 'false'}.items(),
        ),

        # Trajectory follower — waits for /goal_pose or uses preset
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='monster_trayectoria',
                    executable='trajectory_follower',
                    name='trajectory_follower',
                    output='screen',
                    parameters=[
                        config,
                        {
                            'preset':        LaunchConfiguration('preset'),
                            'loop':          LaunchConfiguration('loop'),
                            'use_sim_time':  True,
                        },
                    ],
                )
            ],
        ),
    ])
