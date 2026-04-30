import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Path to package
    pkg_share = get_package_share_directory('s8_py_slam')

    # Path to the URDF file
    # amr or cylinder_plugin
    urdf_path = os.path.join(pkg_share, 'urdf', 'cylinder_plugin.urdf')

    # Path to RViz configuration file
    rviz_config_path = os.path.join('src/s8_py_slam/', 'rviz', 'slam.rviz')
    
    # Read URDF files
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 1) Publish the URDF to /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            # name='robot_state_publisher',
            namespace='rdk_x3_amr',
            output='screen',
            parameters=[{'robot_description': robot_desc,
                        'use_sim_time': True
                        }],
        ),

        # 2) RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
        ),

        # 3) Slam Toolbox
        # Node(
        #     package='slam_toolbox',
        #     executable='async_slam_toolbox_node',
        #     name='async_slam_toolbox',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': True}]
        # ),
    ])
