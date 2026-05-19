# Deprecated — use trajectory.launch.py instead.
from launch import LaunchDescription


def generate_launch_description():
    raise RuntimeError(
        'This file is deprecated. Use trajectory.launch.py instead.\n'
        '  ros2 launch monster_trayectoria trajectory.launch.py'
    )
