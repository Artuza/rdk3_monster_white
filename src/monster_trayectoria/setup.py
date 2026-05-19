import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'monster_trayectoria'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='habikahdz',
    maintainer_email='gaco_04@hotmail.com',
    description='Trajectory follower for the RDK X3 Monster White AMR robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'trajectory_follower  = monster_trayectoria.trajectory_node:main',
            'send_goal            = monster_trayectoria.send_goal:main',
            'drift_square_robot   = monster_trayectoria.drift_square_robot:main',
            'omni_test            = monster_trayectoria.omni_test:main',
            'calibration_collector = monster_trayectoria.calibration_collector:main',
            'optitrack_receiver    = monster_trayectoria.optitrack_receiver:main',
            'optitrack_navigator   = monster_trayectoria.optitrack_navigator:main',
            'square_nav_optitrack  = monster_trayectoria.square_nav_optitrack:main',
            'square_nav_odom       = monster_trayectoria.square_nav_odom:main',
            'characterization_node = monster_trayectoria.characterization_node:main',
            'spin_characterization = monster_trayectoria.spin_characterization:main',
        ],
    },
)
