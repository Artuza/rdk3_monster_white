from setuptools import find_packages, setup
import os
from glob import glob

package_name = 's8_py_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduardo',
    maintainer_email='eduardodavila94@hotmail.com',
    description='-SLAM simulation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pose_tf_sub_pub_exe = s8_py_slam.pose_tf_sub_pub:main",
            "map_terminal_printer_exe = s8_py_slam.map_terminal_printer:main",
            "relative_map_terminal_printer_exe = s8_py_slam.relative_map_terminal_printer:main"
        ],
    },
)
