from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'pickn_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Register this package with ament.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Install the package manifest.
        ('share/' + package_name, ['package.xml']),
        # Include your share files (YAML configs, etc.)
        ('share/' + package_name, ['share/aruco_size_config.yaml']),
        ('share/' + package_name, ['share/arucoID_name_config.yaml']),
        ('share/' + package_name, ['share/axab_calibration.yaml']),
        ('share/' + package_name, ['share/pose_data_memory.yaml']),
        ('share/' + package_name, ['share/tool_offset_points.yaml']),
        ('share/' + package_name, ['share/machine_offset_points.yaml']),
        # Include launch files (if any).
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Remove custom service definitions from data_files
        # since they now live in pickn_place_interfaces.
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erds',
    maintainer_email='erdie@qltyss.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_perception = pickn_place.calibration_perception:main',
            'calibration_node = pickn_place.calibration_node:main',
            'aruco_perception = pickn_place.aruco_perception:main',
            'calibration_move = pickn_place.calibration_move:main',
            'pose_generator = pickn_place.pose_generator:main',
            'axxb_calibration = pickn_place.axxb_calibration:main',
            'manipulate_move = pickn_place.manipulate_move:main',
            'motion_command_service = pickn_place.motion_command_service:main',
            'machine_mount_teach = pickn_place.machine_mount_teach:main',
            'obstacle_generator = pickn_place.obstacle_generator:main',
            'tool_mount_teach = pickn_place.tool_mount_teach:main'
        ],
    },
)

