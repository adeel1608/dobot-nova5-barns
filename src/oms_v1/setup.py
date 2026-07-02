#!/usr/bin/env python3
from setuptools import setup, find_packages

package_name = 'oms_v1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'pyorbbecsdk==1.3.2'],
    zip_safe=True,
    maintainer='Adeel Mohammed Khan',
    maintainer_email='adeel@qltyss.com',
    description='Basic ROS2 Humble Python package for oms_v1',
    license='Apache License 2.0',
    tests_require=['pytest'],
    extras_require={
        # Streamlit GUI for production-point and marker teach updates.
        # Kept optional so the production runtime does not pull in streamlit
        # unless the operator workstation explicitly installs it:
        #   pip install -e .[gui]
        'gui': ['streamlit>=1.30'],
    },
    entry_points={
        'console_scripts': [
            'cli = oms_v1.cli:main',
            'oms_v1_gui = oms_v1.gui.streamlit_app:cli_main',
        ],
    },
)
