import os
from glob import glob
from setuptools import setup

package_name = 'otter_ros'
examples = 'examples'
pynmeagps = 'pynmeagps'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, pynmeagps],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Offroad Robotics @ Ingenuity Labs',
    maintainer_email='15mrc5@queensu.ca',
    description='This package is used to interface with the Maritime Robotics Otter USV using ROS2.',
    license='MIT license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'otter_subscriber = otter_ros.otter_subscriber:main',
            'otter_publisher = otter_ros.otter_publisher:main',
            'time_sync = examples.time_sync_sub:main',
            'otter_logger = otter_ros.otter_logger:main',
            'dynamic_transform = otter_ros.dynamic_transform:main',
            'otter_tf2_broadcaster = otter_ros.otter_tf2_broadcaster:main',
            'step_input = otter_ros.step_input:main',
            'sinusoid_input = otter_ros.sinusoid_input:main',

        ],
    },
)
