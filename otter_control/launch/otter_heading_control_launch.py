'''This is a launch file for the otter nodes'''

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    # Otter Base Launch
    otter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('otter_ros'),
                         'launch', 'otter_base_launch.py')),
    )

    controller = Node(
        package='otter_ros',
        executable='heading_controller',
        name='controller'
    )

    logger = Node(
        package='otter_ros',
        executable='logger',
        name='logger'
    )

    ld.add_action(otter)

    ld.add_action(controller)
    ld.add_action(logger)

    return ld
