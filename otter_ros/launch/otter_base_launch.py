'''launch_otter_base.py

This is a basic launch file for Otter ROS with the Otter USV

'''

import os
import datetime

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from ament_index_python import get_package_share_directory


def generate_launch_description():
    """Returns a launch description for Otter base functions

    Mandatory nodes:
    - otter_publisher
    - otter_subscriber

    ROS functions:
    - static transforms
    - bag recording in MCAP format
    """

    configdir = os.path.join(
        get_package_share_directory('otter_ros'), 'config')

    # Launch parameters
    bag_name = LaunchConfiguration('bag_name')

    # Bag name defaults to system time
    now = datetime.datetime.now()
    bag_name_str = now.strftime("%Y_%m_%dT%H_%M_%S")

    bag_name_launch_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='otter_bag_' + bag_name_str,
        description='Filename for the bag file. Default is otter_bag_<system time>.'
    )

    # Add nodes for Otter USV basic communications
    config = os.path.join(configdir, 'otterUSV.yaml')

    # Otter Publisher
    pub = Node(
        package='otter_ros',
        executable='otter_publisher',
        name='otter_publisher',
        parameters=[config],
    )

    # Otter Subscriber
    sub = Node(
        package='otter_ros',
        executable='otter_subscriber',
        name='otter_subscriber',
        parameters=[config],
    )

    # Transforms
    static_transforms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('otter_ros'),
                         'launch', 'otter_static_transforms_launch.py'))
    )

    otter_transform = Node(
        package='otter_ros',
        executable='otter_tf2_broadcaster',
        name='otter_tf2_broadcaster',
    )

    # Bag recording
    bag_all = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a',
             '-o', bag_name, '--storage', 'mcap'],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # launch arguments
    ld.add_action(bag_name_launch_arg)

    # required nodes
    ld.add_action(pub)
    ld.add_action(sub)

    # tranforms
    ld.add_action(otter_transform)
    ld.add_action(static_transforms)

    # data collection (bagging)
    ld.add_action(bag_all)

    return ld
