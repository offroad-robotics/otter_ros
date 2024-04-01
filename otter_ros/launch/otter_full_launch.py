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
    """Returns a launch description for Otter with sensors

    Launch otter_base_launch.py in addition to sensor nodes:
    - velodyne
    - sbg_device

    """

    configdir = os.path.join(
        get_package_share_directory('otter_ros'), 'config')

    # Launch parameters
    bag_name = LaunchConfiguration('bag_name')
    use_velodyne = LaunchConfiguration('use_velodyne')
    use_sbg_serial = LaunchConfiguration('use_sbg_serial')

    # Disable optional nodes by default
    use_velodyne_launch_arg = DeclareLaunchArgument(
        'use_velodyne',
        default_value='False',
        description='Flag to activate Velodyne VLP-16 LiDAR sensor. Default is False.',
        choices=['True', 'False']
    )
    use_sbg_serial_launch_arg = DeclareLaunchArgument(
        'use_sbg_serial',
        default_value='False',
        description='Flag to activate SBG INS connection. Default is False.',
        choices=['True', 'False']
    )

    # Bag name defaults to system time
    now = datetime.datetime.now()
    bag_name_str = now.strftime("%Y_%m_%dT%H_%M_%S")

    bag_name_launch_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='otter_bag_' + bag_name_str,
        description='Filename for the bag file. Default is otter_bag_<system time>.'
    )

    # Otter Base Launch
    otter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('otter_ros'),
                         'launch', 'otter_base_launch.py')),
        launch_arguments={'bag_name': bag_name}.items()
    )

    # Add nodes for external sensors

    # Velodyne VLP-16 LiDAR
    velodyne = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('velodyne'),
                         'launch', 'velodyne-all-nodes-VLP16-launch.py')),
        condition=IfCondition(use_velodyne)
    )

    # SBG Ellipse INS
    config = os.path.join(configdir, 'sbg_ellipseD.yaml')
    sbg = Node(
        package='sbg_driver',
        executable='sbg_device',
        output='screen',
        parameters=[config],
        condition=IfCondition(use_sbg_serial),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # launch arguments
    ld.add_action(use_velodyne_launch_arg)
    ld.add_action(use_sbg_serial_launch_arg)
    ld.add_action(bag_name_launch_arg)

    # required nodes
    ld.add_action(otter)

    # optional nodes
    ld.add_action(velodyne)
    ld.add_action(sbg)

    return ld
