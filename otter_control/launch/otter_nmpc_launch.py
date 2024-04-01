'''This is a launch file for the otter nodes'''

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
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

    wpt_file = LaunchConfiguration('wpt_file')

    DeclareLaunchArgument(
        'wpt_file',
        default_value=None,
        description='The waypoint file name. Matches the wpt_file name on the otter pi.'
    )

    wpt_file_path_obs = PathJoinSubstitution(
        ['otter@192.168.53.2:/var/lib/obs/files/', wpt_file])
    wpt_file_path_local = PathJoinSubstitution(
        ['/home/payload/otter_ws/wpt_paths/', wpt_file])

    controller = Node(
        package='otter_ros',
        executable='nmpc_controller',
        name='nmpc_controller',
        parameters=[{'wpt_file': wpt_file_path_local}]
    )

    sindy_logger = Node(
        package='otter_ros',
        executable='sindy_logger',
        name='sindy_logger',
        parameters=[{'wpt_file': wpt_file_path_local}]
    )
    log = Node(
        package='otter_ros',
        executable='otter_logger',
        name='otter_logger',
        parameters=[{'wpt_file': wpt_file_path_local}]
    )

    copy_wpt_file = ExecuteProcess(cmd=['sshpass', '-p', '"mrOtter69!"', 'scp', wpt_file_path_obs, '/home/payload/otter_ws/wpt_paths'],
                                   output='screen', shell=True,
                                   log_cmd=True)

    ld.add_action(otter)

    ld.add_action(copy_wpt_file)
    ld.add_action(controller)
    ld.add_action(log)
    ld.add_action(sindy_logger)
    return ld
