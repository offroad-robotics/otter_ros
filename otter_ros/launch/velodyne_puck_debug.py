"""
Custom launch file for the velodyne driver, pointcloud, and laserscan nodes.

Loads configuration from otter_ros.
"""

import os
import yaml

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():
    driver_params_file = os.path.join(ament_index_python.get_package_share_directory(
        'otter_ros'), 'config', 'vlp16_params.yaml')
    velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[driver_params_file])

    convert_share_dir = ament_index_python.packages.get_package_share_directory(
        'velodyne_pointcloud')
    convert_params_file = os.path.join(
        convert_share_dir, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(
            f)['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(
        convert_share_dir, 'params', 'VLP16db.yaml')
    velodyne_transform_node = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                      executable='velodyne_transform_node',
                                                      output='both',
                                                      parameters=[convert_params])

    laserscan_share_dir = ament_index_python.packages.get_package_share_directory(
        'velodyne_laserscan')
    laserscan_params_file = os.path.join(
        laserscan_share_dir, 'config', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node = launch_ros.actions.Node(package='velodyne_laserscan',
                                                      executable='velodyne_laserscan_node',
                                                      output='both',
                                                      parameters=[laserscan_params_file])

    return launch.LaunchDescription([velodyne_driver_node,
                                     velodyne_transform_node,
                                     velodyne_laserscan_node,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=velodyne_driver_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])


# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription

# from launch.launch_description_sources import PythonLaunchDescriptionSource

# from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
# from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition

# from ament_index_python import get_package_share_directory


# def generate_launch_description():
#     # Velodyne VLP-16 LiDAR
#     return LaunchDescription([
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(get_package_share_directory('velodyne'),
#                              'launch', 'velodyne-all-nodes-VLP16-launch.py')),
#         )
#     ])
