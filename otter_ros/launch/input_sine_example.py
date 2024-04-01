'''This is a basic launch file for Otter ROS on the Otter USV'''

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    input_frequency = LaunchConfiguration('input_frequency')
    delay = LaunchConfiguration('delay')
    input_amplitude = LaunchConfiguration('input_amplitude')

    # Define any parameters that need to be passed to nodes
    input_frequency_launch_arg = DeclareLaunchArgument(
        'input_frequency',
        default_value='1.0',
        description='Frequency of input sinusoid [0, inf]'
    )

    delay_launch_arg = DeclareLaunchArgument(
        'delay',
        default_value='5.0',
        description='Time to wait before and after sinusoidal input [0, inf]'
    )

    amplitude_launch_arg = DeclareLaunchArgument(
        'input_amplitude',
        default_value='0.5',
        description='Amplitude of sinusoid [0, inf]'
    )

    # Add nodes for Otter USV basic communications
    # Otter Publisher
    pub = Node(
        package='otter_ros',
        executable='otter_publisher',
        name='otter_publisher'
    )
    # Otter Subscriber
    sub = Node(
        package='otter_ros',
        executable='otter_subscriber',
        name='otter_subscriber'
    )

    # Sine Input
    sine_input = Node(
        package='otter_ros',
        executable='sinusoid_input',
        name='sinusoid_input',
        parameters=[
            {'input_frequency': input_frequency},
            {'delay': delay},
            {'input_amplitude': input_amplitude}
        ]
    )

    # Define static transforms for the Otter USV and sensors
    # INS transform (frame defined by Otter OBS)
    static_transform_ins = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'otter', 'ins']
    )
    # Velodyne transform
    static_transform_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.40', '0', '0.50', '0', '0', '0', 'otter', 'velodyne']
    )

    # Bag recording
    bag_all = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '--storage', 'mcap'], output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(input_frequency_launch_arg)
    ld.add_action(delay_launch_arg)
    ld.add_action(amplitude_launch_arg)

    ld.add_action(pub)
    ld.add_action(sub)
    ld.add_action(sine_input)

    ld.add_action(static_transform_ins)
    ld.add_action(static_transform_velodyne)

    ld.add_action(bag_all)

    return ld
