'''

Launch static transforms for Otter USV.

Modify this for your Otter configuration!

'''

from launch import LaunchDescription
from launch_ros.actions import Node

import numpy as np


def generate_launch_description():
    return LaunchDescription([
        # World NED to World (ENU)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0',
                       '%.9f' % (np.pi/2.0), '0', '%.9f' % np.pi,
                       'world', 'world_ned']
        ),

        # Otter to World NED is dynamic, see otter_tf2_broadcaster.py

        # INS to Otter
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0',
                       '0', '0', '0',
                       'otter', 'ins']
        ),

        # Velodyne to Otter
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.40', '0', '-0.50',
                       '0', '0', '%.9f' % np.pi,
                       'otter', 'velodyne']
        ),

    ])
