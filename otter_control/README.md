# Otter Control

## Overview

Otter Control uses OtterROS for autonomous control research.


## Launch files

This folder contains launch files for the **otter_control** package.

- **otter_launch_nmpc:** launches publisher and subscriber for the Otter and an NMPC controller node developed by Riley Cooper.

  - **`wpt_file`**: path to waypoint file. Defaults to None.

- **otter_heading_control_launch:** launches publisher and subscriber for the Otter and the controller node.

## Nodes

### nmpc_controller

This is a node that implements a nonlinear model predictive controller (NMPC) for the Otter USV. The nonlinear model of the Otter is based on _Handbook of Marine Craft Hydrodynamics and Motion Control_ (Thor I. Fossen)and the accompanying MATLAB/Python code.

#### Subscribed Topics

- **`/otter_gps`** ([sensor_msgs/NavSatFix](https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatFix.html)): GPS position of the Otter USV. Includes latitude, longitude, and altitude.

- **`/otter_imu`** ([sensor_msgs/Imu](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html)): orientation information of the Otter USV including *roll*, *pitch*, *yaw*, *p*, *q*, and *r*. The rates of change of *roll*, *pitch*, and *yaw* are *p*, *q*, and *r* respectively.

- **`/otter_cogsog`** ([otter_msgs/COGSOG](https://code.engineering.queensu.ca/offroad/otter-msgs/-/blob/master/msg/COGSOG.msg)): Course over ground, speed over ground, and resultant twist x, y vector

#### Published Topics

- **`/control_commands`** ([geometry_msgs/WrenchStamped](https://docs.ros.org/en/api/geometry_msgs/html/msg/WrenchStamped.html)): force and torque of Otter motors. Only populated parameters should be "force.x" and "torque.z".

- **`/controller_errors`** ([otter_msgs/ControllerErrors](https://code.engineering.queensu.ca/offroad/otter-msgs/-/blob/master/msg/ControllerErrors.msg)): This message contains lateral, heading and speed error values collected from the controllers and the closest waypoint to the USV.


[ROS2]: http://www.ros.org
