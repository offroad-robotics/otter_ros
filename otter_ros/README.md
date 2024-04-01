# Otter ROS

## Overview

<img src="https://i.imgur.com/4PBpnlb.png" alt="Otter ROS Logo" width="200"/>

Otter ROS is a ROS2 driver for the Maritime Robotics Otter USV, intended to be run on a secondary computer system to communicate with the proprietary hardware on the vessel. It is intended for research purposes and in development by the Offroad Robotics research group at Ingenuity Labs at Queen's University.

Please see accompanying message package [Otter Msgs](https://code.engineering.queensu.ca/offroad/otter-msgs) for more information on custom messages used in Otter ROS.

A few controllers have also been included that are based on the work of Thor I. Fossen in the book _Handbook of Marine Craft Hydrodynamics and Motion Control_ and his accompanying [MatLab simulator](https://github.com/cybergalactic/MSS) _Marine Systems Simulator_ (**MSS**) and [Python simulator](https://github.com/cybergalactic/PythonVehicleSimulator) _PythonVehicleSimulator_.

**Ensure communications with the Otter USV before launching Otter ROS.** Edit config/otter_params.yaml to match networking setup.

**Keywords:** otter usv, usv driver, maritime robotics, ros2, python, nmea, navigation, control

### License

The source code is presently unlicensed.

**Authors: Riley Cooper, Sabrina Button, Thomas Sears, Joshua Marshall<br />
Acknowledgements: Thor I. Fossen<br />
Affiliation: [Offroad Robotics, Queen's University](https://offroad.engineering.queensu.ca/)<br />
Maintainer: Sabrina Button, sabrina.button@queensu.ca**

The Otter ROS package has been tested under [ROS2] Foxy on Ubuntu 20.04, interfaced with a Maritime Robotics Otter USV equipped with a VLP-16 LiDAR. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System 2 (ROS2)](http://wiki.ros.org) (middleware for robotics),
- [Velodyne](https://github.com/ros-drivers/velodyne) (LiDAR driver) - _OPTIONAL_, remove from launch files if not using

        sudo rosdep install --from-paths src

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

    cd ros2_ws/src
    git clone https://code.engineering.queensu.ca/offroad/otter-ros.git
    cd ../
    rosdep install --from-paths . --ignore-src
    colcon build

## Usage

Run the main nodes (+ velodyne driver) with

    ros2 launch otter_ros otter_launch_base.py

## NMEA Input

- **`MARGPS`**: otter lat, lon, alt, sog, cog, hog
- **`MARIMU`**: otter roll, pitch, yaw, p,q,r
- **`MAROTR`**: otter port and starboard motor rpm, temp and power, as well as total power and number of batteries

## Config files

### otter_params.yaml

This file contains the parameters for the Otter ROS driver. The default values are set for the Otter USV. The parameters are:

- **`socket`**: entry socket for otter, defaults to 2009
- **`ip_address`**: ip for otter, defaults to
- **`frame_id`**: frame id for the odometry messages

### ekf_with_gps.yaml

This file contains the parameters for the robot_localization navsat transform. The default values are set for the Otter USV. The most important parameter is:

- **`magnetic_declination`**: magnetic declination of the USV's operating environment, defaults to 0.141953445247 (Kingston, Ontario, July 2023)

## Launch files

This folder contains launch files for the **otter_ros** package.

- **otter_launch_nmpc:** launches pub,sub for otter and an NMPC controller node developed by Riley Cooper.

  - **`autobag`**: records all topics to a bag file by default. Set to false to disable.
  - **`bag_name`**: name of bag file to record to. Defaults to "/mnt/data/otter_ros_default_bag".
  - **`wpt_file`**: path to waypoint file. Defaults to None.

- **otter_sys_id_test_launch:** a launch file created for testing in dec 2021 and launches the test plan file created for this testing on top of pub, sub for the otter.
- **otter_heading_control_launch:** launches pub,sub for otter and a controller node.
- **otter_navsat_transform_launch:** launches a navsat transform for the otter which converts otter_imu and otter_gps to an otter_odom Odometry message. Should be launched concurrently with otter_launch.

## Nodes

### otter_publisher

This node handles all the data being transmitted from the Otter OBS system. The data comes over TCP port 2009 (by default).

#### Published Topics

- **`/otter_status`** ([otter_msgs/OtterStatus](https://code.engineering.queensu.ca/offroad/otter-msgs/-/blob/master/msg/OtterStatus.msg)): MAROTR data

- **`/otter_cogsog`** ([otter_msgs/COGSOG](https://code.engineering.queensu.ca/offroad/otter-msgs/-/blob/master/msg/COGSOG.msg)): Course over ground, speed over ground, and resultant twist x, y vector

- **`/otter_gps`** ([sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)): GPS lat, long, alt

- **`/otter_imu`** ([sensor_msgs/Imu](https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)): orientation and angular velocities

### otter_subscriber

This is a file that handles sending data to the otter. As of now, it will only send force in x and torque in z commands to the otter via the manual command. Additional commands may be added later. It also publishes time stamped versions of the data for logging so that the time stamp is as accurate as possible to when the command is being sent out for logging purposes.

#### Subscribed Topics

- **`/control_commands`** ([geometry_msgs/WrenchStamped](https://docs.ros.org/en/api/geometry_msgs/html/msg/WrenchStamped.html)): force and torque of Otter motors. Only populated parameters should be "force.x" and "torque.z".

### nmpc_controller

This is a node that implements a nonlinear model predictive controller (NMPC) for the Otter USV. It is based on the work of Thor I. Fossen in the book _Handbook of Marine Craft Hydrodynamics and Motion Control_ and his accompanying MatLab simulator.

#### Subscribed Topics

- **`/otter_odom`** ([nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)): Odometry message containing the Otter's COG (float64) and SOG (float64) for convenience, though this information is also available in /otter_odom using trigonometry on twist x and y.

#### Published Topics

- **`/control_commands`** ([geometry_msgs/WrenchStamped](https://docs.ros.org/en/api/geometry_msgs/html/msg/WrenchStamped.html)): force and torque of Otter motors. Only populated parameters should be "force.x" and "torque.z".

### otter_logger

This file contains a node which subscribes to all the data being recorded by the otter and all data being sent to the otter. It then formats it nicely into a csv file for offline analysis. By default, it is set up to save the csv files in a folder called "csv_logs" in the workspace folder. This can be changed and modified by editing the \_path variable in the main function of this file.

#### Subscribed Topics

- **`/otter_odom`** ([nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)): Odometry message containing the Otter's COG (float64) and SOG (float64) for convenience, though this information is also available in /otter_odom using trigonometry on twist x and y.

- **`/otter_status`** ([otter_msgs/OtterStatus](https://code.engineering.queensu.ca/offroad/otter-msgs/-/blob/master/msg/OtterStatus.msg)): MAROTR data

- **`/otter_cogsog`** ([otter_msgs/COGSOG](https://code.engineering.queensu.ca/offroad/otter-msgs/-/blob/master/msg/COGSOG.msg)): Course over ground, speed over ground, and resultant twist x, y vector

- **`/otter_gps`** ([sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)): GPS lat, long, alt

- **`/otter_imu`** ([sensor_msgs/Imu](https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)): orientation and angular velocities

- **`/control_commands`** ([geometry_msgs/WrenchStamped](https://docs.ros.org/en/api/geometry_msgs/html/msg/WrenchStamped.html)): force and torque of Otter motors. Only populated parameters should be "force.x" and "torque.z".

## Embedded Package: pynmeagps

This is a python package that interprets NMEA messages. NMEA is a commonly used standardized message format that the Otter uses for its message structure over the system's local network.
The package has been modified to include the proprietary messages from Maritime Robotics. The
messages included are outlined in the **Otter External control interface** document provided by Maritime Robotics.  
_Disclaimer:_ There may be some messages that are not included as Maritime Robotics continues to add more commands to their system. All of the commands available at the time this project was created are included.  
The modified files from the package are listed below and a modified version of the package is included in the _pynmeagps_ folder.
To use this package you can add a default version of it to your python distribution using pip or a similar process, then find the install folder and add/replace the files with the modified files in this project or add my changes to the appropriate files.

- **nmeatypes_core.py**: added PMAR as a talker
- **nmeatypes_get.py**: added all of the messages coming from the Otter (e.g. GPS, IMU, etc)
- **nmeatypes_set.py**: added all of the messages going to the Otther (e.g. ABT, MAN, etc)

## odometry with robot_localization navsat transform (experimental)

This package uses [robot_localization](docs.ros.org/en/melodic/api/robot_localization/html) to create an Odometry message using the Otter's GPS and IMU. You can launch the navsat transform with the other nodes via `ros2 launch otter_ros otter_navsat_transform_launch.py`. The package is configured to work in Kingston Ontario; which has a magnetic declination of 8.13 degrees (0.141953445247 radians) as of July 2023. Before use, change parameter "magnetic_declination" in config/ekf_with_gps.yaml to the [magnetic declination](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?#declination) of your USV's operating environment. This aspect of the package has not been formally tested and is still under development

#### Subscribed Topics

- **`/otter_gps`** ([sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)): GPS lat, long, alt

- **`/otter_imu`** ([sensor_msgs/Imu](https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)): orientation and angular velocities

#### Published Topics

- **`/otter_odom`**: Odometry message containing the Otter's position in a local frame as derived by GPS and IMU data

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://code.engineering.queensu.ca/offroad/otter-ros/-/issues).

### Roadmap

- [ ] Add support for more custom Otter messages
- [ ] Add support for more controllers
- [ ] Add support for more input to the Otter
- [ ] Add support for camera output from the Otter
- [ ] Add flexiblity in config files to incorporate (or toggle use of) more sensors

[ROS2]: http://www.ros.org
