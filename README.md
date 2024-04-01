# OtterROS

## Overview

<img src="https://i.imgur.com/4PBpnlb.png" alt="Otter ROS Logo" width="200"/>

OtterROS is a suite of ROS 2 packages for the [Maritime Robotics] Otter USV, including the core `otter_ros` interface package that enables ROS 2 on the Otter USV. Created for robotics research purposes by [Offroad Robotics] at [Queen's University].

The Otter ROS package has been developed and tested on [ROS 2] Foxy Fitzroy and Ubuntu 20.04 LTS.

Authors: [Riley Cooper](15mrc5@queensu.ca), [Sabrina Button](sabrina.button@queensu.ca), [Thomas Sears](thomas.sears@queensu.ca), [Joshua Marshall](joshua.marshall@queensu.ca)
Affiliation: [Offroad Robotics], [Queen's University]

**Keywords:** otter usv, maritime robotics, ros2, python, nmea, navigation, control

### Referencing OtterROS

The first public release was accompanied by a paper and poster at the [Workshop on Field Robotics](https://norlab-ulaval.github.io/workshop_field_robotics_icra2024/) at [ICRA 2024](https://2024.ieee-icra.org/). If you use OtterROS in your research or published works, please use the following citation so we can see what other users are up to!

```latex
@inproceedings{offroad_otter_ros,
  title={{OtterROS}: Picking and Programming an Uncrewed Surface Vessel for Experimental Field Robotics Research with {ROS 2}},
  author={Thomas M. C. Sears, M. Riley Cooper, Sabrina R. Button, and Joshua A. Marshall},
  booktitle={2024 {IEEE} International Conference on Robotics and Automation {(ICRA)}}
  seriestitle={Workshop on Field Robotics},
  year={2024}
}
```

## Package Directory

### `otter_ros`
This is the main package of OtterROS. It establishes the interface between the back seat driver architecture of the Otter USV and ROS 2.

This is a required package.

### `otter_msgs`
This interface (i.e., message) package is needed to use `otter_ros`. It defines custom message structures that simplify using the Otter in ROS 2.

This is a required package.

### `otter_control`
This package is for controllers that use `otter_ros` to control the Otter.

This is an optional package.

## Installation

### Dependencies

Required:
- [Robot Operating System 2 (ROS 2)](https://docs.ros.org/en/foxy/index.html) (Originally developed for "Foxy Fotzroy")

Optional:
- [Velodyne driver](https://github.com/ros-drivers/velodyne): Velodyne Puck (VLP-16) LiDAR
- [SBG ROS 2 driver](https://github.com/SBG-Systems/sbg_ros2_driver): SBG Ellipse-D IMU


### Building

To build from source, clone the latest version from this repository into your ROS 2 workspace, source ROS 2, and compile the package using:

```bash
cd ros2_ws/src
git clone https://github.com/offroad-robotics/otter_ros.git
cd ../
rosdep install --from-paths . --ignore-src
colcon build --symlink-install
```

## Usage

After build, source `ros2_ws`, then start the core functions of `otter_ros`:

```bash
source ros2_ws/install/local_setup.bash
ros2 launch otter_ros otter_base_launch.py
```

See individual OtterROS packages for explanations and example usage of lower level functions.

## Bugs & Feature Requests

Please report bugs and request features using the Issue Tracker.

[ROS 2]: http://www.ros.org
[Offroad Robotics]: https://offroad.engineering.queensu.ca/
[Queen's University]: https://queensu.ca
[Maritime Robotics]: https://www.maritimerobotics.com/