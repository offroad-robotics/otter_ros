# OtterROS

The OtterROS paper is available on arXiv: [OtterROS: Picking and Programming an Uncrewed Surface Vessel for Experimental Field Robotics Research with ROS 2](https://arxiv.org/abs/2404.05627).

## Overview

<img src="https://i.imgur.com/4PBpnlb.png" alt="Otter ROS Logo" width="200"/>

OtterROS is a suite of ROS 2 packages for the [Maritime Robotics] Otter USV, including the core `otter_ros` interface package that enables ROS 2 on the Otter USV. Created for robotics research purposes by [Offroad Robotics] at [Queen's University].

The Otter ROS package has been developed and tested on [ROS 2] Foxy Fitzroy and Ubuntu 20.04 LTS.

If you have any questions, please create an issue or reach out to the team at [Offroad Robotics].

**Creators**: Riley Cooper, Sabrina Button, Thomas Sears, and Joshua Marshall.

**Affiliation**: [Offroad Robotics] is a member of the [Ingenuity Labs Research Institute] at [Queen's University].

**Keywords:** otter usv, maritime robotics, ros2, python, nmea, navigation, control

### Acknowledgements

Development of OtterROS included field tests within the traditional territories of the Anishinabee, Mississauga, Algonquin, Haudenosaunee, and Huron-Wendat.

### Funding

This project was funded in part by the NSERC Canadian Robotics Network (NCRN) under NSERC project NETGP 508451-17, the Vanier Canada Graduate Scholarships program, and Queen’s University’s USSRF program. The Otter USV was purchased with generous support from the Ingenuity Labs Research Institute’s Research Equipment Fund.

### Referencing OtterROS

The first release was accompanied by a paper accepted at the [Workshop on Field Robotics](https://norlab-ulaval.github.io/workshop_field_robotics_icra2024/) at [ICRA 2024](https://2024.ieee-icra.org/). If you use OtterROS in your research or published works, please use the following citation so we can see what other users are up to!

```latex
@inproceedings{sears2024otterros,
  title     = {{OtterROS}: Picking and Programming an Uncrewed Surface Vessel for Experimental Field Robotics Research with {ROS 2}},
  author    = {Thomas M. C. Sears, M. Riley Cooper, Sabrina R. Button, and Joshua A. Marshall},
  booktitle = {{IEEE} {ICRA} Workshop on Field Robotics},
  year      = {2024}
}
```

<!-- ArXiv reference left here for posterity -->
<!-- ```latex
@misc{sears2024otterros,
      title={{OtterROS}: Picking and Programming an Uncrewed Surface Vessel for Experimental Field Robotics Research with {ROS} 2}, 
      author={Thomas M. C. Sears and M. Riley Cooper and Sabrina R. Button and Joshua A. Marshall},
      year={2024},
      eprint={2404.05627},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
``` -->

## Package Directory

### `otter_ros`
This is the main package of OtterROS. It establishes the interface between the back seat driver architecture of the Otter USV and ROS 2.

This is a **required** package.

### `otter_msgs`
This interface (i.e., message) package is needed to use `otter_ros`. It defines custom message structures that simplify using the Otter in ROS 2.

This is a **required** package.

### `otter_control`
This package is for external command and control of the Otter through ROS.

This is an *optional* package.

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
[Ingenuity Labs Research Institute]: https://ingenuitylabs.queensu.ca/
