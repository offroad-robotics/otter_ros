# Otter Msgs

## Overview

<img src="https://i.imgur.com/P7wIyZF.png" alt="Otter Msgs Logo" width="200"/>

A ROS 2 custom message repository for use with OtterROS.

## Message Types

### `OtterStatus.msg`

A message which includes the power data and status of the Otter USV.

Message data:

    std_msgs/Header header
    BilateralStatus starboard # see BilateralStatus
    BilateralStatus port
    float64 power # total power as a percentage
    int64 batteries # number of batteries onboard

### `BilateralStatus.msg`

A message which includes the power data and status of a single side of the Otter USV.

Message data:

    std_msgs/Header header
    float64 rpm # revolutions per minute of the motor on this side
    float64 temp # temperature of the motor on this side
    float64 power # power as a percentage of the total power

### `COGSOG.msg`

A message which includes the COG, SOG and resultant vector of the Otter USV.

Message data:

    std_msgs/Header header
    float64 cog # course over ground in degrees
    float64 sog # speed over ground in m/s
    geometry_msgs/Twist twist # linear velocity in m/s

### `OtterTime.msg`

A message which includes the time according the Otter USV (and not according to ROS).

Message data:

    std_msgs/Header header
    int64 time # time in seconds since the epoch

