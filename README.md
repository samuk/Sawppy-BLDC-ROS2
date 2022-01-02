# earth_rover_localization [![Build Status](https://travis-ci.com/earthrover/OpenER.svg?branch=master)](https://travis-ci.com/earthrover/OpenER)

This repository is a fork of the Earth Rover Agribot. Contains thoughts on Open Hardware Ag robot & investigations into a ROS2 port.


# BOM open hardware 4x4 drive 4x4 steer

- [Radxa CM3 module](https://www.cnx-software.com/2021/11/07/radxa-cm3-raspberry-pi-cm4-alternative/) in [Carrier board](https://hackaday.io/project/165108-carrier-board-for-the-raspberry-pi-compute-module)
- [Acorn Motherboard?](https://github.com/Twisted-Fields/acorn-robot-electronics/blob/main/README.md)
- [ ZED-F9P Sparkfun RTK](https://www.ardusimple.com/rtk-open-source-hardware/) No [ROS2](https://github.com/ros-agriculture/ublox_f9p/issues/12) [ublox exists](https://index.ros.org/p/ublox_gps/github-KumarRobotics-ublox/)
- [4 x dual channel SimpleFOC motor controllers](https://github.com/rosmo-robot/Rosmo_ESC)
- [Tinkerforge IMU](https://www.tinkerforge.com/en/shop/bricks/imu-v2-brick.html) [ROS2](https://github.com/aussierobots/ublox_dgnss) [ROS2 yet](https://discourse.ros.org/t/ros-tinkerforge-imu-v2-bricks-driver/15539)
- Body TBD maybe add 4x4 steer using BLDC to [Rover](https://github.com/tlalexander/rover_designs)
- [Wheels](https://www.aliexpress.com/item/32839959696.html) 
- Hang a [DeltaX](https://www.deltaxrobot.com/) off the bottom?

 ![rover](https://github.com/tlalexander/rover_designs/raw/master/images/rover_beach.jpg)

Overview
------
- [earth_rover_localization](https://github.com/earthrover/earth_rover_localization/tree/master/earth_rover_localization): ROS package to configure the EKF of the robot_localization package. Uses sensor fusion of GPS [Piksy Multi](https://www.swiftnav.com/piksi-multi) and IMU [MTi-3 AHRS](https://www.xsens.com/products/mti-1-series/)
- [piksi_multi_rtk](https://github.com/earthrover/earth_rover_piksi): Repository that contains ROS driver and utilities for Piksi RTK receiver device.
- [xsens_mti_ros_node](https://github.com/xsens/xsens_mti_ros_node): ROS driver for third and fourth generation of Xsens IMU devices.

License
-------
The source code is released under a [BSD 3-Clause license](https://github.com/earthrover/er_localisation/blob/master/LICENSE.md).

Bugs & Feature Requests
-------
Please report bugs and request features using the [Issue Tracker](https://github.com/earthrover/er_localisation/issues).

Acknowledgement
-------
Earth Rover ROS – The Open-Source Agribot (http://open.earthrover.cc/)

Andres Palomino apalomino(at)earthrover.cc
Ricard Pardell ricard(at)earthrover.cc

<!--
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png"
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg"
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287.
