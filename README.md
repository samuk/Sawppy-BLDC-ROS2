
Thoughts on ROS2 Open Hardware robot

# BOM (mostly) open hardware

- ~£400 Open source [Jetson baseboard](https://capablerobot.com/products/nx-baseboard/) start with a Nano compute module, upgrade if needed. 
- ~£300 [Tenacity rover body](https://github.com/jetdillo/tenacity_rover#readme)
- ~£40 [Improved suspension from SRA](https://twitter.com/SmallRobotArmy/status/1476667953546346530)
- ~£20 [Open Core running uROS](https://github.com/rosmo-robot/Open-Core-M5stack/blob/main/README.md)
- ~£90 [3 x dual channel SimpleFOC motor controllers](https://github.com/rosmo-robot/Rosmo_ESC)
- ~100[Tinkerforge IMU](https://www.tinkerforge.com/en/shop/bricks/imu-v2-brick.html) [ROS2 ](https://discourse.ros.org/t/ros-tinkerforge-imu-v2-bricks-driver/15539)
- ~£240 [6x 4108 BLDC motor TBD](https://s.click.aliexpress.com/e/_AE2SCu)
- ~70 [4 x 35kg servo](https://www.hiwonder.hk/products/hiwonder-hts-35h-high-voltage-bus-servo-35kg-torque-with-data-feedback)
- ~£100 Jehu [18650 battery pack](https://jag35.com/collections/pcb-based-products/products/high-power-18650-battery-module-diy-pcb-kit-75x)

# optional extras
- ~£170 [ZED-F9P Sparkfun RTK](https://www.ardusimple.com/rtk-open-source-hardware/) Or maybe [$$Ark](https://arkelectron.com/product/ark-rtk-gps/)
- ~£200 Hang a [Nindmani Delta](https://github.com/AutoRoboCulture/nindamani-the-weed-removal-robot)
- ~£40 Meshtastic for [RTK comms](https://meshtastic.discourse.group/) latency a problem?

# OS / Basic nav
- [Dashing on Jetson Nano](https://github.com/ANI717/Headless-Jetson-Nano-Setup)
- [Tensorflow model to follow crop rows - Output Cmd_vel](https://github.com/ANI717/ANI717_Robotics#design-diagram)
- [Swappy ROS2 Rover for Ackerman drive - SubscribeCmd_vel](https://github.com/mgonzs13/ros2_rover)

# Ag Navigation notes (later)

![Visual & ML](https://pbs.twimg.com/media/FIRSEUpXoA8Sf_V?format=jpg&name=900x900)

The core of this navigation strategy is the VisualServoing and the local planner packages



# Notes
- https://github.com/NMBURobotics/vox_nav
- Maybe add 4x4 steer using BLDC to [Rover](https://github.com/tlalexander/rover_designs)
- [Radxa CM3 module](https://www.cnx-software.com/2021/11/07/radxa-cm3-raspberry-pi-cm4-alternative/) in [Carrier board](https://hackaday.io/project/165108-carrier-board-for-the-raspberry-pi-compute-module) probably underpowered.
- [Acorn Motherboard?](https://github.com/Twisted-Fields/acorn-robot-electronics/blob/main/README.md) Unclear what value added if going ROS2 route.
- [Nema23 BLDC](https://www.aliexpress.com/item/32799131056.html) for steering, if starting from Jetbot may need skid to start with.



Overview
------

Looks like original Earth Rover is running on Xavier, Ubuntu 20.04 & Foxy? Dashing?

[Teb-Local-Planner](https://github.com/rst-tu-dortmund/teb_local_planner/tree/foxy-devel)

[Robot_localization](https://github.com/cra-ros-pkg/robot_localization/tree/ros2)

[ROS2 RTK](https://github.com/aussierobots/ublox_dgnss)

Integrate [visual servoing](https://github.com/PRBonn/visual-crop-row-navigation#readme) or [Neural network somehow](https://github.com/samuk/ANI717_Robotics)?
- [earth_rover_localization](https://github.com/earthrover/earth_rover_localization/tree/master/earth_rover_localization): ROS package to configure the EKF of the robot_localization package. 
- 

Would need to edit to reflect alternate RTK/ IMU
- Uses sensor fusion of GPS [Piksy Multi](https://www.swiftnav.com/piksi-multi) and IMU [MTi-3 AHRS](https://www.xsens.com/products/mti-1-series/)
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
