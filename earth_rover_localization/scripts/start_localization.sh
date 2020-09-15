#!/bin/bash

mount=$1

rosrun earth_rover_localization get_first_RTK.py $(rospack find earth_rover_localization)/cfg

if [ ${mount} == "tractor" ]
then
   roslaunch earth_rover_localization scouting_localization_node.launch
elif [ ${mount} == "rover" ]
then
   roslaunch earth_rover_localization rover_localization_node.launch 
else
   echo "ERROR: ${mount} is not an acceptable argument"
fi