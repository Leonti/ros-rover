#!/bin/bash

. ./setup.sh
ROS_DOMAIN_ID=45 ros2 launch rover_gazebo rover_ui.launch.py world:=rover_room.world