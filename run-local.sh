#!/bin/bash

. ./setup.sh
ROS_DOMAIN_ID=45 ros2 service call /start_motor std_srvs/srv/Empty
ROS_DOMAIN_ID=45 ros2 launch rover_gazebo rover.launch.py world:=rover_room.world