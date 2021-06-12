#!/bin/bash

. ./setup.sh
ROS_DOMAIN_ID=45 ros2 service call /stop_motor std_srvs/srv/Empty