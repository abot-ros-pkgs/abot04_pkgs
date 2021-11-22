#!/bin/bash
# Basical rosmaster
export ROS_MASTER_URI=http://192.168.100.10:11311
export ROS_HOST_NAME=192.168.100.10
export ROS_IP=192.168.100.10
roscore &
echo "Wait 10 sec for roscore"
sleep 10

# twist_driver
source `rospack find abot04_driver`/script/twist_driver_run.bash &
