#!/bin/bash

export ROS_MASTER_URI=http://192.168.100.10:11311
export ROS_HOST_NAME=192.168.100.10
export ROS_IP=192.168.100.10
rosparam load `rospack find abot04_driver`/config/controll.yaml
rosparam load `rospack find abot04_driver`/config/kinematics.yaml
while true
do
    python3 `rospack find abot04_driver`/script/twist_driver.py
done
