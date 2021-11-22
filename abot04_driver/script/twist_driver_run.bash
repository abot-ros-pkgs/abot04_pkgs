#!/bin/bash
echo "Start param loading"
rosparam load `rospack find abot04_driver`/config/controll.yaml
rosparam load `rospack find abot04_driver`/config/kinematics.yaml
echo "Start driver"
rosrun abot04_driver odom_publisher.py &
while true
do
    python3 `rospack find abot04_driver`/script/twist_driver.py
done
