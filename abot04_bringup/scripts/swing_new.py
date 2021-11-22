#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand
import math as m

if __name__ == "__main__":
    rospy.init_node('abot04_dynamixel')
    rospy.wait_for_service('/abot04_dynamixel/dynamixel_command')
    try:
        proxy = rospy.ServiceProxy('/abot04_dynamixel/dynamixel_command',DynamixelCommand)
        
