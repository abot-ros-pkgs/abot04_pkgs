#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math as m

class SwingLRF():
    def __init__(self):
        self.jt_pub = rospy.Publisher('/dynamixel_workbench/joint_trajectory', JointTrajectory, queue_size=1)
        self.jt = JointTrajectory()
        self.jt.joint_names.append("pan")
        self.jt.joint_names.append("tilt")

        self.jtpoint = JointTrajectoryPoint()
        self.jtpoint.positions.append(0)
        self.jtpoint.positions.append(-7./6.*m.pi)
        self.jtpoint.time_from_start = 1.0
        self.jt.points.append(self.jtpoint)
        self.jtpoint.positions[0] = 0
        self.jtpoint.positions[1] = -7./6.*m.pi
        self.jtpoint.time_from_start = 1.0
        self.jt.points.append(self.jtpoint)
        self.jt_pub.publish(self.jt)
        self.cnt=0
        print "Initialized"
        
    def run(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.jt.points[0].positions[0] = self.jt.points[0].positions[1]
            self.jt.points[0].positions[1] += m.pi/6.*m.cos(self.cnt%2*m.pi)
            self.jt.points[1].positions[0] = self.jt.points[1].positions[1]
            self.jt.points[1].positions[1] += -m.pi/6.*m.cos(self.cnt%2*m.pi)
            self.jt.points[0].time_from_start = 0.5
            self.jt.points[1].time_from_start = 0.5
            self.jt.header.stamp = rospy.Time.now()
            # self.jt_pub.publish(self.jt)
            self.cnt += 1
            print "loop"
            rate.sleep()
    
if __name__ == '__main__':
    rospy.init_node('swing_lrf', anonymous=True)
    sf = SwingLRF()
    sf.run()
