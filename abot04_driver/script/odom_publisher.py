#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

class Abot04OdomPublisher():
    def __init__(self):
        #broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        #publisher
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
        #subscriber
        rospy.Subscriber("odom_buf", Vector3, self.bufCB)

        # values
        # for broadcaster
        self.t = TransformStamped()
        self.t.header.stamp = rospy.Time.now()
        self.t.header.frame_id = 'odom'
        self.t.child_frame_id = 'base_footprint'

        # for publisher
        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_footrprint"

        # for buffer
        self.odom_buf = Vector3()

        
    def bufCB(self, data):
        self.odom_buf.x = data.x
        self.odom_buf.y = data.y
        self.odom_buf.z = data.z

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            data = self.odom_buf
            time = rospy.Time.now()
            # Broadcaster
            self.t.transform.translation = data
            q = tf.transformations.quaternion_from_euler(0,0,data.z)
            rotation = Quaternion(*q)
            self.t.transform.rotation = rotation
            self.t.header.stamp = time
            self.br.sendTransform(self.t)
            # Publisher
            self.odom.pose.pose.position.x = data.x
            self.odom.pose.pose.position.y = data.y
            self.odom.pose.pose.position.z = 0.
            self.odom.pose.pose.orientation = rotation
            self.odom.header.stamp = time
            self.odom_pub.publish(self.odom)
            
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("abot04_odom_publisher")
    aop = Abot04OdomPublisher()
    aop.loop()
    rospy.spin()
