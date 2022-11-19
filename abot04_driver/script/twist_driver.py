import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

import odrive
from odrive.enums import *
import math as m
import sys

class Abot04TwistDriver():
    def __init__(self):
        # Odrive setup
        while True:
            rospy.loginfo("Openning ODrive....")
            self.odrv0 = odrive.find_any()
            if self.odrv0 is not None:
                rospy.loginfo("ODrive is Opened !!")
                break
            else:
                print("Failed to open ODrive...Try Again!!")
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.input_vel = 0
        self.odrv0.axis1.controller.input_vel = 0
        # subscriber
        rospy.Subscriber("cmd_vel", Twist, self.TwistCB)
        # publisher
        self.odom_pub = rospy.Publisher("odom_buf", Vector3, queue_size=1)
        # parameter
        self.right_wheel_radius = rospy.get_param("/abot04/right/wheel_radius")
        self.left_wheel_radius = rospy.get_param("/abot04/left/wheel_radius")
        self.tread = rospy.get_param("/abot04/tread")
        self.linear_vel_limit = rospy.get_param("/abot04/linear/vel_limit")
        self.angular_vel_limit = rospy.get_param("/abot04/angular/vel_limit")
        self.last_right_vel = 0.0
        self.last_left_vel = 0.0
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.odom = Vector3()
        self.right_pos = -self.odrv0.axis0.encoder.pos_estimate*m.pi*self.right_wheel_radius
        self.left_pos = self.odrv0.axis1.encoder.pos_estimate*m.pi*self.right_wheel_radius
        self.last_right_pos = self.right_pos
        self.last_left_pos = self.left_pos
        rospy.loginfo("Initialized")

    def control(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # Calculate each velocity m/sec
            (right_vel,left_vel) = self.calcEachVel(self.target_linear_vel,self.target_angular_vel)
            # convert to round/sec
            right_rps = right_vel/(m.pi*self.right_wheel_radius)
            left_rps = left_vel/(m.pi*self.left_wheel_radius)
            try:
                # Get current position
                self.right_pos = -self.odrv0.axis0.encoder.pos_estimate*m.pi*self.right_wheel_radius
                self.left_pos = self.odrv0.axis1.encoder.pos_estimate*m.pi*self.right_wheel_radius

                right_pos_diff = self.right_pos - self.last_right_pos
                left_pos_diff = self.left_pos - self.last_left_pos
                self.calcOdom(right_pos_diff, left_pos_diff)

                # Set velocity
                self.odrv0.axis0.controller.input_vel = -right_rps
                self.odrv0.axis1.controller.input_vel = left_rps

                # Update last pos
                self.last_right_pos = self.right_pos
                self.last_left_pos = self.left_pos

                rate.sleep()

            except AttributeError as error:
                # Output expected AttributeErrors.
                rospy.signal_shutdown(error)
            except KeyboardInterrupt:
                self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
                self.odrv0.axis1.requested_state = AXIS_STATE_IDLE
                rospy.signal_shutdown("KeyboardInterrupt")

    def calcEachVel(self,target_linear,target_angular):
        # Bounds check
        if abs(target_linear) > abs(self.linear_vel_limit):
            target_linear = target_linear/abs(target_linear) * self.linear_vel_limit
        if abs(target_angular) > abs(self.angular_vel_limit):
            target_angular = target_angular/abs(target_angular)*self.angular_vel_limit
        # Convert to each vel
        right_vel = target_linear + (self.tread/2.)*target_angular
        left_vel = target_linear - (self.tread/2.)*target_angular
        return (right_vel, left_vel)
                
    def calcOdom(self,right_delta_dist, left_delta_dist):
        delta_linear = (right_delta_dist + left_delta_dist)/2.0
        delta_yaw = (right_delta_dist - left_delta_dist)/self.tread
        approximate_delta_linear = 0.0
        turn_rad = 0.0
        
        if abs(delta_yaw) < 245e-3 :
            self.odom.x = self.odom.x + delta_linear * m.cos(self.odom.z + (delta_yaw/2.)) 
            self.odom.y = self.odom.y + delta_linear * m.sin(self.odom.z + (delta_yaw/2.))
            self.odom.z = self.odom.z + delta_yaw
        else:
            turn_rad = delta_linear/delta_yaw
            approximate_delta_linear = 2.0*turn_rad*m.sin((delta_yaw/2.))
            self.odom.x = self.odom.x +  approximate_delta_linear * m.cos(self.odom.z + (delta_yaw/2.))
            self.odom.y = self.odom.y + approximate_delta_linear * m.sin(self.odom.z + (delta_yaw/2.))
            self.odom.z = self.odom.z + delta_yaw
        self.odom_pub.publish(self.odom)
        
    def TwistCB(self, data):
        self.target_linear_vel = data.linear.x
        self.target_angular_vel = data.angular.z

if __name__ == '__main__':
    rospy.init_node("abot04_twist_driver",disable_signals=True)
    atd = Abot04TwistDriver()
    atd.control()
    rospy.spin()
