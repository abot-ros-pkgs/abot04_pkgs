import rospy
from geometry_msgs.msg import Twist

import odrive
from odrive.enums import *
import math as m
import sys

class Abot04TwistDriver():
    def __init__(self):
        # Odrive setup
        while True:
            print("Openning ODrive....")
            self.odrv0 = odrive.find_any()
            if self.odrv0 is not None:
                print("ODrive is Opened !!")
                break
            else:
                print("Failed to open ODrive...Try Again!!")
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.vel_setpoint = 0
        self.odrv0.axis1.controller.vel_setpoint = 0
        # subscriber
        rospy.Subscriber("cmd_vel", Twist, self.TwistCB)
        # parameter
        self.right_wheel_radius = rospy.get_param("/abot04/right/wheel_radius")
        self.left_wheel_radius = rospy.get_param("/abot04/left/wheel_radius")
        self.tread = rospy.get_param("/abot04/tread")
        self.linear_vel_limit = rospy.get_param("/abot04/linear/vel_limit")
        self.linear_acc_limit = rospy.get_param("/abot04/linear/acc_limit")
        self.angular_vel_limit = rospy.get_param("/abot04/angular/vel_limit")
        self.angular_acc_limit = rospy.get_param("/abot04/angular/acc_limit")
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0
        print("Initialized")

    def TwistCB(self, data):
        # Check if the odrv0 is able to use
        if self.odrv0 is None:
            sys.exit(1)
        
        target_linear = data.linear.x
        target_angular = data.angular.z
        # Bounds check
        if abs(data.linear.x) > abs(self.linear_vel_limit):
            target_linear = data.linear.x/abs(data.linear.x) * self.linear_vel_limit
        if abs(data.angular.z) > abs(self.angular_vel_limit):
            target_angular = data.angular.z/abs(data.angular.z)*self.angular_vel_limit
        
        right_speed = target_linear + (self.tread/2.)*target_angular
        left_speed = target_linear - (self.tread/2.)*target_angular
        right_pulse = right_speed/(m.pi*self.right_wheel_radius)*900.
        left_pulse = left_speed/(m.pi*self.left_wheel_radius)*900.
        try:
            self.odrv0.axis0.controller.vel_setpoint = -right_pulse
            self.odrv0.axis1.controller.vel_setpoint = left_pulse
        except AttributeError as error:
            # Output expected AttributeErrors.
            rospy.signal_shutdown(error)
        except KeyboardInterrupt:
            self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
            self.odrv0.axis1.requested_state = AXIS_STATE_IDLE
            rospy.signal_shutdown("KeyboardInterrupt")
            
if __name__ == '__main__':
    rospy.init_node("abot04_twist_driver",disable_signals=True)
    atd = Abot04TwistDriver()
    rospy.spin()
