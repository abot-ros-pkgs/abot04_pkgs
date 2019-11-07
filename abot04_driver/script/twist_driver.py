import rospy
from geometry_msgs.msg import Twist

import odrive
from odrive.enums import *
import math as m

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
        self.odrv0.axis0.controller.vel_setpoint = -right_pulse
        self.odrv0.axis1.controller.vel_setpoint = left_pulse
        
if __name__ == '__main__':
    rospy.init_node("abot04_twist_driver")
    atd = Abot04TwistDriver()
    rospy.spin()
