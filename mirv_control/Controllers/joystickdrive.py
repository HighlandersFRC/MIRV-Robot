#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy

class JoystickDrive:
    def __init__(self):
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)

    def joy_callback(self, joy_msg: Joy):
        pass