#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class JoystickDrive:
    def __init__(self):
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.intake_command_pub = rospy.Publisher("intake/command", String, queue_size = 10)

    def joy_callback(self, joy_msg: Joy):
        pass

    def send_intake_command(self, cmd: str):
        self.intake_command_pub.publish(cmd)

