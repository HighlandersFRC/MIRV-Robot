#!usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, String

class RobotController:
    def __init__(self):
        self.powerdrive_pub = rospy.Publisher("PowerDrive", Float64MultiArray, queue_size = 10)
        self.intake_command_pub = rospy.Publisher("intake/command", String, queue_size = 10)

    def set_intake_state(self, state: str):
        self.intake_command_pub.publish(state)

    def power_drive(self, left, right):
        powers = Float64MultiArray()
        powers.data = [left, right]
        self.powerdrive_pub.publish(powers)