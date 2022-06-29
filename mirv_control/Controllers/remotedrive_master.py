#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from robot_controller import RobotController

mirv = RobotController()

"""
Joystick Buttons: 
0 - A - intake disable
1 - B - intake reset
2 - X - intake intake
3 - Y - intake store
4 - LB - intake deposit
5 - RB
6 - Back
7 - Start
8 - Left Stick In
9 - Right Stick In
10 - LT
11 - RT
"""

def joy_callback(msg):
    drive = msg.axes[1]
    turn = msg.axes[3]
    left = drive - turn
    right = -drive - turn
    mirv.power_drive(left, right)

    buttons = msg.buttons
    if buttons[0]:
        mirv.set_intake_state("disable")
    if buttons[1]:
        mirv.set_intake_state("reset")
    if buttons[2]:
        mirv.set_intake_state("intake")
    if buttons[3]:
        mirv.set_intake_state("store")
    if buttons[4]:
        mirv.set_intake_state("deposit")

def run():
    rospy.init_node("RemoteDrive")

    joy_sub = rospy.Subscriber("joy", Joy, joy_callback)

    while not rospy.is_shutdown():
        pass

if __name__ == "__main__":
    run()