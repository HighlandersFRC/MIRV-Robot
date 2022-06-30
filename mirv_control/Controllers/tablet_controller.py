#!/usr/bin/env python3
import rospy
from robot_controller import RobotController
from std_msgs.msg import String
import json

mirv = RobotController()

def cloud_callback(json_string):
    drive = 0
    turn = 0

    commands = json.loads(json_string)
    for key in commands:
        if key == "joystick_x":
            turn = commands[key]
        if key == "joystick_y":
            drive = commands[key]
        if key == "intake":
            mirv.set_intake_state(commands[key])

    left = drive - turn
    right = -drive - turn
    if "joystick_x" in commands or "joystick_y" in commands:
        mirv.power_drive(left, right)

def run():
    rospy.init_node("TabletDrive")

    cloud_command_sub = rospy.Subscriber("CloudCommand", String, cloud_callback)

    while not rospy.is_shutdown():
        pass

if __name__ == "__main__":
    run()