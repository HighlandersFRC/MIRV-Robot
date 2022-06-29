#!/usr/bin/env python3
import rospy
from joystickdrive_controller import JoystickDrive

def run():
    rospy.init_node("RemoteDrive")
    joystick_drive = JoystickDrive()
    while not rospy.is_shutdown():
        intake_command = input("Intake command: ")
        joystick_drive.send_intake_command(intake_command)


if __name__ == "__main__":
    run()