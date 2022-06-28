#!/usr/bin/env python3
import rospy
import sys
sys.path.append("..")
from Controllers.joystickdrive import JoystickDrive

def run():
    joystick_drive = JoystickDrive()

if __name__ == "__main__":
    run()