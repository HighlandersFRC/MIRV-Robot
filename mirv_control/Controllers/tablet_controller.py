#!/usr/bin/env python3
import rospy
from robot_controller import RobotController
from std_msgs.msg import String
import json
import logging

mirv = RobotController()


DRIVING_SCALE = 0.5
TURNING_SCALE = 0.5


def cloud_callback(json_string):
    if not json_string or not json_string.data:
        return

    try:
        commands = json.loads(json_string.data)
    except ValueError:
        logging.error(f"Unable to parse json from command: {json_string.data}")
        return

    subsystem = commands.get('subsystem')
    command = commands.get('command', '')
    parameters = commands.get('parameters', {})

    if subsystem == "general":
        print('general')
    elif subsystem == "intake":
        print('intake')
        mirv.set_intake_state(command)
    elif subsystem == "drivetrain":
        print('drivetrain')
        if command == "arcade":
            turn = parameters.get('x', 0) * TURNING_SCALE
            drive = parameters.get('y', 0) * DRIVING_SCALE
            left = -drive + turn
            right = drive + turn
            logging.debug(f"Sending drivetrain command to rover: left: {}")
            mirv.power_drive(left, right)
        elif command == "tank":
            left = parameters.get('x', 0) * DRIVING_SCALE
            right = parameters.get('y', 0) * DRIVING_SCALE
            mirv.power_drive(left, right)
        else:
            logging.error(f"Invalid drivetrain command: {command}")
    else:
        logging.error(f"Invalid command subsystem: {subsystem}")


def run():
    rospy.init_node("TabletDrive")

    cloud_command_sub = rospy.Subscriber(
        "CloudCommands", String, cloud_callback)

    while not rospy.is_shutdown():
        pass


if __name__ == "__main__":
    run()
