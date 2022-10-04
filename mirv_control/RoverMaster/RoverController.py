#!/usr/bin/env python3

# Effectively Robot Main, responsible for
# periodicly checking for faults,
# managing hierarchy of systems
# calling macros or subsystem calls

import time
import RoverInterface
import rospy
from RoverInterface import RoverInterface
from RoverMacros import roverMacros as RoverMacros


class RoverController():
    def __init__(self):
        rospy.init_node("RoverController")
        self.rate = rospy.Rate(1)
        self.interface = RoverInterface()
        self.macros = RoverMacros(self.interface)
        self.interface.loadRoverMacro(self.macros)

    def updateStatus(self):
        while not rospy.is_shutdown():
            self.interface.stateWatchdog()
            self.rate.sleep()
        print("exited loop")

    def main(self):
        print("RoverController MAIN")
        self.interface.setPiLits("idle")


if __name__ == "__main__":
    controller = RoverController()
    time.sleep(4)
    controller.main()
    controller.updateStatus()
