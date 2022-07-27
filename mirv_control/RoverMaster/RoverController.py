#!/usr/bin/env python3

# Effectively Robot Main, responsible for 
# periodicly checking for faults,
# managing hierarchy of systems
# calling macros or subsystem calls

import RoverInterface
import mirv_control.msg
import rospy
import actionlib
from RoverInterface import RoverInterface
import threading
from PiLitController import PiLitControl as PiLitController
from RoverMacros import roverMacros as RoverMacros
from FaultState import FaultStates

class RoverController():
    def __init__(self):
        rospy.init_node("RoverController")
        self.rate = rospy.Rate(1)
        self.interface = RoverInterface()
        self.macros = RoverMacros(self.interface)
        self.faults = FaultStates(self.interface)
        # self.interface.run()
    def updateStatus(self):
        while not rospy.is_shutdown():
            # print("looping")
            self.interface.cloudController_client_goal()
            self.rate.sleep()

    def main(self):
        print(self.interface.Calibrate_client_goal())
        point1 = self.interface.CoordConversion_client_goal([40.473905, -104.969732])
        point2 = self.interface.CoordConversion_client_goal([40.473956, -104.969676])
        point3 = self.interface.CoordConversion_client_goal([40.474010, -104.969626])
        point4 = self.interface.CoordConversion_client_goal([40.474064, -104.969567])
        point5 = self.interface.CoordConversion_client_goal([40.474131, -104.969497])
        point6 = self.interface.CoordConversion_client_goal([40.474182, -104.969450])

        target = [point1, point2, point3, point4, point5, point6]

        # target = self.interface.getPlacementPoints()

        self.macros.placeAllPiLits(target)

        # target = [point1]
        # estimatedPiLitAngle = self.interface.PP_client_goal(target)
        # self.macros.placePiLit(4)
        # self.interface.pickup_client_goal("switch_right", 5)
        # target = [point2, point3, point4]
        # estimatedPiLitAngle = self.interface.PP_client_goal(target)
        # self.interface.pickup_client_goal("switch_left", 5)
        # target = [point5]
        # self.interface.PP_client_goal(target)
        # self.interface.pickup_client_goal("switch_right", 5)
if __name__ == "__main__":
    controller = RoverController()
    updateStatusThread = threading.Thread(target = controller.updateStatus, name="updateStatus")
    mainThread = threading.Thread(target = controller.main, name = "thread2")

    updateStatusThread.start()
    mainThread.start()

    print("after")
