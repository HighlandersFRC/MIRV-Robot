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

        # rospy.init_node("DatabaseActionClient")
        self.client = actionlib.SimpleActionClient("Database", mirv_control.msg.DatabaseAction)
        self.client.wait_for_server()
        # self.interface.run()
    def updateStatus(self):
        while not rospy.is_shutdown():
            # print("looping")
            self.interface.cloudController_client_goal()
            self.rate.sleep()

    def main(self):
        self.interface.garage_client_goal(0)
        # self.macros.pickupOnePiLit()
        # print(self.interface.Calibrate_client_goal())
        # # point1 = self.interface.CoordConversion_client_goal([40.47418975564257, -104.96924959123135])
        # point2 = self.interface.CoordConversion_client_goal([40.47421934074345, -104.96928915381432])
        # point3 = self.interface.CoordConversion_client_goal([40.474175983263564,  -104.96935822069644])
        # point4 = self.interface.CoordConversion_client_goal([40.4740867177757, -104.96949836611748])
        # point5 = self.interface.CoordConversion_client_goal([40.4740362189614, -104.96944606304169])
        # # # point6 = self.interface.CoordConversion_client_goal([40.47404009733802, -104.96954446111727])

        # target = [point2, point3, point4, point5]
        # self.interface.PP_client_goal(target)
        
        # # mirv_control.msg.DatabaseGoal.SendLatest = True
        # # self.goal = mirv_control.msg.DatabaseGoal
        # # self.client.send_goal(self.goal)
        # # self.client.wait_for_result()
        # # placedPiLitLocations = self.client.get_result()
        # # points = [[placedPiLitLocations.latitude[i], placedPiLitLocations.longitude[i]] for i in range(len(placedPiLitLocations.latitude))]

        # self.macros.pickupAllPiLits(points, False)
if __name__ == "__main__":
    controller = RoverController()
    updateStatusThread = threading.Thread(target = controller.updateStatus, name="updateStatus")
    mainThread = threading.Thread(target = controller.main, name = "thread2")

    updateStatusThread.start()
    mainThread.start()

    print("after")
