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

class RoverController():
    def __init__(self):
        rospy.init_node("RoverController")
        self.rate = rospy.Rate(1)
        self.interface = RoverInterface()
        # self.interface.run()
    def updateStatus(self):
        while not rospy.is_shutdown():
            # print("looping")
            self.interface.cloudController_client_goal()
            self.rate.sleep()

    def main(self):
        point1 = self.interface.CoordConversion_client_goal([40.4741943, -104.9692452])
        point2 = self.interface.CoordConversion_client_goal([40.4742012, -104.9693351])
        point3 = self.interface.CoordConversion_client_goal([40.4741130, -104.9694685])
        point4 = self.interface.CoordConversion_client_goal([40.4740620, -104.9694769])
        # point5 = self.interface.CoordConversion_client_goal([40.4740181, -104.9694273])
        # # point6 = self.interface.CoordConversion_client_goal([40.4741910, -104.9692516])
        # # point7 = self.interface.CoordConversion_client_goal([40.4741910, -104.9692516])
        # # point8 = self.interface.CoordConversion_client_goal([40.4741910, -104.9692516])
        # # point2 = self.interface.CoordConversion_client_goal([40.4742288, -104.9692942])
        print(self.interface.Calibrate_client_goal())
        # # print(point2)
        # target = [point1]
        # estimatedPiLitAngle = self.interface.PP_client_goal(target)
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
