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
        point1 = self.interface.CoordConversion_client_goal([40.4742168, -104.9692674])
        point2 = self.interface.CoordConversion_client_goal([40.4742219, -104.9692925])
        print(point2)
        target = [point1,point2]
        # target = [[1, -16.6304], [1,1]]
        # target = [[1,-1]]
        self.interface.PP_client_goal(target)
        self.interface.pickup_client_goal("switch_right")
        point2 = self.interface.CoordConversion_client_goal([40.4742168, -104.9692674])
        target = [point2,[1,-1]]
        self.interface.PP_client_goal(target)

if __name__ == "__main__":
    controller = RoverController()
    updateStatusThread = threading.Thread(target = controller.updateStatus, name="updateStatus")
    mainThread = threading.Thread(target = controller.main, name = "thread2")

    updateStatusThread.start()
    mainThread.start()

    print("after")
