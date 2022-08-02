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
        self.macros.pickupOnePiLit()
        # print(self.interface.Calibrate_client_goal())
        # point1 = self.interface.CoordConversion_client_goal([40.47392328681292, -104.96971142862967])
        # # point2 = self.interface.CoordConversion_client_goal([40.47394062987316, -104.96967924212126])
        # point3 = self.interface.CoordConversion_client_goal([40.47396868481405, -104.96964035009026])
        # # point4 = self.interface.CoordConversion_client_goal([40.47399571956421, -104.96960279916378])
        # point5 = self.interface.CoordConversion_client_goal([40.47401561305263, -104.96957128320763])
        # # point6 = self.interface.CoordConversion_client_goal([40.47404009733802, -104.96954446111727])

        # # point1 = self.interface.CoordConversion_client_goal([40.473920199999995, -104.9697041])
        # # point2 = self.interface.CoordConversion_client_goal([40.4739442, -104.969698])
        # # point3 = self.interface.CoordConversion_client_goal([40.473968299999996, -104.96965089999999])
        # # point4 = self.interface.CoordConversion_client_goal([40.473991, -104.9696108])
        # # point5 = self.interface.CoordConversion_client_goal([40.4740143, -104.9695814])
        # # point6 = self.interface.CoordConversion_client_goal([40.4740385, -104.9695536])

        # # point1 = self.interface.CoordConversion_client_goal([40.4738434, -104.9696352])
        # # point2 = self.interface.CoordConversion_client_goal([40.4738715, -104.9697072])


        # target = [point1, point3, point5]
        # # target = [point1, point6]

        # self.macros.placeAllPiLits(target)

        # # # # # target = self.interface.getPlacementPoints()

        # points = self.interface.getLatestSqlPoints()

        # point7 = self.interface.CoordConversion_client_goal([40.47406707881761, -104.96963093669557])
        # point8 = self.interface.CoordConversion_client_goal([40.47395230869571, -104.96976437659356])
        # point9 = self.interface.CoordConversion_client_goal([40.4738521, -104.9698155])
        # target2= [point7, point8, point9]
        # self.interface.PP_client_goal(target2)

        # # self.macros.placeAllPiLitsNoMovement(6)

        # # self.interface.PP_client_goal(target)
        
        # # mirv_control.msg.DatabaseGoal.SendLatest = True
        # # self.goal = mirv_control.msg.DatabaseGoal
        # # self.client.send_goal(self.goal)
        # # self.client.wait_for_result()
        # # placedPiLitLocations = self.client.get_result()
        # # points = [[placedPiLitLocations.latitude[i], placedPiLitLocations.longitude[i]] for i in range(len(placedPiLitLocations.latitude))]
        # # print(points)

        # # target3 = points

        # # self.interface.PP_client_goal(target3)

        # self.macros.pickupAllPiLits(points, False)
        # point1 = self.interface.CoordConversion_client_goal([40.4741954, -104.9692536])
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
