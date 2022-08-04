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
        # self.macros.pickupOnePiLit()
        print(self.interface.Calibrate_client_goal())
        targetGlobal = self.interface.getPlacementPoints()
        print(targetGlobal)
        # point1 = self.interface.CoordConversion_client_goal([40.47411757814349, -104.9694349989295])
        # point2 = self.interface.CoordConversion_client_goal([40.474141730247005, -104.96941578849388])
        # point3 = self.interface.CoordConversion_client_goal([40.47416588234732, -104.96939657804444])
        # point4 = self.interface.CoordConversion_client_goal([40.47419003444447, -104.96937736758117])
        # point5 = self.interface.CoordConversion_client_goal([40.47421418653845, -104.9693581571041])
        # # point6 = self.interface.CoordConversion_client_goal([40.47404009733802, -104.96954446111727])

        point1 = self.interface.CoordConversion_client_goal([40.474175874801,-104.969360084 ])
        # # point2 = self.interface.CoordConversion_client_goal([40.4739442, -104.969698])
        # # point3 = self.interface.CoordConversion_client_goal([40.473968299999996, -104.96965089999999])
        # # point4 = self.interface.CoordConversion_client_goal([40.473991, -104.9696108])
        # # point5 = self.interface.CoordConversion_client_goal([40.4740143, -104.9695814])
        # # point6 = self.interface.CoordConversion_client_goal([40.4740385, -104.9695536])

        # point1 = self.interface.CoordConversion_client_goal([40.4738434, -104.9696352])
        # # point2 = self.interface.CoordConversion_client_goal([40.4738715, -104.9697072])


        target = self.interface.convertPointsToTruckCoordinates(targetGlobal)
        # self.interface.PP_client_goal(target)
        target2 = [point1]

        self.macros.placeAllPiLits(target)

        # # # # # target = self.interface.getPlacementPoints()

        # points = self.interface.getLatestSqlPoints()

        # point7 = self.interface.CoordConversion_client_goal([40.47406707881761, -104.96963093669557])
        # point8 = self.interface.CoordConversion_client_goal([40.47395230869571, -104.96976437659356])
        # point9 = self.interface.CoordConversion_client_goal([40.4738521, -104.9698155])
        # target2= [point7, point8, point9]
        # self.interface.PP_client_goal(target2)

        # # self.macros.placeAllPiLitsNoMovement(6)

        self.interface.PP_client_goal(target2)
        
        # mirv_control.msg.DatabaseGoal.SendLatest = True
        # self.goal = mirv_control.msg.DatabaseGoal
        # self.client.send_goal(self.goal)
        # self.client.wait_for_result()
        # placedPiLitLocations = self.client.get_result()
        # points = [[placedPiLitLocations.latitude[i], placedPiLitLocations.longitude[i]] for i in range(len(placedPiLitLocations.latitude))]
        # print(points)

        points = self.interface.getLatestSqlPoints()
    
        
        print(points)
        self.macros.pickupAllPiLits(points, False)
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
