#!/usr/bin/env python3

import math
import rospy
import mirv_control.msg
import actionlib
import threading
from PiLitController import PiLitControl as PiLitController
from std_msgs.msg import String, Float64MultiArray
import time
import numpy as np
import placement
from RoverInterface import RoverInterface
from sensor_msgs.msg import NavSatFix


class roverMacros():
    def __init__(self, Interface: RoverInterface):
        self.interface = Interface

    def undock(self):
        self.interface.changeNeuralNetworkSelected("none")
        self.interface.deployGarage()
        self.interface.Calibrate_client_goal()
        self.interface.turn(math.pi, 4)

    def dock(self, estimatedAngleToGarage):
        self.interface.changeNeuralNetworkSelected("aruco")
        lineUpPoint1 = [5, 0]
        lineUpPoint2 = [3.5, 0]
        lineUpPoint3 = [1, 0]
        lineUpPoints = [lineUpPoint1, lineUpPoint2, lineUpPoint3]
        self.interface.deployGarage()
        self.interface.PP_client_goal(lineUpPoints)
        # point towards aruco tag
        # itentify lateral offset to aruco tag
        # turn to the lateral direction and drive the measure distance
        # turn back towards the garage
        self.interface.garage_client_goal(0)
        self.interface.retractGarage()

    def dockNoPathing(self):
        self.interface.changeNeuralNetworkSelected("aruco")
        self.interface.deployGarage()
        self.interface.garage_client_goal(0)

    def placePiLitFromSide(self, timeout, intakeSide):
        start_time = time.time()
        self.interface.intake_command_pub.publish(String(intakeSide))
        self.interface.intake_command_pub.publish(String("deposit"))
        while self.interface.limit_switches[1]:
            if time.time() - start_time > timeout:
                print("Timed out - Deposit")
                return False
        time.sleep(1)
        return True

    def placePiLit(self):
        stored = self.interface.getPiLitsStored()
        if not stored or len(stored) != 2:
            rospy.logerr("Invalid number of stored Pi-Lits")
            return
        if stored[0] > stored[1]:
            side = "switch_right"
        else:
            side = "switch_left"
        self.placePiLitFromSide(7, side)
        self.interface.intake_command_pub.publish(String("reset"))
        self.interface.loadPointToSQL("deploy", side)

    def placeAllPiLits(self, firstPoint, roughHeading, formation_type):
        self.interface.changeNeuralNetworkSelected("lanes")
        firstPointTruckCoord = self.interface.CoordConversion_client_goal(
            firstPoint)
        startingTarget = [firstPointTruckCoord]
        self.interface.PP_client_goal(startingTarget)
        detected_lanes = {'right': (firstPoint[0], firstPoint[1])}
        points = placement.generate_pi_lit_formation(
            detected_lanes, roughHeading, 3, formation_type)
        print("Calculated Placement Points: ", points)

        self.interface.changeNeuralNetworkSelected("none")
        intakeSide = "switch_right"

        for pnt in points:
            point = self.interface.CoordConversion_client_goal(pnt)
            print("Converting Point")
            target = [point]
            self.interface.PP_client_goal(target)
            print("Going to PP target")
            self.placePiLitFromSide(6, intakeSide)
            self.interface.intake_command_pub.publish(String("reset"))
            self.interface.loadPointToSQL("deploy", intakeSide)
            if(intakeSide == "switch_right"):
                intakeSide = "switch_left"
            else:
                intakeSide = "switch_right"
            time.sleep(2)
        return True

    def placeAllPiLitsNoMovement(self, count):
        intakeSide = "switch_right"
        for i in range(0, count):
            self.placePiLitFromSide(4, intakeSide)
            self.interface.intake_command_pub.publish(String("reset"))
            if(intakeSide == "switch_right"):
                intakeSide = "switch_left"
            else:
                intakeSide = "switch_right"
            time.sleep(4)

    def interceptPoint(self, finalPoint):
        currentPoint = np.array(self.interface.getCurrentTruckOdom())
        print(currentPoint)
        print(finalPoint)
        d = ((currentPoint[0]-finalPoint[0][0])**2 +
             (currentPoint[1]-finalPoint[0][1])**2)**0.5
        UV = np.array([(currentPoint[0]-finalPoint[0][0])/d,
                      (currentPoint[1]-finalPoint[0][1])/d])
        d2 = d
        if(d > 1):
            d2 = d-1
        TP = currentPoint - UV*d2
        return TP

    def pickupPiLit(self):
        self.interface.changeNeuralNetworkSelected("piLit")
        stored = self.interface.getPiLitsStored()
        if not stored or len(stored) != 2:
            rospy.logerr("Invalid number of stored Pi-Lits")
            return
        if stored[0] < stored[1]:
            side = "switch_right"
        else:
            side = "switch_left"
        self.interface.pickup_client_goal(side, 0)
        self.interface.loadPointToSQL("retrieve", side)
        self.interface.changeNeuralNetworkSelected("none")

    def testPointTurn(self):
        self.interface.pointTurn(180, 10)

    def testDriveDistance(self):
        self.interface.driveDistance(1, 0.25, .1)

    def pickupAllPiLits(self, lists, reverse):
        self.interface.changeNeuralNetworkSelected("piLit")
        intakeSide = "switch_right"

        if(reverse):
            lists.reverse()

        points = [[lists.latitude[i], lists.longitude[i]]
                  for i in range(len(lists.latitude))]
        for point in lists:
            print(f"POINT {point}")
            convertedPoint = [
                self.interface.CoordConversion_client_goal(point)]
            target = self.interceptPoint(convertedPoint)
            # placementX = convertedPoint[0]
            # placementY = convertedPoint[1]

            # currentLocationLongLat = [self.interface.getCurrentLatitude, self.interface.getCurrentLongitude]

            # navSatFixMsg = NavSatFix()
            # navSatFixMsg.latitude = currentLocationLongLat[0]
            # navSatFixMsg.longitude = currentLocationLongLat[0]
            # navSatFixMsg.altitude = 1492

            # currentLocationConverted = self.interface.CoordConversion_client_goal(currentLocationLongLat)

            # currentX = currentLocationConverted[0]
            # currentY = currentLocationConverted[1]

            # angleToPlacement = math.atan2(placementY, placementX)
            # distanceToPlacement = math.sqrt((math.pow(placementX - currentX, 2)) + (math.pow(placementY - currentY, 2)))
            # distanceBeforePiLit = distanceToPlacement - 1

            # pickupDistanceX = distanceBeforePiLit * math.cos(angleToPlacement)
            # pickupDistanceY = distanceBeforePiLit * math.sin(angleToPlacement)

            # convertedPoint = [pickupDistanceX, pickupDistanceY
            print(target)
            angle = self.interface.PP_client_goal([target])
            self.interface.pickup_client_goal(intakeSide, angle)

            print("finished pickup")

            self.interface.loadPointToSQL("retrieve", intakeSide)

            if(intakeSide == "switch_right"):
                intakeSide = "switch_left"
            else:
                intakeSide = "switch_right"
