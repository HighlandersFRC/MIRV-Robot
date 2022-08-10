#!/usr/bin/env python3

import math
from RoverInterface import RoverInterface
import rospy
import mirv_control.msg
import actionlib
from RoverInterface import RoverInterface
import threading
from PiLitController import PiLitControl as PiLitController
from std_msgs.msg import String, Float64MultiArray
import time
import numpy as np
from RoverInterface import RoverInterface

from sensor_msgs.msg import NavSatFix

class roverMacros():
    def __init__(self, Interface: RoverInterface):
        self.interface = Interface

    def undock(self):
        self.interface.deployGarage()
        self.interface.drive(-0.2, 6)
        self.interface.turn(math.pi, 4)

    def placePiLitFromSide(self, timeout, intakeSide):
        start_time = time.time()
        self.interface.intake_command_pub.publish(String(intakeSide))
        self.interface.intake_command_pub.publish(String("deposit"))
        while not self.interface.limit_switches[3] and not self.interface.limit_switches[2]:
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
        self.placePiLitFromSide(6, side)
        time.sleep(2)
        self.interface.intake_command_pub.publish(String("reset"))
        self.interface.loadPointToSQL("deploy", side)

    def placeAllPiLits(self, points):
        intakeSide = "switch_right"
        for point in points:
            target = [point]
            self.interface.PP_client_goal(target)
            self.placePiLitFromSide(6, intakeSide)
            self.interface.intake_command_pub.publish(String("reset"))

            self.interface.loadPointToSQL("deploy", intakeSide)

            if(intakeSide == "switch_right"):
                intakeSide = "switch_left"
            else:
                intakeSide = "switch_right"

            time.sleep(2)

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
        d = ((currentPoint[0]-finalPoint[0][0])**2 +(currentPoint[1]-finalPoint[0][1])**2)**0.5
        UV = np.array([(currentPoint[0]-finalPoint[0][0])/d , (currentPoint[1]-finalPoint[0][1])/d])
        d2 = d
        if(d >1):
            d2 = d-1
        TP = currentPoint - UV*d2
        return TP

    def pickupPiLit(self):
        stored = self.interface.getPiLitsStored()
        if not stored or len(stored) != 2:
            rospy.logerr("Invalid number of stored Pi-Lits")
            return
        if stored[0] < stored[1]:
            side = "switch_right"
        else:
            side = "switch_left"
        self.interface.pickup_client_goal(side, 0.1)
        self.interface.loadPointToSQL("retrieve", side)

    def pickupAllPiLits(self, lists, reverse):
        intakeSide = "switch_right"
        # points = [[lists.latitude[i], lists.longitude[i]] for i in range(len(lists.latitude))]
        if(reverse):
            lists.reverse()
        for point in lists:
            print(f"POINT {point}")
            convertedPoint = [self.interface.CoordConversion_client_goal(point)]
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

            
