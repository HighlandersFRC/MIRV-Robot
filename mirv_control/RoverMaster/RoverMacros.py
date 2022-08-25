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
        lineUpPoint2 = [3.5,0]
        lineUpPoint3 = [1, 0]
        lineUpPoints = [lineUpPoint1, lineUpPoint2, lineUpPoint3]

        if self.interface.cancelled:
            return
        self.interface.deployGarage()
        if self.interface.cancelled:
            return
        self.interface.PP_client_goal(lineUpPoints)
        if self.interface.cancelled:
            return
        self.interface.garage_client_goal(0)
        if self.interface.cancelled:
            return
        self.interface.retractGarage()

    def dockNoPathing(self):
        self.interface.changeNeuralNetworkSelected("aruco")
        self.interface.deployGarage()
        self.interface.garage_client_goal(0)

    def placePiLitFromSide(self, timeout, intakeSide):
        start_time = time.time()
        if self.interface.cancelled:
            return False
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
        if self.interface.cancelled:
            return
        self.placePiLitFromSide(7, side)
        if self.interface.cancelled:
            return
        self.interface.intake_command_pub.publish(String("reset"))
        self.interface.loadPointToSQL("deploy", side)

    def placeAllPiLits(self, firstPoint, roughHeading, formation_type):
        self.interface.changeNeuralNetworkSelected("lanes")
        firstPointTruckCoord = self.interface.CoordConversion_client_goal(firstPoint)
        startingTarget = [firstPointTruckCoord]
        self.interface.PP_client_goal(startingTarget)
        detected_lanes = {'right': (firstPoint[0], firstPoint[1])}
        points = placement.generate_pi_lit_formation(detected_lanes, roughHeading, 3, formation_type)
        print("Calculated Placement Points: ", points)

        self.interface.changeNeuralNetworkSelected("none")
        intakeSide = "switch_right"

        for pnt  in points:
            point = self.interface.CoordConversion_client_goal(pnt)
            print("Converting Point")
            target = [point]
            if self.interface.cancelled:
                return
            self.interface.PP_client_goal(target)
            print("Going to PP target")
            if self.interface.cancelled:
                return
            self.placePiLitFromSide(6, intakeSide)
            if self.interface.cancelled:
                return
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
        d = ((currentPoint[0]-finalPoint[0][0])**2 +(currentPoint[1]-finalPoint[0][1])**2)**0.5
        UV = np.array([(currentPoint[0]-finalPoint[0][0])/d , (currentPoint[1]-finalPoint[0][1])/d])
        d2 = d
        if(d >1):
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

        #if self.interface.cancelled:
        #    return
        self.interface.pickup_client_goal(side, 0)
        #if self.interface.cancelled:
        #    return
        self.interface.loadPointToSQL("retrieve", side)
        self.interface.changeNeuralNetworkSelected("none")

    def pickupAllPiLits(self, reverse):
        self.interface.changeNeuralNetworkSelected("piLit")
        intakeSide = "switch_right"
        
        # if(reverse):
        #     lists.reverse()

        # points = [[lists.latitude[i], lists.longitude[i]] for i in range(len(lists.latitude))]

        points = self.interface.getLatestSqlPoints()
        print("Retrieving Pi-lits at: ", points)

        if(reverse):
             points.reverse()

        for point in points:
            if self.interface.cancelled:
                return
            print(f"POINT {point}")
            convertedPoint = self.interface.CoordConversion_client_goal(point)
            
            print("Converted Point:", convertedPoint)
            placementX = convertedPoint[0]
            placementY = convertedPoint[1]

            currentLocationConverted = self.interface.CoordConversion_client_goal([self.interface.getCurrentLatitude(), self.interface.getCurrentLongitude()])

            currentX = currentLocationConverted[0]
            currentY = currentLocationConverted[1]

            theta = self.interface.getCameraIMU()

            xTBody = placementX*math.cos(theta) + placementY*math.sin(theta) - currentY*math.sin(theta) - currentX*math.cos(theta)
            yTBody = -placementX*math.sin(theta) + placementY*math.cos(theta) - currentY*math.cos(theta) + currentX*math.sin(theta)

            print(xTBody, yTBody)

            distanceToPlacement = round(math.sqrt(math.pow(xTBody, 2) + math.pow(yTBody, 2)), 3)

            angleToPlacement = math.atan2(yTBody, xTBody) - theta

            # distanceToPlacement = math.sqrt((math.pow(placementX - currentX, 2)) + (math.pow(placementY - currentY, 2)))
            
            if distanceToPlacement > 3:
                print(f"Rover is {distanceToPlacement} m from pilit. Performing Path Pickup.")
                distanceBeforePiLit = distanceToPlacement - 1
                helperDistance = distanceBeforePiLit - 1

                targetX = distanceBeforePiLit * math.cos(angleToPlacement) + currentX
                targetY = distanceBeforePiLit * math.sin(angleToPlacement) + currentY

                helperPointX = helperDistance * math.cos(angleToPlacement) + currentX
                helperPointY = helperDistance * math.sin(angleToPlacement) + currentY

                print("Rover Location:", currentX, currentY, "PiLit Location", placementX, placementY, "Target Location", targetX, targetY, "Helper Location", helperPointX, helperPointY)

                targetPoint = [targetX, targetY]
                helperPoint = [helperPointX, helperPointY]

                print("Pickup Path:", [helperPoint, targetPoint])
                angle = self.interface.PP_client_goal([helperPoint, targetPoint])
                print("Pathing Complete", angle)
                #if self.interface.cancelled:
                #    return
                print("Running Pickup")
                self.interface.pickup_client_goal(intakeSide, angle)
            else:
                print("Rover is close to Pi-lit performing basic pickup")
                #angle = self.interface.PP_client_goal([currentLocationConverted])
                print("Turning Towards: ", math.degrees(angleToPlacement))

                self.interface.pickup_client_goal(intakeSide, math.degrees(angleToPlacement) % 360)
            

            print("Rover Location", self.interface.CoordConversion_client_goal([self.interface.getCurrentLatitude(), self.interface.getCurrentLongitude()])
)
            print("finished pickup")
            

            self.interface.loadPointToSQL("retrieve", intakeSide)

            if(intakeSide == "switch_right"):
                intakeSide = "switch_left"
            else:
                intakeSide = "switch_right"

            
