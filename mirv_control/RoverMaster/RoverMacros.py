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

    # Initiate docking procedure. Requires Rover to be more than 5 meters from the garage
    def dock(self):

        # Step 0: Enable garage detection
        self.interface.changeNeuralNetworkSelected("aruco")
        time.sleep(5)

        # Step 1: Deploy garage
        self.interface.deployGarage()

        # # Step 2: Drive to front of garage
        # print(f"Step 2: Driving to front of garage")
        # lineUpPoint1 = [3, 0]
        # lineUpPoint2 = [1, 0]
        # lineUpPoints = [lineUpPoint1, lineUpPoint2]
        # self.interface.PP_client_goal(lineUpPoints)

        # ## TODO: Cancel
        # if self.interface.cancelled:
        #     return

        # Step 3: Turn towards tag
        # TODO: Substep: turn roughly towards garage?? Just in case it is not in view
        time.sleep(5)
        garage_angle = self.interface.garageLocation.angle_to_garage
        print(f"Step 3: Turning to relative angle of {garage_angle} degrees")
        if not garage_angle:
            rospy.logerr("Garage was not found in frame - Step 2")
            self.interface.changeNeuralNetworkSelected("none")
            return
        self.interface.pointTurn(garage_angle, 5)

        # Step 4: itentify offsets to aruco tag
        time.sleep(5)
        x = self.interface.garageLocation.rover_position_x_from_garage
        y = self.interface.garageLocation.rover_position_y_from_garage
        if not x or not y:
            rospy.logerr("Garage was not found in frame - Step 3")
            self.interface.changeNeuralNetworkSelected("none")
            return

        theta_1 = (180 / math.pi) * math.atan2(y, x)
        theta_2 = (90 - abs(theta_1))*theta_1/abs(theta_1)
        print(f"Angles: {theta_1}, {theta_2}")
        angle_to_perpendicular_degrees = theta_2
        distance_meters = abs(y)
        print(f"Distance: {distance_meters}")

        # Step 5: Turn towards perpendicular vector of garage
        time.sleep(5)
        print(
            f"Step 5: Turning to relative angle of {angle_to_perpendicular_degrees} degrees")
        self.interface.pointTurn(angle_to_perpendicular_degrees, 5)

        # Step 6: Drive to directly in front of garage
        time.sleep(5)
        print(f"Step 6: Moving distance of {distance_meters} meters")
        self.interface.driveDistance(distance_meters, 0.25, 0.1)

        # Step 7: Turn to face garage
        time.sleep(5)
        print(f"Step 7: Turning to relative angle of {90} degrees")
        self.interface.pointTurn(-90*theta_1/abs(theta_1), 5)

        # Step 8: Turn towards tag
        time.sleep(5)
        garage_angle = self.interface.garageLocation.angle_to_garage
        print(f"Step 8: Turning to relative angle of {garage_angle} degrees")
        if not garage_angle:
            rospy.logerr("Garage was not found in frame - Step 8")
            self.interface.changeNeuralNetworkSelected("none")
            return
        self.interface.pointTurn(garage_angle, 5)

        # Step 9: Drive directly towards back of garage
        print(f"Driving Into Garage")
        self.interface.drive_into_garage(0)

        # Step 10: Retract garage
        print(f"Retracting Garage")
        self.interface.retractGarage()

        # # Step 10: Disable camera neural network
        # self.interface.changeNeuralNetworkSelected("none")

    def dockNoPathing(self):
        self.interface.changeNeuralNetworkSelected("aruco")
        self.interface.deployGarage()
        self.interface.drive_into_garage(0)
        self.interface.changeNeuralNetworkSelected("none")

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

        result = self.interface.pickup_client_goal(side, 0)
        if result.finished:
            self.interface.loadPointToSQL("retrieve", side)
        self.interface.changeNeuralNetworkSelected("none")

    def testPointTurn(self):
        self.interface.pointTurn(180, 5)

    def testDriveDistance(self):
        self.interface.driveDistance(1.0, 0.25, 0.1)

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

            currentLocationConverted = self.interface.CoordConversion_client_goal(
                [self.interface.getCurrentLatitude(), self.interface.getCurrentLongitude()])

            print("Current Point:", currentLocationConverted)
            currentX = currentLocationConverted[0]
            currentY = currentLocationConverted[1]

            deltaX = placementX - currentX
            deltaY = -(placementY - currentY)

            absAngleToPlacement = math.atan2(deltaY, deltaX)
            angleToPlacement = absAngleToPlacement + self.interface.heading
            distanceToPlacement = math.sqrt(deltaX**2 + deltaY**2)


            # Start By Turning in the general direction of the target
            self.interface.pointTurn(math.degrees(angleToPlacement)%360, 5)
            
            if distanceToPlacement > 3:
                distanceBeforePiLit = distanceToPlacement - 1
                helperDistance = distanceBeforePiLit - 1

                targetX = distanceBeforePiLit * \
                    math.cos(-absAngleToPlacement) + currentX
                targetY = distanceBeforePiLit * \
                    math.sin(-absAngleToPlacement) + currentY

                helperPointX = helperDistance * \
                    math.cos(-absAngleToPlacement) + currentX
                helperPointY = helperDistance * \
                    math.sin(-absAngleToPlacement) + currentY


                print("Rover Location:", currentX, currentY, "PiLit Location", placementX, placementY, "Target Location", targetX, targetY, "Helper Location", helperPointX, helperPointY)
                targetPoint = [targetX, targetY]
                helperPoint = [helperPointX, helperPointY]


                print("Pickup Path:", [helperPoint, targetPoint])
                self.interface.PP_client_goal([helperPoint, targetPoint])
            
            
            elif distanceToPlacement > 2:
                self.interface.driveDistance(distanceToPlacement -1.5, 0.25, 0.1)
            
            time.sleep(2)
            currentLocationConverted = self.interface.CoordConversion_client_goal(
                [self.interface.getCurrentLatitude(), self.interface.getCurrentLongitude()])


            print("Current Point:", currentLocationConverted)
            currentX = currentLocationConverted[0]
            currentY = currentLocationConverted[1]

            deltaX = placementX - currentX
            deltaY = -(placementY - currentY)

            angleToPlacement = math.atan2(deltaY, deltaX) + self.interface.heading
            distanceToPlacement = math.sqrt(deltaX**2 + deltaY**2)
            self.interface.pointTurn(math.degrees(angleToPlacement)%360, 5)


            stored = self.interface.getPiLitsStored()
            if not stored or len(stored) != 2:
                rospy.logerr("Invalid number of stored Pi-Lits")
                return
            if stored[0] < stored[1]:
                side = "switch_right"
            else:
                side = "switch_left"

            result = self.interface.pickup_client_goal(side, 0)
            if result.finished:
                print("Recovery Successful")
                self.interface.loadPointToSQL("retrieve", side)
            time.sleep(2)

        self.interface.changeNeuralNetworkSelected("none")
        
