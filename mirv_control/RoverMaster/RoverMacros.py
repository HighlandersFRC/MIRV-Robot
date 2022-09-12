#!/usr/bin/env python3

import math
import rospy
import mirv_control.msg
import actionlib
import threading
from std_msgs.msg import String, Float64MultiArray
from mirv_control.msg import single_lane_detection
from mirv_control.msg import DetectLanesResult
import time
import numpy as np
import placement
from RoverInterface import RoverInterface
from sensor_msgs.msg import NavSatFix


def cancellable(func):
    def method(self, *args, **kwargs):
        if not self.interface.cancelled:
            return func(self, *args, **kwargs)
    return method


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
        self.interface.wait(0.5)

        # Step 1: Deploy garage
        self.interface.deployGarage()

        # # Step 2: Drive to front of garage
        # print(f"Step 2: Driving to front of garage")
        lineUpPoint1 = [3.5, 0]
        lineUpPoint2 = [2, 0]
        lineUpPoints = [lineUpPoint1, lineUpPoint2]
        self.interface.PP_client_goal(lineUpPoints)

        self.interface.wait(0.5)
        currentLocationConverted = self.interface.CoordConversion_client_goal(
            [self.interface.getCurrentLatitude(), self.interface.getCurrentLongitude()])
        print("Current Location", currentLocationConverted)

        absAngleToPlacement = math.atan2(
            currentLocationConverted[1], -currentLocationConverted[0])
        angleToPlacement = absAngleToPlacement + self.interface.heading
        
        print(f"Turning: {angleToPlacement} to face garage")
        self.interface.pointTurn(math.degrees(angleToPlacement) % 360, 5)

        # ## TODO: Cancel
        # if self.interface.cancelled:
        #     return

        # Step 3: Turn towards tag
        # TODO: Substep: turn roughly towards garage?? Just in case it is not in view
        self.interface.wait(5)
        if self.interface.garageLocation is not None:
            garage_angle = self.interface.garageLocation.angle_to_garage
        else:
            rospy.logerr("Unable to Dock. No Garage Location Present")
            return False
        print(f"Step 3: Turning to relative angle of {garage_angle} degrees")
        if not garage_angle:
            rospy.logerr("Garage was not found in frame - Step 2")
            self.interface.changeNeuralNetworkSelected("none")
            return False
        self.interface.pointTurn(garage_angle, 5)

        # Step 4: itentify offsets to aruco tag
        self.interface.wait(5)
        x = self.interface.garageLocation.rover_position_x_from_garage
        y = self.interface.garageLocation.rover_position_y_from_garage
        if not x or not y:
            rospy.logerr("Garage was not found in frame - Step 3")
            self.interface.changeNeuralNetworkSelected("none")
            return False

        theta_1 = (180 / math.pi) * math.atan2(y, x)
        theta_2 = (90 - abs(theta_1))*theta_1/abs(theta_1)
        print(f"Angles: {theta_1}, {theta_2}")
        angle_to_perpendicular_degrees = theta_2
        distance_meters = abs(y)
        print(f"Distance: {distance_meters}")

        # Step 5: Turn towards perpendicular vector of garage
        self.interface.wait(0.5)
        print(
            f"Step 5: Turning to relative angle of {angle_to_perpendicular_degrees} degrees")
        self.interface.pointTurn(angle_to_perpendicular_degrees, 5)

        # Step 6: Drive to directly in front of garage
        self.interface.wait(0.5)
        print(f"Step 6: Moving distance of {distance_meters} meters")
        self.interface.driveDistance(distance_meters, 0.25, 0.1)

        # Step 7: Turn to face garage
        self.interface.wait(0.5)
        print(f"Step 7: Turning to relative angle of {90} degrees")
        self.interface.pointTurn(-90*theta_1/abs(theta_1), 5)

        # Step 8: Turn towards tag
        self.interface.wait(0.5)
        garage_angle = self.interface.garageLocation.angle_to_garage
        print(f"Step 8: Turning to relative angle of {garage_angle} degrees")
        if not garage_angle:
            rospy.logerr("Garage was not found in frame - Step 8")
            self.interface.changeNeuralNetworkSelected("none")
            return False
        self.interface.pointTurn(garage_angle, 5)

        # Step 9: Drive directly towards back of garage
        print(f"Driving Into Garage")
        self.interface.drive_into_garage(0)

        docked = self.interface.wait_for_docked()
        if not docked:
            return False
        
        
        # Step 10: Retract garage
        print(f"Retracting Garage")
        self.interface.retractGarage()

        # # Step 10: Disable camera neural network
        self.interface.changeNeuralNetworkSelected("none")
        return True

    def dockNoPathing(self):
        self.interface.changeNeuralNetworkSelected("aruco")
        self.interface.deployGarage()
        self.interface.drive_into_garage(0)
        self.interface.changeNeuralNetworkSelected("none")

    @cancellable
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
        self.interface.wait(1)
        self.interface.intake_command_pub.publish(String("reset"))
        print("Stopping Drive")
        self.interface.drive(0, 0.5)
        self.interface.wait(0.5)
        self.interface.drive(0.2, 2)
        self.interface.loadPointToSQL("deploy", intakeSide)
        return True

    @cancellable
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

    def get_angle_error(self, a1, a2):
        heading_error = (a1 - a2) % 360
        if heading_error > 180:
            heading_error -= 360
        return heading_error

    @cancellable
    def placeAllPiLits(self, firstPoint, roughHeading, formation_type):
        self.interface.changeNeuralNetworkSelected("lanes")

        firstPointTruckCoord = self.interface.CoordConversion_client_goal(
            firstPoint)
        startingTarget = [firstPointTruckCoord]
        print("Starting Target", startingTarget)
        self.interface.PP_client_goal(startingTarget)

        self.interface.wait(5)
        print("Global Heading", math.degrees(
            self.interface.globalHeading), "Rough Heading", roughHeading)
        self.interface.pointTurn(
            roughHeading - (math.degrees(self.interface.globalHeading)) % 360, 5)

        self.interface.wait(5)
        print("Updated Global Heading", math.degrees(
            self.interface.globalHeading))
        print("Local Heading", self.interface.heading)

        # If 3 pi-lits, send to other method. This one is long, so needs a different algorithm
        if formation_type in ['taper_right_3', 'taper_left_3']:
            self.long_pilit_placement(formation_type)
            self.interface.changeNeuralNetworkSelected("none")
            return

        ##########################
        # Detect Lane Lines
        ##########################
        lane_detections = self.laneLineSearchSequence()
        if not lane_detections:
            print(f"NO LANES DETECTED, RETURNING 224, {lane_detections}")
            self.interface.changeNeuralNetworkSelected("none")
            return

        heading_error = self.get_angle_error(math.degrees(
            self.interface.heading), lane_detections.net_heading)
        if abs(heading_error) > 10:
            print("Heading of {heading_error} is off by more than threshold")
            self.interface.pointTurn(heading_error, 5)
            self.interface.wait(5)
            new_lane_detections = self.interface.Lane_Lines_goal()
            if new_lane_detections and len(new_lane_detections.lane_detections) != 0:
                lane_detections = new_lane_detections
                heading_error = self.get_angle_error(math.degrees(
                    self.interface.heading), lane_detections.net_heading)

        if len(lane_detections.lane_detections) == 0:
            print(f"NO LANES DETECTED, RETURNING 242, {lane_detections}")
            self.interface.changeNeuralNetworkSelected("none")
            return

        print(f"LANES DETECTED: {lane_detections.lane_detections}")
        print(f"LANE HEADING: {lane_detections.net_heading}")
        print(f"LANE WIDTH: {lane_detections.width}")

        ##########################
        # Mock Lane Line Detection
        ##########################
        # ## Logic to Create Fake Lane Lines
        # lane_detections = DetectLanesResult()
        # lane_detections.net_heading = float(math.degrees(self.interface.heading))
        # lane_detections.width = float(3)

        # detections = []
        # lane = single_lane_detection()
        # lane.heading = float(math.degrees(self.interface.heading))
        # end_point = placement.getEndPoint(
        #     self.interface.xPos + 1.5 * math.cos(self.interface.heading) + 3 * math.sin(self.interface.heading), self.interface.yPos + 1.5 * math.sin(self.interface.heading) + 3 * math.cos(self.interface.heading), lane.heading - 90, lane_detections.width/2)
        # lane.start_x = float(end_point[0])
        # lane.start_y = float(end_point[1])
        # lane.end_x = float(0)
        # lane.end_y = float(0)
        # lane.lane_type = 'left'
        # detections.append(lane)
        # lane_detections.lane_detections = detections

        points = placement.generate_pi_lit_formation(
            lane_detections.lane_detections, lane_detections.net_heading, lane_detections.width, formation_type)

        print("Calculated Placement Points: ", points)

        self.interface.changeNeuralNetworkSelected("none")

        # return self.placeMultiplePiLits(points)

    @cancellable
    def placeMultiplePiLits(self, points):
        for pnt in points:
            point = [pnt[0], pnt[1]]
            target = [point]
            if self.interface.cancelled:
                return

            currentLocationConverted = self.interface.CoordConversion_client_goal(
                [self.interface.getCurrentLatitude(), self.interface.getCurrentLongitude()])
            # print("Current Location", currentLocationConverted)
            deltaX = pnt[0] - currentLocationConverted[0]
            deltaY = -(pnt[1] - currentLocationConverted[1])

            absAngleToPlacement = math.atan2(deltaY, deltaX)
            angleToPlacement = absAngleToPlacement + self.interface.heading
            self.interface.pointTurn(math.degrees(angleToPlacement) % 360, 5)

            # print("Target", target)
            self.interface.PP_client_goal(target)
            # print("Going to PP target")

            self.placePiLit()
            print("Placed Pi-Lit:", self.interface.xPos, self.interface.yPos)
            self.interface.wait(2)
        if not self.interface.cancelled:
            self.interface.setPiLits("simultaneous")
        return True

    @cancellable
    def laneLineSearchSequence(self):
        detections = []

        initial_angle = math.degrees(self.interface.heading)
        positions = [
            0, 25, 50, -25, -50
        ]

        for target_angle in positions:
            self.interface.wait(5)
            curr_angle = math.degrees(self.interface.heading)
            print('search angles: ', initial_angle, curr_angle, target_angle)
            print(initial_angle - curr_angle + target_angle)
            self.interface.pointTurn(
                (-(initial_angle - curr_angle) + target_angle), 5)
            lane_detections = self.interface.Lane_Lines_goal()
            # 79, 53, 336, 258

            # could use this to return all
            detections.append((curr_angle, lane_detections))
            
            if lane_detections and len(lane_detections.lane_detections) != 0:
                print("RETURNING LANE DETECTIONS")
                return lane_detections

        return None

    @cancellable
    def long_pilit_placement(self, formation_type):
        # Step 1: identify current lane location and heading
        initial_lane_detection = self.laneLineSearchSequence()
        if not initial_lane_detection:
            print("NO LANES DETECTED INITIAL, RETURNING")
            return

        # Step 2: Place 1 pi-lit in correct position
        start_formation = formation_type + '_start'

        starting_points = placement.generate_pi_lit_formation(
            initial_lane_detection.lane_detections, initial_lane_detection.net_heading, initial_lane_detection.width, start_formation)

        self.placeMultiplePiLits(starting_points)

        # Step 3: Drive forward forwards 20 feet along heading
        destination = placement.getEndPoint(
            self.interface.xPos, self.interface.yPos, initial_lane_detection.net_heading, 20 * placement.FEET_TO_METERS)
        laneDestination = self.interface.CoordConversion_client_goal(
            destination)
        self.interface.PP_client_goal([laneDestination])

        # Step 4: Identify lanes again
        final_lane_detection = self.laneLineSearchSequence()
        if not final_lane_detection:
            print("NO LANES DETECTED FINAL, RETURNING")
            return

        # Step 5: Determine very accurate lane heading
        initial_center = placement.get_center_coordinates(
            initial_lane_detection.lane_detections, initial_lane_detection.net_heading, initial_lane_detection.width)
        final_center = placement.get_center_coordinates(
            final_lane_detection.lane_detections, final_lane_detection.net_heading, final_lane_detection.width)
        net_heading = placement.get_angle_robot_frame(
            initial_center, final_center)

        # Step 6: Place all Pi-Lits
        end_formation = formation_type + '_end'

        ending_points = placement.generate_pi_lit_formation(
            final_lane_detection.lane_detections, net_heading, final_lane_detection.width, end_formation)

        self.placeMultiplePiLits(ending_points)

    def placeAllPiLitsNoMovement(self, count):
        intakeSide = "switch_right"
        for i in range(0, count):
            self.placePiLitFromSide(4, intakeSide)
            self.interface.intake_command_pub.publish(String("reset"))
            if(intakeSide == "switch_right"):
                intakeSide = "switch_left"
            else:
                intakeSide = "switch_right"
            self.interface.wait(4)

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

    @cancellable
    def pickupPiLit(self):
        self.interface.changeNeuralNetworkSelected("piLit")
        stored = self.interface.getPiLitsStored()
        print("Stored Pi-lits", stored)
        if not stored or len(stored) != 2:
            rospy.logerr("Invalid number of stored Pi-Lits")
            return
        if stored[0] < stored[1]:
            side = "switch_right"
        else:
            side = "switch_left"

        result = self.interface.pickup_client_goal(side, 0)
        print(result)
        print(result.finished)
        if result is not None and result.finished:
            self.interface.loadPointToSQL("retrieve", side)
        else:
            print("Pickup Failed")
        self.interface.changeNeuralNetworkSelected("none")

    def testPointTurn(self):
        self.interface.pointTurn(180, 5)

    def testDriveDistance(self):
        self.interface.driveDistance(1.0, 0.25, 0.1)

    def pickupAllPiLits(self, reverse):
        self.interface.changeNeuralNetworkSelected("piLit")
        self.interface.setPiLits("idle")
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
            if distanceToPlacement > 1:
                self.interface.pointTurn(
                    math.degrees(angleToPlacement) % 360, 5)

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

                print("Rover Location:", currentX, currentY, "PiLit Location", placementX, placementY,
                      "Target Location", targetX, targetY, "Helper Location", helperPointX, helperPointY)
                targetPoint = [targetX, targetY]
                helperPoint = [helperPointX, helperPointY]

                print("Pickup Path:", [helperPoint, targetPoint])
                self.interface.PP_client_goal([helperPoint, targetPoint])

            elif distanceToPlacement > 2:
                self.interface.driveDistance(
                    distanceToPlacement - 1.5, 0.25, 0.1)

            elif distanceToPlacement < 1:
                self.interface.driveDistance(-1, 0.25, 0.1)

            self.interface.wait(2)
            currentLocationConverted = self.interface.CoordConversion_client_goal(
                [self.interface.getCurrentLatitude(), self.interface.getCurrentLongitude()])

            print("Current Point:", currentLocationConverted)
            currentX = currentLocationConverted[0]
            currentY = currentLocationConverted[1]

            deltaX = placementX - currentX
            deltaY = -(placementY - currentY)

            angleToPlacement = math.atan2(
                deltaY, deltaX) + self.interface.heading
            distanceToPlacement = math.sqrt(deltaX**2 + deltaY**2)
            self.interface.pointTurn(math.degrees(angleToPlacement) % 360, 5)

            self.pickupPiLit()

        self.interface.changeNeuralNetworkSelected("none")
