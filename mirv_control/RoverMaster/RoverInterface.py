#!/usr/bin/env python3
from __future__ import print_function
import queue
import rospy
from PiLitController import PiLitControl
# Brings in the SimpleActionClient
import actionlib
import time
from std_msgs.msg import Float64, Float64MultiArray, String
from mirv_control.msg import garage_state_msg
from mirv_control.msg import garage_position as GaragePosition
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import os
import sys
import pickle
import helpful_functions_lib as conversion
import math
import json
from threading import Thread
import multiprocessing
import mirv_control.msg

from mirv_control.msg import pilit_db_msg

from sensor_msgs.msg import NavSatFix


class RoverInterface():
    RoverMacro = None
    stateMsg = String
    TELEOP_DRIVE = "remote_operation"
    TELEOP_DRIVE_AUTONOMOUS = "remote_operation_autonomous"
    DISCONNECTED = "disconnected"
    CONNECTED_DISABLED = "disabled"
    CONNECTED_ENABLED = "idle"
    AUTONOMOUS = "autonomous"
    E_STOP = "e_stop"
    DOCKED = "docked"

    def __init__(self):

        self.lastmsg = None
        self.roverState = self.DISCONNECTED
        print("setting up server connections")
        self.calibrationClient = actionlib.SimpleActionClient(
            'StartingHeading', mirv_control.msg.IMUCalibrationAction)
        self.calibrationClient.wait_for_server()
        print("connected to starting heading AS")
        self.PPclient = actionlib.SimpleActionClient(
            'PurePursuitAS', mirv_control.msg.PurePursuitAction)
        self.PPclient.wait_for_server()
        print("connected to Pure Pursuit Server")
        self.pickupClient = actionlib.SimpleActionClient(
            "PickupAS", mirv_control.msg.MovementToPiLitAction)
        self.pickupClient.wait_for_server()
        print("connected to PiLit pickup Server")
        self.TruckCordClient = actionlib.SimpleActionClient(
            'NavSatToTruckAS', mirv_control.msg.NavSatToTruckAction)
        self.TruckCordClient.wait_for_server()
        print("connected to Pure Truck CordinateAS")
        self.databaseClient = actionlib.SimpleActionClient(
            "Database", mirv_control.msg.DatabaseAction)
        self.databaseClient.wait_for_server()
        print("connected to Database Query Server")
        self.garageClient = actionlib.SimpleActionClient(
            "Docking", mirv_control.msg.GarageAction)
        self.garageClient.wait_for_server()
        print("connected to Garage Server")
        self.TeleopClient = actionlib.SimpleActionClient(
            "TeleopDrive", mirv_control.msg.generalAction)
        self.TeleopClient.wait_for_server()
        print("connected to teleop drive")
        self.pointTurnClient = actionlib.SimpleActionClient(
            "PointTurnRelativeAS", mirv_control.msg.PointTurnAction)
        self.pointTurnClient.wait_for_server()
        print("connected to PointTurn Server")
        self.driveDistanceClient = actionlib.SimpleActionClient(
            "DriveDistanceAS", mirv_control.msg.DriveDistanceAction)
        self.driveDistanceClient.wait_for_server()
        print("connected to driveDistance Server")
        # self.PlacementGeneratorClient = actionlib.SimpleActionClient("PlacementLocationGenerator", mirv_control.msg.GeneratePlacementLocationsAction)
        # self.PlacementGeneratorClient.wait_for_server()
        # print("connected to placement generator")

        self.pilit_controller = PiLitControl()

        self.isJoystickControl = True
        self.isPurePursuitControl = False
        self.purePursuitTarget = []
        self.isPickupControl = False
        self.latitude = 0
        self.longitude = 0
        self.altitude = 0
        self.xPos = 0
        self.yPos = 0
        self.heading = 0
        self.placementPoints = []
        self.storedPiLits = [4, 4]
        self.garage_state = "invalid"
        self.limit_switches = [1, 1, 0, 0]
        self.heartBeatTime = 0
        self.tasks = []
        self.garageLocation = None
        self.startingHeading = 0
        self.globalHeading = 0
        self.cancelled = False

        self.imu_buffer = []
        self.imu = 0

        # SUBSCRIBERS
        self.gpsOdomSub = rospy.Subscriber(
            "gps/fix", NavSatFix, self.updateOdometry)
        self.truckOdomSub = rospy.Subscriber(
            "/EKF/Odometry", Odometry, self.updateTruckOdom)
        self.placementLocationSub = rospy.Subscriber(
            "placementLocation", Float64MultiArray, self.updatePlacementPoints)
        self.garage_sub = rospy.Subscriber(
            "GarageStatus", garage_state_msg, self.garage_state_callback)
        self.intake_limit_switch_sub = rospy.Subscriber(
            "intake/limitswitches", Float64MultiArray, self.limit_switch_callback)
        self.cloud_sub = rospy.Subscriber(
            "CloudCommands", String, self.cloud_callback)
        self.startHeadingSub = rospy.Subscriber(
            "Start/Heading", Float64, self.setStartingHeading)
        self.garage_location_sub = rospy.Subscriber(
            "GaragePosition", GaragePosition, self.garage_location_callback)

        # PUBLISHERS
        self.sqlPub = rospy.Publisher(
            "pilit/events", pilit_db_msg, queue_size=5)
        self.simpleDrivePub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.intake_command_pub = rospy.Publisher(
            "intake/command", String, queue_size=5)
        self.garage_pub = rospy.Publisher(
            "GarageCommands", String, queue_size=5)
        self.statePublisher = rospy.Publisher(
            "RoverState", String, queue_size=5)
        self.neuralNetworkSelector = rospy.Publisher(
            "neuralNetworkSelector", String, queue_size=1)
        # self.placementSub = rospy.Subscriber('pathingPointInput', Float64MultiArray, self.updatePlacementPoints)

    # def setPiLitSequence(self, is_wave: bool):
    #     self.pilit_controller.patternType(is_wave)
    #     self.pilit_controller.reset()

    # def setPiLitSequenceReversed(self, reversed: bool):
    #     self.pilit_controller.reversePattern(reversed)
    #     self.pilit_controller.reset()

    def updateIMU(self, data):
        self.imu_buffer.append(data.data)
        if len(self.imu_buffer) > 10:
            self.imu_buffer.pop(0)

        avg = 0
        for val in self.imu_buffer:
            avg += val

        self.imu = avg / len(self.imu_buffer)

    def getCameraIMU(self):
        return self.imu

    # options are piLit, lanes, piLitAndLanes, aruco, and none

    def changeNeuralNetworkSelected(self, selectedNetwork):
        self.neuralNetworkSelector.publish(selectedNetwork)

    def loadRoverMacro(self, macro):
        self.RoverMacro = macro
        rospy.loginfo("Rover Macros loaded into Rover Interface")

    def garage_location_callback(self, msg):
        self.garageLocation = msg

    def garage_state_callback(self, msg):
        self.garage_state = msg.state

    def limit_switch_callback(self, switches):
        self.limit_switches = switches.data

    def stopIntakeAndMagazine(self):
        self.intake_command_pub.publish(String("disable"))

    def intakeDown(self):
        self.intake_command_pub.publish(String("down"))

    def intakeUp(self):
        self.intake_command_pub.publish(String("reset"))

    def magazineIn(self):
        self.intake_command_pub.publish(String("mag_in"))

    def magazineOut(self):
        self.intake_command_pub.publish(String("mag_out"))

    def deployGarage(self):
        self.garage_pub.publish(String("deploy"))
        while self.garage_state != "deployed":
            time.sleep(0.1)

    def retractGarage(self):
        self.garage_pub.publish(String("retract"))
        while self.garage_state != "retracted_latched":
            time.sleep(0.1)

    def drive(self, vel, seconds):
        twist = Twist()
        twist.linear.x = vel
        startTime = time.time()
        self.simpleDrivePub.publish(twist)
        while time.time() - startTime < seconds:
            pass
        stop = Twist()
        stop.linear.x = 0
        self.simpleDrivePub.publish(stop)

    def turn(self, radians, seconds):
        if seconds == 0:
            return False
        twist = Twist()
        twist.angular.z = radians / seconds
        stop = Twist()
        stop.angular.z = 0
        startTime = time.time()
        self.simpleDrivePub.publish(twist)
        while time.time() - startTime < seconds - 1:
            pass
        self.simpleDrivePub.publish(stop)
        return True

    def updatePlacementPoints(self, data):
        self.placementPoints = self.convertOneDimArrayToTwoDim(list(data.data))
        self.placementPoints = self.convertPointsToTruckCoordinates(
            self.placementPoints)

    def getPlacementPoints(self):
        return self.placementPoints

    def convertOneDimArrayToTwoDim(self, points):
        pointList = []
        for i in range(int((len(points))/2)):
            tempX = points[0]
            points.pop(0)
            tempY = points[0]
            points.pop(0)
            pointList.append([tempX, tempY])
        return pointList

    def convertPointsToTruckCoordinates(self, points):
        for i in range(len(points)):
            points[i] = self.CoordConversion_client_goal(points[i])
            time.sleep(0.1)
        return points

    def getDriveClients(self):
        return self.calibrationClient, self.PPclient, self.pickupClient

    def updateOdometry(self, data):

        # Heading is in radians
        if hasattr(self, "globalHeading") and self.globalHeading != 0:
            #print("Global Heading", self.globalHeading)
            offset_meters = 0.3048
            de = offset_meters * math.cos(self.globalHeading)
            dn = offset_meters * math.sin(self.globalHeading)

            R = 6378137

            # Coordinate offsets in radians
            dLat = dn/R
            dLon = de/(R*math.cos(math.radians(data.latitude)))

            self.latitude = data.latitude + math.degrees(dLat)
            self.longitude = data.longitude + math.degrees(dLon)
        else:
            self.latitude = data.latitude
            self.longitude = data.longitude
        self.altitude = data.altitude

    def updateTruckOdom(self, data):
        self.xPos = data.pose.pose.position.x
        self.yPos = data.pose.pose.position.y
        self.heading = conversion.quat_from_pose2eul(
            data.pose.pose.orientation)[0]

        if self.startingHeading != 0:
            self.globalHeading = math.radians(math.degrees(
                self.startingHeading - math.pi/2 + self.heading) % 360)

    def getCurrentTruckOdom(self):
        return ([self.xPos, self.yPos])

    def getCurrentLatitude(self):
        return self.latitude

    def getCurrentLongitude(self):
        return self.longitude

    def getCurrentAltitude(self):
        return self.altitude

    def setStartingHeading(self, data):
        self.startingHeading = math.radians(data.data)
        self.startHeadSet = True

    def loadPointToSQL(self, action, intakeSide):
        msg = pilit_db_msg()
        msg.deploy_or_retrieve.data = action
        msg.side.data = intakeSide

        navSatFixMsg = NavSatFix()
        navSatFixMsg.latitude = self.latitude
        navSatFixMsg.longitude = self.longitude
        navSatFixMsg.altitude = self.altitude

        msg.gps_pos = navSatFixMsg
        self.sqlPub.publish(msg)

    def enableTeleopDrive(self, Halt):
        temp = ""
        mirv_control.msg.generalGoal.goal = temp
        goal = mirv_control.msg.generalGoal
        self.TeleopClient.send_goal(goal)
        if Halt:
            self.TeleopClient.wait_for_result()

    def disableTeleopDrive(self):
        self.TeleopClient.cancel_all_goals()

    def convertToOneD(self, TwoDArray):
        temp = []
        for i in range(len(TwoDArray)):
            try:
                temp.append(TwoDArray[i][0])
                temp.append(TwoDArray[i][1])
            except:
                raise Exception("Invalid points entered")
        return temp

    def getIsJoystickControl(self):
        return self.isJoystickControl

    def getIsPurePursuitControl(self):
        return self.isPurePursuitControl

    def getIsPickupControl(self):
        return self.isPickupControl

    def Calibrate_client_goal(self):
        mirv_control.msg.IMUCalibrationGoal.calibrate = True
        goal = mirv_control.msg.IMUCalibrationGoal
        self.intake_command_pub.publish(String("reset"))
        self.calibrationClient.send_goal(goal)
        self.calibrationClient.wait_for_result()
        return self.calibrationClient.get_result().succeeded

    def PP_client_cancel(self):
        self.PPclient.cancel_all_goals()

    def PP_client_goal(self, targetPoints2D):
        rospy.loginfo("running pure pursuit")
        try:
            targetPoints1D = self.convertToOneD(targetPoints2D)
            mirv_control.msg.PurePursuitGoal.TargetPoints = targetPoints1D
            mirv_control.msg.PurePursuitGoal.NumTargetPoints = int(
                len(targetPoints1D)/2)
            goal = mirv_control.msg.PurePursuitGoal
            self.PPclient.send_goal(goal)
            self.PPclient.wait_for_result()
            print(self.PPclient.get_result())
            return self.PPclient.get_result().angleToTarget
        except:
            rospy.logerr("Failed to run pure pursuit action")

    def pickup_client_goal(self, intakeSide, angleToTarget):
        mirv_control.msg.MovementToPiLitGoal.runPID = True
        mirv_control.msg.MovementToPiLitGoal.intakeSide = intakeSide
        mirv_control.msg.MovementToPiLitGoal.estimatedPiLitAngle = angleToTarget
        goal = mirv_control.msg.MovementToPiLitGoal
        self.pickupClient.send_goal(goal)
        self.pickupClient.wait_for_result()
        return self.pickupClient.get_result()

    def drive_into_garage(self, angleToTarget):
        mirv_control.msg.GarageGoal.runPID = True
        mirv_control.msg.GarageGoal.estimatedGarageAngle = angleToTarget
        goal = mirv_control.msg.GarageGoal
        self.garageClient.send_goal(goal)
        self.garageClient.wait_for_result()

    def indentify_garage_orientation(self, angleToTarget):
        mirv_control.msg.GarageGoal.runPID = True
        mirv_control.msg.GarageGoal.estimatedGarageAngle = angleToTarget
        goal = mirv_control.msg.GarageGoal
        self.garageClient.send_goal(goal)
        self.garageClient.wait_for_result()

    def pickup_client_cancel(self):
        self.pickupClient.cancel_all_goals()

    def CoordConversion_client_goal(self, point):
        # point is in lat long altitude
        mirv_control.msg.NavSatToTruckGoal.longitude = point[1]
        mirv_control.msg.NavSatToTruckGoal.latitude = point[0]
        mirv_control.msg.NavSatToTruckGoal.altitude = 1492  # TODO: Use actual altitude??
        goal = mirv_control.msg.NavSatToTruckGoal
        self.TruckCordClient.send_goal(goal)
        print("sentGoal")
        self.TruckCordClient.wait_for_result()
        truckPoint = self.TruckCordClient.get_result()
        print("Got Result: ", [truckPoint.truckCoordX, truckPoint.truckCoordY])
        return ([truckPoint.truckCoordX, truckPoint.truckCoordY])

    def getLatestSqlPoints(self):
        try:
            mirv_control.msg.DatabaseGoal.table = "pilits"
            self.goal = mirv_control.msg.DatabaseGoal
            self.databaseClient.send_goal(self.goal)
            self.databaseClient.wait_for_result()
            placedPiLitLocations = self.databaseClient.get_result()
            points = [[placedPiLitLocations.latitude[i], placedPiLitLocations.longitude[i]]
                      for i in range(len(placedPiLitLocations.latitude))]
            rospy.loginfo(f"PICKUP POINTS: {points}")
            return points
        except:
            rospy.logerr("Failed to retrieve Pi-Lit locations")

    def getPiLitsStored(self):
        try:
            mirv_control.msg.DatabaseGoal.table = "pilits-stored"
            goal = mirv_control.msg.DatabaseGoal
            self.databaseClient.send_goal(goal)
            self.databaseClient.wait_for_result()
            sides = self.databaseClient.get_result()
            sides = sides.altitude
            rospy.loginfo(f"Pi-Lits stored right: {sides[0]} left: {sides[1]}")
            return sides
        except:
            rospy.logerr("Failed to retrieve number of stored Pi-Lits")

    def cancelAllCommands(self, runIntake):
        print("Cancelling All")
        self.PP_client_cancel()
        self.disableTeleopDrive()
        self.pickup_client_cancel()
        self.tasks = []
        self.cancelled = True
        if(runIntake):
            self.intakeUp()
            self.magazineIn()
            time.sleep(1)
            self.stopIntakeAndMagazine()

    def stateWatchdog(self):
        if self.roverState == self.CONNECTED_DISABLED or self.roverState == self.DISCONNECTED or self.roverState == self.DOCKED:
            self.cancelAllCommands(False)
        if self.roverState == self.TELEOP_DRIVE:
            if self.TeleopClient.get_state() != "ACTIVE":
                self.enableTeleopDrive(False)
        self.stateMsg = self.roverState
        self.statePublisher.publish(self.stateMsg)

    def eSTOP(self):
        self.cancelled = True
        self.roverState = self.E_STOP
        os.system("rosnode kill --all")

    def setPiLits(self, command):
        self.cancelled = False
        if command == "simultaneous":
            self.pilit_controller.inhibit(False)
            self.pilit_controller.patternType(False)
        elif command == "wave_reverse":
            self.pilit_controller.inhibit(False)
            self.pilit_controller.patternType(True)
            self.pilit_controller.reversePattern()
        elif command == "wave":
            self.pilit_controller.inhibit(False)
            self.pilit_controller.patternType(True)
            self.pilit_controller.patternType(True)
        elif command == "idle":
            self.pilit_controller.inhibit(True)
        else:
            rospy.logwarn(
                "Received Unrecognized Light Command type: " + str(command))

    def driveToPoint(self, lat, long):
        self.cancelled = False
        self.roverState = self.AUTONOMOUS
        point = [lat, long]
        TruckPoint = self.CoordConversion_client_goal(point)
        self.PP_client_goal([TruckPoint])
        self.roverState = self.CONNECTED_ENABLED

    def placeOnePilit(self):
        self.cancelled = False
        lastState = self.roverState
        self.roverState = self.TELEOP_DRIVE_AUTONOMOUS
        self.RoverMacro.placePiLit()
        self.roverState = lastState

    def pickupOnePilit(self):
        self.cancelled = False
        lastState = self.roverState
        self.roverState = self.TELEOP_DRIVE_AUTONOMOUS
        self.RoverMacro.pickupPiLit()
        self.roverState = lastState

    def deployAllPilits(self, lat, long, heading, formation):
        self.cancelled = False
        self.roverState = self.AUTONOMOUS
        self.RoverMacro.placeAllPiLits([lat, long], heading, formation)
        self.roverState = self.CONNECTED_ENABLED

    def pickupAllPilits(self):
        self.cancelled = False
        self.roverState = self.AUTONOMOUS
        self.RoverMacro.pickupAllPiLits(True)
        self.roverState = self.CONNECTED_ENABLED

    def undock(self):
        self.cancelled = False
        self.roverState = self.AUTONOMOUS
        self.RoverMacro.undock()
        self.roverState = self.CONNECTED_DISABLED

    def dock(self):
        self.cancelled = False
        self.roverState = self.AUTONOMOUS
        self.RoverMacro.dock()
        self.roverState = self.CONNECTED_DISABLED

    def cloud_callback(self, message):

        self.heartBeatTime = rospy.get_time()
        msg = json.loads(message.data)

        subsystem = msg.get("subsystem", {})
        command = msg.get("command", {})

        if self.roverState == self.DISCONNECTED:
            if self.garage_state != "deployed":
                self.roverState = self.DOCKED
            else:
                self.roverState = self.CONNECTED_DISABLED

        if subsystem != 'heartbeat' and command != 'arcade':
            print(message, self.roverState)

        if command == "e_stop":
            self.eSTOP()
        elif subsystem == "pi_lit":
            self.setPiLits(command)
        elif subsystem == "general":
            if command == "disable":
                self.roverState = self.CONNECTED_DISABLED
            elif command == "enable":
                if self.roverState == self.CONNECTED_DISABLED:
                    self.roverState = self.CONNECTED_ENABLED
            elif command == "deploy":
                if self.garage_state != "deployed":
                    t = Thread(target=self.undock)
                    t.start()
                    self.tasks.append(t)
                else:
                    rospy.logwarn("invalid command, rover is not docked")

            elif command == "stow":
                t = Thread(target=self.dock)
                t.start()
                self.tasks.append(t)
            elif command == "cancel":
                print("Received Cancel Command")
                self.cancelAllCommands(True)
                self.roverState = self.CONNECTED_ENABLED
            elif command == "deploy_pi_lits":
                self.roverState = self.AUTONOMOUS
                heading = msg.get("commandParameters", {}).get("heading", 0)
                lat = msg.get("commandParameters", {}).get(
                    "location", {}).get("lat")
                long = msg.get("commandParameters", {}).get(
                    "location", {}).get("long")
                formation = msg.get("commandParameters", {}).get(
                    "formation", "taper_right_5")
                t = Thread(target=self.deployAllPilits,
                           args=(lat, long, heading, formation))
                t.start()
                self.tasks.append(t)
            elif command == "retrieve_pi_lits":
                t = Thread(target=self.pickupAllPilits)
                t.start()
                self.tasks.append(t)
            elif command == "enable_remote_operation":
                self.roverState = self.TELEOP_DRIVE
            elif command == "disable_remote_operation":
                self.roverState = self.CONNECTED_ENABLED
            else:
                rospy.logerr("Unknown command in general subsystem")
        elif subsystem == "heartbeat":
            self.heartBeatTime = rospy.get_time()
        elif subsystem == "intake":
            if command == "pickup_1_pi_lit":
                t = Thread(target=self.pickupOnePilit)
                t.start()
                self.tasks.append(t)
            elif command == "place_1_pi_lit":
                t = Thread(target=self.placeOnePilit)
                t.start()
                self.tasks.append(t)
            else:
                rospy.logerr(
                    "Unknown command in intake subsystem. Command: " + str(command))
        elif subsystem == "drivetrain":
            if command == "arcade":
                pass
            elif command == "to_location":
                lat = msg.get("commandParameters", []).get("lat")
                long = msg.get("commandParameters", []).get("long")
                t = Thread(target=self.driveToPoint, args=(lat, long))
                t.start()
                self.tasks.append(t)
            else:
                rospy.logerr("Unknown command in drivetrain subsystem")
        else:
            rospy.logerr("Unknown subsystem: " + str(subsystem))
        self.lastmsg = pickle.dumps(msg)

    def pointTurn(self, targetAngle, successThreshold):
        print(f"INITIATING POINT TURN WITH {targetAngle} and {successThreshold}")
        mirv_control.msg.PointTurnGoal.targetAngle = targetAngle
        mirv_control.msg.PointTurnGoal.successThreshold = successThreshold
        goal = mirv_control.msg.PointTurnGoal
        self.pointTurnClient.send_goal(goal)
        self.pointTurnClient.wait_for_result()
        return self.pointTurnClient.get_result()

    def driveDistance(self, targetDistance, velocityMPS, successThreshold):
        mirv_control.msg.DriveDistanceGoal.targetDistanceMeters = targetDistance
        mirv_control.msg.DriveDistanceGoal.velocityMPS = velocityMPS
        mirv_control.msg.DriveDistanceGoal.successThreshold = successThreshold
        goal = mirv_control.msg.DriveDistanceGoal
        self.driveDistanceClient.send_goal(goal)
        self.driveDistanceClient.wait_for_result()
        return self.driveDistanceClient.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('Master_client_py')
        interface = RoverInterface()
        # interface.run()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
