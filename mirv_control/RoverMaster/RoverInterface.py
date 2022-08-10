#!/usr/bin/env python3
from __future__ import print_function
import rospy
from PiLitController import PiLitControl
# Brings in the SimpleActionClient
import actionlib
import time
from std_msgs.msg import Float64, Float64MultiArray, String
from mirv_control.msg import garage_state_msg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import os
import sys

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import mirv_control.msg

from mirv_control.msg import pilit_db_msg

from sensor_msgs.msg import NavSatFix


class RoverInterface():
    RoverMacro = None
    stateMsg = String
    def __init__(self):
        self.roverState = "disconnected"
        print("setting up server connections")
        self.calibrationClient = actionlib.SimpleActionClient('StartingHeading', mirv_control.msg.IMUCalibrationAction)
        self.calibrationClient.wait_for_server()
        print("connected to starting heading AS")
        self.PPclient = actionlib.SimpleActionClient('PurePursuitAS', mirv_control.msg.PurePursuitAction)
        self.PPclient.wait_for_server()
        print("connected to Pure Pursuit Server")
        self.pickupClient = actionlib.SimpleActionClient("PickupAS", mirv_control.msg.MovementToPiLitAction)
        self.pickupClient.wait_for_server()
        print("connected to PiLit pickup Server")
        self.cloudControllerClient = actionlib.SimpleActionClient("CloudAlServer", mirv_control.msg.ControllerAction)
        self.cloudControllerClient.wait_for_server()
        print("connected to Cloud AL Server")
        # self.TruckCordClient = actionlib.SimpleActionClient('NavSatToTruckAS', mirv_control.msg.NavSatToTruckAction)
        # self.TruckCordClient.wait_for_server()
        print("connected to Pure Truck CordinateAS")
        self.databaseClient = actionlib.SimpleActionClient("Database", mirv_control.msg.DatabaseAction)
        self.databaseClient.wait_for_server()
        print("connected to Database Query Server")
        self.garageClient = actionlib.SimpleActionClient("Docking", mirv_control.msg.GarageAction)
        self.garageClient.wait_for_server()
        print("connected to Garage Server")

        #self.pilit_controller = PiLitControl()

        # SUBSCRIBERS
        # self.odometrySub = rospy.Subscriber("/EKF/Odometry", Odometry, self.updateOdometry)
        self.gpsOdomSub = rospy.Subscriber("gps/fix", NavSatFix, self.updateOdometry)
        self.truckOdomSub = rospy.Subscriber("/EKF/Odometry", Odometry, self.updateTruckOdom)
        self.placementLocationSub = rospy.Subscriber("placementLocation", Float64MultiArray, self.updatePlacementPoints)
        self.garage_sub = rospy.Subscriber("GarageStatus", String, self.garage_state_callback)
        self.intake_limit_switch_sub = rospy.Subscriber("intake/limitswitches", Float64MultiArray, self.limit_switch_callback)


        # PUBLISHERS
        self.sqlPub = rospy.Publisher("pilit/events", pilit_db_msg, queue_size=5)
        self.simpleDrivePub = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)
        self.intake_command_pub = rospy.Publisher("intake/command", String, queue_size = 5)
        self.garage_pub = rospy.Publisher("GarageCommands", String, queue_size = 5)
        self.statePublisher = rospy.Publisher("RoverState", String, queue_size = 5)
        # self.placementSub = rospy.Subscriber('pathingPointInput', Float64MultiArray, self.updatePlacementPoints)

        self.isJoystickControl = True
        self.isPurePursuitControl = False
        self.purePursuitTarget = []
        self.isPickupControl = False
        self.latitude = 0
        self.longitude = 0
        self.altitude = 0
        self.xPos = 0
        self.yPos = 0
        self.placementPoints = []
        self.storedPiLits = [4, 4]
        self.garage_state = "invalid"

    # def setPiLitSequence(self, is_wave: bool):
    #     self.pilit_controller.patternType(is_wave)
    #     self.pilit_controller.reset()

    # def setPiLitSequenceReversed(self, reversed: bool):
    #     self.pilit_controller.reversePattern(reversed)
    #     self.pilit_controller.reset()
    def loadRoverMacro(self, macro):
        self.RoverMacro = macro
    def garage_state_callback(self, msg):
        self.garage_state = msg.data

    def limit_switch_callback(self, switches):
        limit_switches = switches.data

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
        startTime = time.time()
        self.simpleDrivePub.publish(twist)
        while time.time() - startTime < seconds:
            pass
        stop = Twist()
        stop.angular.z = 0
        self.simpleDrivePub.publish(stop)
        return True

    def updatePlacementPoints(self, data):
        self.placementPoints = self.convertOneDimArrayToTwoDim(list(data.data))

    def convertOneDimArrayToTwoDim(self, points):
        pointList = []
        for i in range(int((len(points))/2)):
            tempX = points[0]
            points.pop(0)
            tempY = points[0]
            points.pop(0)
            pointList.append([tempX, tempY])
        return pointList

    def getPlacementPoints(self):
        return self.placementPoints

    def convertPointsToTruckCoordinates(self, points):
        for i in range(len(points)):
            points[i] = self.CoordConversion_client_goal(points[i])
        return points
    
    def getDriveClients(self):
        return self.calibrationClient, self.PPclient, self.pickupClient, self.cloudControllerClient

    def updateOdometry(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.altitude = data.altitude

    def updateTruckOdom(self, data):
        self.xPos = data.pose.pose.position.x
        self.yPos = data.pose.pose.position.y

    def getCurrentTruckOdom(self):
        return ([self.xPos, self.yPos])

    def getCurrentLatitude(self):
        return self.latitude

    def getCurrentLongitude(self):
        return self.longitude

    def getCurrentAltitude(self):
        return self.altitude

    def getPlacementPoints(self):
        return self.placementPoints
    
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
        mirv_control.msg.generalGoal.goal = ""
        self.TeleopClient.send_goal(mirv_control.msg.generalGoal.goal)
        if Halt:
            self.TeleopClient.wait_for_result()

    def disableTeleopDrive(self):
        self.TeleopClient.cancel_all_goals()

    def convertToOneD(self,TwoDArray):
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
        rospy.loginfo("canceling pure pursuit goal")
        self.PPclient.cancel_all_goals()
        rospy.loginfo(self.PPclient.get_goal_status_text())

    def PP_client_goal(self, targetPoints2D):
        rospy.loginfo("running pure pursuit")
        try:
            targetPoints1D = self.convertToOneD(targetPoints2D)
            mirv_control.msg.PurePursuitGoal.TargetPoints = targetPoints1D
            mirv_control.msg.PurePursuitGoal.NumTargetPoints = int(len(targetPoints1D)/2)
            goal = mirv_control.msg.PurePursuitGoal
            self.PPclient.send_goal(goal)
            self.PPclient.wait_for_result()
            print(self.PPclient.get_result())
            return self.PPclient.get_result().angleToTarget
        except:
            rospy.logerr("Failed to run pure pursuit action")

    def cloudController_client_goal(self):
        mirv_control.msg.ControllerGoal.runRequested = True
        goal = mirv_control.msg.ControllerGoal
        self.cloudControllerClient.send_goal(goal, feedback_cb = self.cloud_feedback_callback)
        self.cloudControllerClient.wait_for_result()

    def pickup_client_goal(self, intakeSide, angleToTarget):
        mirv_control.msg.MovementToPiLitGoal.runPID = True
        mirv_control.msg.MovementToPiLitGoal.intakeSide = intakeSide
        mirv_control.msg.MovementToPiLitGoal.estimatedPiLitAngle = angleToTarget
        goal = mirv_control.msg.MovementToPiLitGoal
        self.pickupClient.send_goal(goal)
        self.pickupClient.wait_for_result()

    def garage_client_goal(self, angleToTarget):
        mirv_control.msg.GarageGoal.runPID = True
        mirv_control.msg.GarageGoal.estimatedGarageAngle = angleToTarget
        goal = mirv_control.msg.GarageGoal
        self.garageClient.send_goal(goal)
        self.garageClient.wait_for_result()

    def pickup_client_cancel(self, intakeSide):
        print("Cancelling pickup goal")
        self.pickupClient.cancel_all_goals()
        print(self.pickupClient.get_goal_status_text())

    def CoordConversion_client_goal(self, point):
        # point is in lat long altitude
        mirv_control.msg.NavSatToTruckGoal.longitude = point[1]
        mirv_control.msg.NavSatToTruckGoal.latitude = point[0]
        mirv_control.msg.NavSatToTruckGoal.altitude = 1492
        goal = mirv_control.msg.NavSatToTruckGoal
        self.TruckCordClient.send_goal(goal)
        print("sentGoal")
        self.TruckCordClient.wait_for_result()
        truckPoint = self.TruckCordClient.get_result()
        return ([truckPoint.truckCoordX, truckPoint.truckCoordY])

    def getLatestSqlPoints(self):
        try:
            mirv_control.msg.DatabaseGoal.table = "pilits"
            self.goal = mirv_control.msg.DatabaseGoal
            self.databaseClient.send_goal(self.goal)
            self.databaseClient.wait_for_result()
            placedPiLitLocations = self.databaseClient.get_result()
            points = [[placedPiLitLocations.latitude[i], placedPiLitLocations.longitude[i]] for i in range(len(placedPiLitLocations.latitude))]
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
            rospy.loginfo(f"Pi-Lits stored left: {sides[0]} right: {sides[1]}")
            return sides
        except:
            rospy.logerr("Failed to retrieve number of stored Pi-Lits")
    
    def cancelAllCommands(self):
        self.PP_client_cancel()
        self.disableTeleopDrive()
        self.pickup_client_cancel()
        self.intakeUp()
        self.magazineIn()
        self.stopIntakeAndMagazine()
        

    def stateWatchdog(self):
        if self.roverState == "connected_disabled" or self.roverState == "disconnected":
            self.cancelAllCommands
        if self.roverState == "teleop_drive":
            if self.TeleopClient.get_state() != "ACTIVE":
                self.enableTeleopDrive(False)
        self.stateMsg = self.roverState
        self.statePublisher.publish(self.stateMsg)

    def cloud_feedback_callback(self, msg):
        print(rospy.get_time())
        print(msg)
        if msg.EStop:
            self.roverState = "e_stop"
            os.system("rosnode kill --all")
        if not msg.connectedEnabled:
            self.cancelAllCommands()
            self.roverState = "connected_disabled"  
        if msg.connected:
            if self.garage_state != "deployed":
                if msg.deploy:
                    self.RoverMacros.undock
                    self.roverState = "connected_enabled"
                else:
                    rospy.logwarn("invalid command, rover is docked")
                    self.roverState = "connected_disabled"
            else:
                if self.roverState == "connected_enabled":
                    if not msg.connectedEnabled:
                        self.cancelAllCommands()
                        self.roverState = "connected_disabled"
                    else:
                        if (msg.teleopDrive and (msg.pickupPiLit or msg.placePiLit)):
                            self.roverState = "teleop_drive_autonomous"
                            if msg.pickupPiLit:
                                pass
                                # TODO:put macro here
                            if msg.placePiLit:
                                pass
                                # TODO:put macro here
                            self.roverState = "teleop_drive"
                        elif msg.teleopDrive:
                            self.roverState = "teleop_drive"
                            # TODO:put macro here
                            self.roverState = "connected_enabled"

                        else:
                            self.roverState = "autonomous"
                            if msg.deployAllPiLits:
                                # TODO:put macro here
                                self.roverState = "connected_enabled"
                            if msg.retrieveAllPiLits:
                                # TODO:put macro here
                                self.roverState = "connected_enabled"
                            if msg.driveToWaypoint:
                                # TODO:put macro here
                                self.roverState = "connected_enabled"
                            if msg.stow:
                                pass
                                # TODO:put macro here
                                self.roverState = "connected_enabled"
                else:
                    if msg.cancelCommand:
                        self.cancelAllCommands
                        self.roverState = "connected_enabled"
                    else:
                        if self.roverState == "connected_disabled":
                            if msg.connectedEnabled:
                                self.roverState = "connected_enabled"
                            else:
                                rospy.logwarn("invalid command, rover is disabled, must be enabled to take commands")
                        else:
                            rospy.logwarn("invalid command, rover is in state: {}, and must be in connected enable to accept new commands".format(self.roverState))
        else:
            self.roverState = "disconnected"



if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('Master_client_py')
        interface = RoverInterface()
        # interface.run()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)