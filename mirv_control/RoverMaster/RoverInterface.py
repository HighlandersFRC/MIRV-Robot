#!/usr/bin/env python3
from __future__ import print_function
import rospy
# Brings in the SimpleActionClient
import actionlib
import time
from std_msgs.msg import Float64, Float64MultiArray, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import os

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import mirv_control.msg

from mirv_control.msg import pilit_db_msg

from sensor_msgs.msg import NavSatFix


class RoverInterface():
    def __init__(self):
        # print("setting up server connections")
        # self.calibrationClient = actionlib.SimpleActionClient('StartingHeading', mirv_control.msg.IMUCalibrationAction)
        # self.calibrationClient.wait_for_server()
        # print("connected to starting heading AS")
        # self.PPclient = actionlib.SimpleActionClient('PurePursuitAS', mirv_control.msg.PurePursuitAction)
        # self.PPclient.wait_for_server()
        # print("connected to Pure Pursuit Server")
        # self.pickupClient = actionlib.SimpleActionClient("PickupAS", mirv_control.msg.MovementToPiLitAction)
        # self.pickupClient.wait_for_server()
        # print("connected to PiLit pickup Server")
        self.cloudControllerClient = actionlib.SimpleActionClient("CloudAlServer", mirv_control.msg.ControllerAction)
        self.cloudControllerClient.wait_for_server()
        print("connected to Cloud AL Server")
        # self.TruckCordClient = actionlib.SimpleActionClient('NavSatToTruckAS', mirv_control.msg.NavSatToTruckAction)
        # self.TruckCordClient.wait_for_server()
        # print("connected to Truck CordinateAS")
        # self.databaseClient = actionlib.SimpleActionClient("Database", mirv_control.msg.DatabaseAction)
        # self.databaseClient.wait_for_server()
        # print("connected to database server")
        # self.TeleopClient = actionlib.SimpleActionClient("TeleopDrive", mirv_control.msg.generalAction)
        # self.TeleopClient.wait_for_server()
        # print("connected to teleop drive")


        # self.garageClient = actionlib.SimpleActionClient("Docking", mirv_control.msg.GarageAction)
        # self.garageClient.wait_for_server()
        # print("connected to Garage AL Server")

        # self.odometrySub = rospy.Subscriber("/EKF/Odometry", Odometry, self.updateOdometry)
        self.gpsOdomSub = rospy.Subscriber("gps/fix", NavSatFix, self.updateOdometry)
        self.truckOdomSub = rospy.Subscriber("/EKF/Odometry", Odometry, self.updateTruckOdom)
        self.sqlPub = rospy.Publisher("pilit/events", pilit_db_msg, queue_size=5)
        self.simpleDrivePub = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)

        self.intake_command_pub = rospy.Publisher("intake/command", String, queue_size = 10)

        # self.placementSub = rospy.Subscriber('pathingPointInput', Float64MultiArray, self.updatePlacementPoints)

        self.placementLocationSub = rospy.Subscriber("placementLocation", Float64MultiArray, self.updatePlacementPoints)

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
        print("finished interface setup")

    def updatePlacementPoints(self, data):
        # print(data.data)
        self.placementPoints = self.convertOneDimArrayToTwoDim(list(data.data))

    def convertOneDimArrayToTwoDim(self, points):
        pointList = []
        # print((len(points))/2)
        for i in range(int((len(points))/2)):
            tempX = points[0]
            points.pop(0)
            tempY = points[0]
            points.pop(0)
            pointList.append([tempX, tempY])
        return pointList
                # print(pointList)

    def getPlacementPoints(self):
        return self.placementPoints

    def convertPointsToTruckCoordinates(self, points):
        for i in range(len(points)):
            points[i] = self.CoordConversion_client_goal(points[i])

        return points
    
    def getDriveClients(self):
        return self.calibrationClient, self.PPclient, self.pickupClient, self.cloudControllerClient
        
    def sendCmdVel(self, linear, angular):
        pass
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
        time.sleep(4)
        self.intake_command_pub.publish(String("reset"))
        self.calibrationClient.send_goal(goal)
        self.calibrationClient.wait_for_result()
        return self.calibrationClient.get_result().succeeded

    def PP_client_cancel(self):
        print("canceling pure pursuit goal")
        self.PPclient.cancel_all_goals()
        print(self.PPclient.get_goal_status_text())
    def PP_client_goal(self, targetPoints2D):
        print("running pure pursuit")
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
            print("failed to run Pure pursuit action")

    def cloudController_client_goal(self):
        mirv_control.msg.ControllerGoal.runRequested = True
        goal = mirv_control.msg.ControllerGoal
        self.cloudControllerClient.send_goal(goal, feedback_cb = self.cloud_feedback_callback)
        self.cloudControllerClient.wait_for_result()
        print(self.cloudControllerClient.get_result())

    def pickup_client_goal(self, intakeSide, angleToTarget):
        mirv_control.msg.MovementToPiLitGoal.runPID = True
        mirv_control.msg.MovementToPiLitGoal.intakeSide = intakeSide
        mirv_control.msg.MovementToPiLitGoal.estimatedPiLitAngle = angleToTarget

        goal = mirv_control.msg.MovementToPiLitGoal
        self.pickupClient.send_goal(goal)

        self.pickupClient.wait_for_result()

        # print(self.pickupClient.get_result())

    def garage_client_goal(self, angleToTarget):
        mirv_control.msg.GarageGoal.runPID = True
        print(type(angleToTarget))
        mirv_control.msg.GarageGoal.estimatedGarageAngle = angleToTarget

        garageGoal = mirv_control.msg.GarageGoal
        self.garageClient.send_goal(garageGoal)

        self.garageClient.wait_for_result()

        # print(self.pickupClient.get_result())

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
        mirv_control.msg.DatabaseGoal.table = "pilits"
        self.goal = mirv_control.msg.DatabaseGoal
        self.databaseClient.send_goal(self.goal)
        self.databaseClient.wait_for_result()
        placedPiLitLocations = self.databaseClient.get_result()
        points = [[placedPiLitLocations.latitude[i], placedPiLitLocations.longitude[i]] for i in range(len(placedPiLitLocations.latitude))]
        print("PICKUP POINTS: ", points)
        return points

    def feedback_callback(self, msg):
        pass
        # print(msg)
    def cloud_feedback_callback(self, msg):
        print(rospy.get_time())
        print(msg)
        if msg.EStop:
            os.system("rosnode kill --all")
        if msg.teleopDrive:
            pass
            # self.PP_client_cancels
            # self.enableTeleopDrive
            # self.pickup_client_cancel


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('Master_client_py')
        interface = RoverInterface()
        # print(interface.convertToOneD([[1,2],[3,4],[5,6]]))
        # interface.run()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)