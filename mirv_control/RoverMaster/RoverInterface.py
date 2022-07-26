#!/usr/bin/env python3
from __future__ import print_function
import rospy
# Brings in the SimpleActionClient
import actionlib
import time

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import mirv_control.msg
def convertToOneD(TwoDArray):
    for point in TwoDArray:
        pass

class RoverInterface():
    def __init__(self):
        print("setting up server connections")
        self.calibrationClient = actionlib.SimpleActionClient('StartingHeading', mirv_control.msg.IMUCalibrationAction)
        self.calibrationClient.wait_for_server()
        print("connected to Pure Truck CordinateAS")
        self.PPclient = actionlib.SimpleActionClient('PurePursuitAS', mirv_control.msg.PurePursuitAction)
        self.PPclient.wait_for_server()
        print("connected to Pure Pursuit Server")
        self.pickupClient = actionlib.SimpleActionClient("PickupAS", mirv_control.msg.MovementToPiLitAction)
        self.pickupClient.wait_for_server()
        print("connected to PiLit pickup Server")
        self.cloudControllerClient = actionlib.SimpleActionClient("CloudAlServer", mirv_control.msg.ControllerAction)
        self.cloudControllerClient.wait_for_server()
        print("connected to Pure Cloud AL Server")
        self.TruckCordClient = actionlib.SimpleActionClient('NavSatToTruckAS', mirv_control.msg.NavSatToTruckAction)
        self.TruckCordClient.wait_for_server()
        print("connected to Pure Truck CordinateAS")

        self.isJoystickControl = True
        self.isPurePursuitControl = False
        self.purePursuitTarget = []
        self.isPickupControl = False

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
        goal = mirv_control.msg.IMUCalibrationGoal.calibrate = True
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
        # print(self.cloudControllerClient.get_result())

    def pickup_client_goal(self, intakeSide, angleToTarget):
        mirv_control.msg.MovementToPiLitGoal.runPID = True
        mirv_control.msg.MovementToPiLitGoal.intakeSide = intakeSide
        mirv_control.msg.MovementToPiLitGoal.estimatedPiLitAngle = angleToTarget

        goal = mirv_control.msg.MovementToPiLitGoal
        self.pickupClient.send_goal(goal)

        self.pickupClient.wait_for_result()

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

    def feedback_callback(self, msg):
        pass
        # print(msg)
    def cloud_feedback_callback(self, msg):
        # print(msg)

        self.isJoystickControl = msg.joystick
        self.isPurePursuitControl = msg.purePursuit
        self.purePursuitTarget = msg.ppTarget
        self.isPickupControl = msg.pickup

        # print(self.cloudController_client.get_state())
    # def run(self):
    #     # self.PP_client([[10,0], [10,5]])
    #     rate = rospy.Rate(1)
    #     while not rospy.is_shutdown():
    #         print("looping")
    #         self.cloudController_client_goal()
    #         rate.sleep()

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