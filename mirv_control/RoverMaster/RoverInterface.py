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
        self.PPclient = actionlib.SimpleActionClient('PurePursuitAS', mirv_control.msg.PurePursuitAction)
        self.PPclient.wait_for_server()
        print("connected to Pure Pursuit Server")
        # self.pickupClient = actionlib.SimpleActionClient("PickupAS", mirv_control.msg.MovementToPiLitAction)
        # self.pickupClient.wait_for_server()
        # print("connected to PiLit pickup Server")
        # self.cloudControllerClient = actionlib.SimpleActionClient("CloudController", mirv_control.msg.ControllerAction)
        # self.cloudControllerClient.wait_for_server()
        # print("connected to Pure Pursuit Server")

    def convertToOneD(self,TwoDArray):
        temp = []
        for i in range(len(TwoDArray)):
            try:
                temp.append(TwoDArray[i][0])
                temp.append(TwoDArray[i][1])
            except:
                raise Exception("Invalid points entered")
        return temp

    
    def PP_client(self, targetPoints2D):
        # try:
        targetPoints1D = self.convertToOneD(targetPoints2D)
        mirv_control.msg.PurePursuitGoal.TargetPoints = targetPoints1D
        mirv_control.msg.PurePursuitGoal.NumTargetPoints = int(len(targetPoints1D)/2)
        goal = mirv_control.msg.PurePursuitGoal
        self.PPclient.send_goal(goal)
        self.PPclient.wait_for_result()
        print(client.get_result())
        # except:
            # print("failed to run Pure pursuit action")

    def cloudController_client(self):
        goal = mirv_control.msg.ControllerGoal
        self.cloudController_client.send_goal(goal, feedback_callback = self.feedback_callback)
        self.cloudController_client.wait_for_result()
        print(self.cloudController_client.get_result())

    def moveToPiLit_client(self, intakeSide):
        mirv_control.msg.MovementToPiLitGoal.runPID = True
        mirv_control.msg.MovementToPiLitGoal.intakeSide = intakeSide

        goal = mirv_control.msg.MovementToPiLitGoal
        self.pickupClient.send_goal(goal)

        self.pickupClient.wait_for_result()

        print(self.pickupClient.get_result())

    def feedback_callback(self, msg):
        print(msg)
    def run(self):
        self.PP_client([[10,0], [10,5]])

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('Master_client_py')
        interface = RoverInterface()
        # print(interface.convertToOneD([[1,2],[3,4],[5,6]]))
        interface.run()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)