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
        self.pickupClient = actionlib.SimpleActionClient("PickupAS", mirv_control.msg.MovementToPiLitAction)
        print("connected to Pure Pursuit Server")

    def convertToOneD(TwoDArray):
        temp
        for point in TwoDArray:
            temp.append(point[0])
            temp.append(point[1])
        if len(temp)/2 == len(TwoDArray):
            return temp
        else:
            raise Exeption 


    def PP_client(self, targetPoints2D):
        targetPoints1D = self.convertToOneD(targetPoints2D)
        mirv_control.msg.PurePursuitGoal.TargetPoints = [5,0]
        mirv_control.msg.PurePursuitGoal.NumTargetPoints = 1
        goal = mirv_control.msg.PurePursuitGoal
        self.PPclient.send_goal(goal)
        self.PPclient.wait_for_result()
        print(client.get_result())  # A FibonacciResult

    def moveToPiLit_client(self, intakeSide):
        mirv_control.msg.MovementToPiLitGoal.runPID = True
        mirv_control.msg.MovementToPiLitGoal.intakeSide = intakeSide

        goal = mirv_control.msg.MovementToPiLitGoal
        self.pickupClient.send_goal(goal)

        self.pickupClient.wait_for_result()
        
if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('Master_client_py')
        RoverInterface().PP_client
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)