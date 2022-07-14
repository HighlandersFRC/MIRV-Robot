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



def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('PurePursuitAS', mirv_control.msg.PurePursuitAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    mirv_control.msg.PurePursuitGoal.TargetPoints = [5,0]
    mirv_control.msg.PurePursuitGoal.NumTargetPoints = 1
    goal = mirv_control.msg.PurePursuitGoal
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    print(client.get_result())
    time.sleep(10)
    mirv_control.msg.PurePursuitGoal.TargetPoints = [3,-2]
    mirv_control.msg.PurePursuitGoal.NumTargetPoints = 1
    goal = mirv_control.msg.PurePursuitGoal
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    # Prints out the result of executing the action
    print(client.get_result())  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client()
        print(result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)