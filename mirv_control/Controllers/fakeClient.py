#! /usr/bin/env python3
from __future__ import print_function
import rospy
import time

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import mirv_control.msg as msg

def fakeClient():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('RobotController', msg.MovementToPiLitAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    print("CONNECTED TO SERVER")

    # Creates a goal to send to the action server.
    goal = msg.MovementToPiLitGoal(runPID=True, intakeSide = "switch_right")

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    # return client.get_result()  # A FibonacciResult

    time.sleep(3)

    client.send_goal(goal)

    print("SENDING SECOND GOAL")

    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fake_client_py')
        result = fakeClient()
        print("RESULT: ", result)
        # print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion")