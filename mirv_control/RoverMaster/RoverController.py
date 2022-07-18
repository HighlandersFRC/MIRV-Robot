#!/usr/bin/env python3

# Effectively Robot Main, responsible for 
# periodicly checking for faults,
# managing hierarchy of systems
# calling macros or subsystem calls

from std_msgs import String
import RoverInterface
import mirv_control.msg
import rospy
import actionlib

class RoverController():
    # create messages that are used to publish feedback/result
    _feedback = mirv_control.msg.ControllerFeedback()
    _result = mirv_control.msg.ControllerResult()
  
    def __init__(self):
        rospy.init_node("RoverController")

        cloud_sub = rospy.Subscriber("CloudCommands", String, self.cloud_cb)

        self._action_name = "RoverController"
        self._as = actionlib.SimpleActionServer(self._action_name, mirv_control.msg.ControllerAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def cloud_cb(self, msg):
        joystick = msg.joystick
        purePursuit = msg.purePursuit
        ppTarget = msg.ppTarget
        pickup = msg.pickup
        self._feedback.joystick = joystick
        self._feedback.purePursuit = purePursuit
        self._feedback.ppTarget = ppTarget
        self._feedback.pickup = pickup

        self._as.publish_feedback(self._feedback)

    
            