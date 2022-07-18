#!/usr/bin/env python3

# Effectively Robot Main, responsible for 
# periodicly checking for faults,
# managing hierarchy of systems
# calling macros or subsystem calls

import math
from std_msgs import String
import RoverInterface
import mirv_control.msg
from geometry_msgs.msg import Twist
import rospy
import actionlib
import json

class RoverController():
    # create messages that are used to publish feedback/result
    _feedback = mirv_control.msg.ControllerFeedback()
    _result = mirv_control.msg.ControllerResult()
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
    rosPubMsg = Twist()
    maxAngularVel = 2
    maxStrafeVel = 3

    def __init__(self):
        rospy.init_node("CloudAlServer")

        cloud_sub = rospy.Subscriber("CloudCommands", String, self.cloud_cb)

        self._action_name = "CloudAlServer"
        self._as = actionlib.SimpleActionServer(self._action_name, mirv_control.msg.ControllerAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def scaleJoyInput(self, x, y):
        x = 1 + (-4 * math.tanh(abs(y)))/math.pi
        angVel = x * self.maxAngularVel
        linVel = y * self.maxStrafeVel
        self.rosPubMsg.linear.x = linVel
        self.rosPubMsg.angular.z = angVel
        


    def cloud_cb(self, message):
        msg = json.loads(message)
        joystickX = msg.get("commandParameters").get("x")
        joystickY = msg.get("commandParameters").get("y")

        if(joystickX != 0 or joystickY != 0):
            joystick = True
            scaleJoyInput(joystickX, joystickY)
            self.pub.publish(self.rosPubMsg)
        else:
            joystick = False
        purePursuit = msg.get("purePursuit")
        if(purePursuit == None):
            purePursuit = False
        ppTarget = msg.get("ppTarget")
        if(ppTarget == None):
            ppTarget = False
        if(msg.get("command") == "retrieve_pi_lits"):
            pickup = True
        else:
            pickup = False
        self._feedback.joystick = joystick
        self._feedback.purePursuit = purePursuit
        self._feedback.ppTarget = ppTarget
        self._feedback.pickup = pickup

        self._as.publish_feedback(self._feedback)

    
            