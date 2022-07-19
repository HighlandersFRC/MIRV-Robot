#!/usr/bin/env python3

# Effectively Robot Main, responsible for 
# periodicly checking for faults,
# managing hierarchy of systems
# calling macros or subsystem calls

import math
from std_msgs.msg import String
import mirv_control.msg
from geometry_msgs.msg import Twist
import rospy
import actionlib
import json

class CloudAlServer():
    # create messages that are used to publish feedback/result
    _feedback = mirv_control.msg.ControllerFeedback()
    _result = mirv_control.msg.ControllerResult()
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
    rosPubMsg = Twist()
    maxAngularVel = 2
    maxStrafeVel = 1
    lastJoy = True

    def __init__(self):
        rospy.init_node("CloudAlServer")

        cloud_sub = rospy.Subscriber("CloudCommands", String, self.cloud_cb)

        self._action_name = "CloudAlServer"
        self._as = actionlib.SimpleActionServer(self._action_name, mirv_control.msg.ControllerAction, auto_start = False)
        self._as.register_goal_callback(self.execute_cb)
        self._as.register_preempt_callback()
        self._as.start()

    def scaleJoyInput(self, x, y):
        # x = 1 + (-4 * math.tanh(abs(y)))/math.pi
        angVel = x * self.maxAngularVel
        linVel = self.maxStrafeVel * abs(y) * -y
        self.rosPubMsg.linear.x = -y
        self.rosPubMsg.angular.z = -x

    def execute_cb(self):
        goal = self._as.accept_new_goal()
        if(self.joystick == True):
            print("driving with a joystick")
        else:
            self._result.purePursuit = purePursuit
            self._result.ppTarget = ppTarget
            self._result.pickup = pickup
            self._result.finished = True
            self._as.set_succeeded(self._result)
        

    def cloud_cb(self, message):
        msg = json.loads(message.data)
        joystickX = msg.get("commandParameters", {}).get("x",0)
        joystickY = msg.get("commandParameters", {}).get("y",0)
        print("looping")
        
        if(joystickX != 0 or joystickY != 0):
            self.joystick = True
            self.lastJoy = True
            self.scaleJoyInput(joystickX, joystickY)
            print(self.rosPubMsg)
            self.pub.publish(self.rosPubMsg)
            
        else:
            self.joystick = False
        print("{}, {}".format(self.joystick, self.lastJoy))
        if(self.joystick == False and self.lastJoy == True):
            print("setting to zero")
            self.rosPubMsg.linear.x = 0
            self.rosPubMsg.angular.z = 0
            self.pub.publish(self.rosPubMsg)
            self.lastJoy = False

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
        if(self._as.is_active()):
            self._feedback.joystick = self.joystick
            self._as.publish_feedback(self._feedback)
    def run(self):
        print("starting Cloud actionLibserver")
        rospy.spin()

if __name__ == '__main__':
    server = CloudAlServer()
    server.run()
    
            