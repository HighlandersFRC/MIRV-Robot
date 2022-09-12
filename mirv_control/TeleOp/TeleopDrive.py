#!/usr/bin/env python3
import math
import numpy as np
import rospy
import json
import actionlib
import mirv_control.msg as ASmsg
from geometry_msgs.msg import Twist
from std_msgs.msg import String



class TeleopDrive():
    result = ASmsg.NavSatToTruckResult()
    maxAngularVel = 1.5
    maxStrafeVel = 1
    rosPubMsg = Twist()
    active = False
    result = ASmsg.generalResult()
    def __init__(self):
        rospy.init_node('TeleopDrive', anonymous=True)
        self.cloud_sub = rospy.Subscriber("CloudCommands", String, self.cloud_cb)
        self.drive_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)

        self._action_name = "TeleopDrive"
        self._as = actionlib.SimpleActionServer(self._action_name, ASmsg.mirv_control.msg.generalAction, auto_start = False)
        self._as.register_goal_callback(self.execute_cb)
        self._as.register_preempt_callback(self.preemptCallback)
        self._as.start()

    def scaleJoyInput(self, x, y):
        angVel = -x * self.maxAngularVel
        linVel = self.maxStrafeVel * abs(y) * -y

        #self.rosPubMsg.linear.x = -y
        #self.rosPubMsg.angular.z = -x
        return linVel, angVel
        

    def preemptCallback(self):
        if(self._as.is_new_goal_available()):
            self._as.set_preempted()
            
        else:
            print("aborting teleop_drive")
            self._as.set_aborted()

    def execute_cb(self):
        goal = self._as.accept_new_goal()
        # self.active = True   dont need
        # while goal.is_active():
        #     time.sleep(0.1)
        # self._as.set_succeeded()


    def cloud_cb(self, data):

        msg = json.loads(data.data) #Needs to be data.data
        command = msg.get("command","")


        

        if self._as.is_active:


            if command == "disable_remote_operation" or command == "disable":
                print("\n Dasabeling remote control\n")
                joystickX = 0
                joystickY = 0
                self.active = False
                self.result.result = ""
                self.rosPubMsg.linear.x = 0
                self.rosPubMsg.angular.z = 0
                self.drive_pub.publish(self.rosPubMsg)

                self._as.set_succeeded(self.result)
            else:
                if "commandParameters" in msg and msg["commandParameters"] is not None:
                    joystickX = msg.get("commandParameters", {}).get("x",0)
                    joystickY = msg.get("commandParameters", {}).get("y",0)
                    linear, angular = self.scaleJoyInput(joystickX, joystickY)
                    self.rosPubMsg.linear.x = linear
                    self.rosPubMsg.angular.z = angular
                    self.drive_pub.publish(self.rosPubMsg)

                
                
        
    def run(self):
        rospy.spin()

teleop_drive = TeleopDrive()
if __name__ == '__main__':
    teleop_drive.run()
