#!/usr/bin/env python3
import math
import numpy as np
import rospy
import actionlib
import mirv_control.msg as ASmsg
from geometry_msgs.msg import Twist
from std_msgs.msg import String



class TeleopDrive():
    result = ASmsg.NavSatToTruckResult()
    maxAngularVel = 2
    maxStrafeVel = 1
    rosPubMsg = Twist()
    active = False
    def __init__(self):
        rospy.init_node('TeleopDrive', anonymous=True)
        self.cloud_sub = rospy.Subscriber("CloudCommands", String, self.cloud_cb)
        self.drive_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)

        self._action_name = "TeleopDrive"
        self._as = actionlib.SimpleActionServer(self._action_name, ASmsg.mirv_control.msg.NavSatToTruckAction, auto_start = False)
        self._as.register_goal_callback(self.execute_cb)
        self._as.start()

    def scaleJoyInput(self, x, y):
        angVel = x * self.maxAngularVel
        linVel = self.maxStrafeVel * abs(y) * -y

        #self.rosPubMsg.linear.x = -y
        #self.rosPubMsg.angular.z = -x
        return linVel, angVel
        


    def execute_cb(self):
        goal = self._as.accept_new_goal()
        self.active = True
        while goal.is_active():
            time.sleep(0.1)
        self._as.set_succeeded()


    def cloud_cb(self, data):

        msg = json.loads(data)
        command = msg.get("command","")
        joystickX = msg.get("commandParameters", {}).get("x",0)
        joystickY = msg.get("commandParameters", {}).get("y",0)

        if self.active:
            if command == "disable_remote_operation":
                joystickX = 0
                joystickY = 0
                self.active = False
                self._as.set_succeeded()

            
            linear, angular = self.scaleJoyInput(joystickX, joystickY)
            self.rosPubMsg.linear.x = linear
            self.rosPubMsg.angular.z = angular
            self.drive_pub.publish(self.rosPubMsg)
        
    def run(self):
        rospy.spin()

teleop_drive = TeleopDrive()
if __name__ == '__main__':
    teleop_drive.run()
