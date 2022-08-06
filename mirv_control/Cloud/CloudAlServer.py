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
    cloudConnected = False
    def __init__(self):
        rospy.init_node("CloudAlServer")

        cloud_sub = rospy.Subscriber("CloudCommands", String, self.cloud_cb)
        self._action_name = "CloudAlServer"
        self._as = actionlib.SimpleActionServer(self._action_name, mirv_control.msg.ControllerAction, auto_start = False)
        self._as.register_goal_callback(self.execute_cb)
        self._as.register_preempt_callback(self.preemptCallback)
        self._as.start()

    def execute_cb(self):
        goal = self._as.accept_new_goal()
        if(self.cloudConnected == True):
            print("Cloud connected ")
        else:
            self._result.tabletConnected = self.cloudConnected
            self._result.cloudFault = False
            self._as.set_succeeded(self._result)


    def preemptCallback(self):
        pass 

    def cloud_cb(self, message):
        self.heartBeatTime = 0
        self.cloudConnected = True
        msg = json.loads(message.data)
        subsystem = msg.get("subsystem",{})
        if subsystem == "general":
            command = subsystem.get("command", {})
            if command == "e_stop":
                pass
            elif command == "disable":
                pass
            elif command == "enable":
                pass
            elif command == "deploy":
                pass
            elif command == "stow":
                pass
            elif command == "cancel":
                pass
            elif command == "deploy_pi_lits":
                pass
            elif command == "retrieve_pi_lits":
                pass
            elif command == "enable_remote_operation":
                pass
            elif command == "disable_remote_operation":
                pass
            else:
                rospy.logerr("Unknown command in general subsystem")

        elif subsystem == "heartbeat":
            command = subsystem.get("command", {})
            if command == "heartbeat":
                pass
            else:
                rospy.logerr("Unknown command in general subsystem")
        
        elif subsystem == "intake":
            command = subsystem.get("command", {})
            if command == "pickup_1_pi_lit":
                pass
            elif command == "place_1_pi_lit":
                pass
            else:
                rospy.logerr("Unknown command in intake subsystem")
        elif subsystem == "drivetrain":
            if command == "arcade":
                pass
            elif command == "to_location":
                pass
            else:
                rospy.logerr("Unknown command in intake subsystem")


        else:
            rospy.logerr("Unknown subsystem")
        print("looping")
        
        self._as.publish_feedback(self._feedback)

    def run(self):
        print("starting Cloud actionLibserver")
        loopTime = rospy.get_time()-lastTime
        lastTime = rospy.get_time()
        while not rospy.is_shutdown():
            self.heartBeatTime += loopTime
            if (self.heartBeatTime >= 1):
                self.cloudConnected = False
                rospy.logwarn("lost Connection to Cloud")
                self._as.set_aborted()
                


if __name__ == '__main__':
    server = CloudAlServer()
    server.run()
    
            
