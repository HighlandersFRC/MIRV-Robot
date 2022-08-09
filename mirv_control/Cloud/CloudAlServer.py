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
import time

class CloudAlServer():
    # create messages that are used to publish feedback/result
    _feedback = mirv_control.msg.ControllerFeedback()
    _result = mirv_control.msg.ControllerResult()
    cloudConnected = False
    lastCommandStatus = _feedback
    heartBeatTime = 0
    def __init__(self):
        rospy.init_node("CloudAlServer")

        cloud_sub = rospy.Subscriber("CloudCommands", String, self.cloud_cb)
        self._action_name = "CloudAlServer"
        self._as = actionlib.SimpleActionServer(self._action_name, mirv_control.msg.ControllerAction, auto_start = False)
        self._as.register_goal_callback(self.execute_cb)
        self._as.register_preempt_callback(self.preemptCallback)
        self._as.start()
        self.resetToDefault()
        lastCommandStatus = self._feedback
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
    def resetToDefault(self):
        lastCommandStatus = self._feedback
        self._feedback.connected = self.cloudConnected
        self._feedback.stow = True
        self._feedback.deploy = False
        self._feedback.teleopDrive = False
        self._feedback.EStop = False
        self._feedback.placePiLit = False
        self._feedback.pickupPiLit = False
        self._feedback.connectedEnabled = False
        self._feedback.cancelAutoDrive = True
        self._feedback.deployAllPiLits = False
        self._feedback.retrieveAllPiLits = False
        self._feedback.placeLatLong = []
        self._feedback.formationType = ""
        self._feedback.driveToWaypoint = False
        self._feedback.driveToLatLong = []

    def resetControlState(self):
        self._feedback.teleopDrive = False
        self._feedback.placePiLit = False
        self._feedback.pickupPiLit = False
        self._feedback.deployAllPiLits = False
        self._feedback.retrieveAllPiLits = False
        self._feedback.placeLatLong = []
        self._feedback.formationType = ""
        self._feedback.driveToWaypoint = False
        self._feedback.driveToLatLong = []
    
    def cloud_cb(self, message):
        self.heartBeatTime = 0
        self.cloudConnected = True
        sendRequired = True
        self._feedback.connected = self.cloudConnected
        msg = json.loads(message.data)
        self._feedback.pickupPiLit = False
        self._feedback.placePiLit = False
        subsystem = msg.get("subsystem",{})
        if subsystem == "general":
            command = msg.get("command", {})
            if command == "e_stop":
                self._feedback.EStop = True
            elif command == "disable":
                self._feedback.teleopDrive = False
                self._feedback.connectedEnabled = False
                self._feedback.cancelCommand = True
            elif command == "enable":
                self._feedback.connectedEnabled = True
            elif command == "deploy":
                self._feedback.deploy = True
            elif command == "stow":
                self._feedback.stow = True
            elif command == "cancel":
                self._feedback.cancelCommand = True
            elif command == "deploy_pi_lits":
                print(msg)
                self.resetControlState()
                self._feedback.deployAllPiLits = True
                self._feedback.placeLatLong = [msg.get("commandParameters", {}).get("location",{}).get("lat"),msg.get("commandParameters", {}).get("location",{}).get("long") ]
                self._feedback.formationType = msg.get("commandParameters", {}).get("formation")
            elif command == "retrieve_pi_lits":
                self.resetControlState()
                self._feedback.retrieveAllPiLits = True
            elif command == "enable_remote_operation":
                self.resetControlState()
                self._feedback.teleopDrive = True
            elif command == "disable_remote_operation":
                self.resetControlState()
                self._feedback.teleopDrive = False
            else:
                rospy.logerr("Unknown command in general subsystem")

        elif subsystem == "heartbeat":
            command = msg.get("command", {})
            if command == "heartbeat":
                sendRequired = False
                print("recived heartbeat Command")
            else:
                rospy.logerr("Unknown command in general subsystem")
        
        elif subsystem == "intake":
            command = msg.get("command", {})
            if command == "pickup_1_pi_lit":
                self._feedback.pickupPiLit = True
                self._feedback.placePiLit = False
            elif command == "place_1_pi_lit":
                self._feedback.placePiLit = True
                self._feedback.pickupPiLit = False
            else:
                rospy.logerr("Unknown command in intake subsystem")
        elif subsystem == "drivetrain":
            command = msg.get("command", {})
            if command == "arcade":
                self.resetControlState()
                self._feedback.teleopDrive = True
            elif command == "to_location":
                self.resetControlState()
                self._feedback.driveToWaypoint = True
                self._feedback.driveToLatLong = [msg.get("commandParameters", []).get("lat"),msg.get("commandParameters", []).get("long")]
            else:
                rospy.logerr("Unknown command in drivetrain subsystem")


        else:
            rospy.logerr("Unknown subsystem")
        print("looping")
        
        if (self._as.is_active() and sendRequired):
            print("sending message")
            self._as.publish_feedback(self._feedback)


    def run(self):
        print("starting Cloud actionLibserver")
        lastTime = rospy.get_time()
        while not rospy.is_shutdown():
            loopTime = rospy.get_time()-lastTime
            lastTime = rospy.get_time()
            self.heartBeatTime += loopTime
            if (self.heartBeatTime >= 1.5):
                self.cloudConnected = False
                self.resetToDefault()
                # rospy.logwarn("lost Connection to Cloud")
                if (self._as.is_active()):
                    self._as.set_aborted()
            time.sleep(0.5)


if __name__ == '__main__':
    server = CloudAlServer()
    server.run()
