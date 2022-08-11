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
    TELEOP_DRIVE = "remote_operation"
    TELEOP_DRIVE_AUTONOMOUS = "remote_operation_autonomous"
    DISABLED = "disabled"
    CONNECTED_DISABLED = "connected_disabled"
    CONNECTED_ENABLED = "connected_idle"
    AUTONOMOUS = "autonomous"
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
        print("got request from client")
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
        self._feedback.stow = False
        self._feedback.deploy = False
        self._feedback.teleopDrive = False
        self._feedback.EStop = False
        self._feedback.placePiLit = False
        self._feedback.pickupPiLit = False
        self._feedback.connectedEnabled = False
        self._feedback.cancelCommand = False
        self._feedback.deployAllPiLits = False
        self._feedback.retrieveAllPiLits = False
        self._feedback.placeLatLong = []
        self._feedback.formationType = ""
        self._feedback.driveToWaypoint = False
        self._feedback.driveToLatLong = []
        self._feedback.heartbeat = False

    def resetControlState(self):
        self._feedback.heartbeat = False
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
        # print("got callback from cloud")
        self.heartBeatTime = rospy.get_time()
        if self.cloudConnected == False:
            sendFirst = True
            time.sleep(0.5)
        else:
            sendFirst = False
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
                self._feedback.heartbeat = True
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
        
        # if ((self._as.is_active()) and (sendRequired or sendFirst)):
        if (self._as.is_active()):
            print("sending message")
            self._as.publish_feedback(self._feedback)


    def run(self):
        print("starting Cloud actionLibserver")
        while not rospy.is_shutdown():
            if ((rospy.get_time()- self.heartBeatTime) >= 2.1):
                self.cloudConnected = False
                rospy.logwarn("lost connection to Cloud with time :{}".format(self.heartBeatTime))
                self.resetToDefault()
                if (self._as.is_active()):
                    self._as.publish_feedback(self._feedback)
                time.sleep(1)
                # rospy.logwarn("lost Connection to Cloud")
                if (self._as.is_active()):
                    self._as.set_aborted()


if __name__ == '__main__':
    server = CloudAlServer()
    server.run()
