#!/usr/bin/env python3

from RoverInterface import RoverInterface
import rospy
import mirv_control.msg
import actionlib
from RoverInterface import RoverInterface
import threading
from PiLitController import PiLitControl as PiLitController
from std_msgs.msg import String, Float64MultiArray
import time

class roverMacros():
    def __init__(self):
        self.intake_command_pub = rospy.Publisher("intake/command", String, queue_size = 10)
        self.intake_limit_switch_sub = rospy.Subscriber("intake/limitswitches", Float64MultiArray, self.limit_switch_callback)
        self.interface = RoverInterface()
        self.limit_switches = [1, 1, 0, 0]

    def limit_switch_callback(self, switches):
        limit_switches = switches.data
        print(limit_switches)

    def placePiLit(self, timeout, intakeSide):
        start_time = time.time()
        self.intake_command_pub.publish(String("deposit"))
        self.intake_command_pub.publish(String(intakeSide))
        while not self.limit_switches[3] and not self.limit_switches[2]:
            if time.time() - start_time > timeout:
                print("Timed out - Deposit")
                return False
        time.sleep(1)
        return True

    def placeAllPiLits(self, points):
        intakeSide = "switch_right"
        for point in points:
            target = [point]
            self.interface.PP_client_goal(target)
            self.placePiLit(4, intakeSide)
            self.intake_command_pub.publish(String("reset"))

            self.interface.loadPointToSQL("deploy", intakeSide)

            if(intakeSide == "switch_right"):
                intakeSide = "switch_left"
            else:
                intakeSide = "switch_right"

    def pickupAllPiLits(self, points):
        intakeSide = "switch_right"
        for point in points:
            target = [point]
            self.interface.PP_client_goal(target)
            self.interface.pickup_client_goal(intakeSide, 5)

            self.interface.loadPointToSQL("retrieve", intakeSide)

            if(intakeSide == "switch_right"):
                intakeSide = "switch_left"
            else:
                intakeSide = "switch_right"

            
