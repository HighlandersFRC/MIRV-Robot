#!/usr/bin/env python3

# Effectively Robot Main, responsible for 
# periodicly checking for faults,
# managing hierarchy of systems
# calling macros or subsystem calls

from std_msgs import String
import RoverInterface
import mirv_control.msg
import rospy


class RoverController():
    def __init__(self):
        rospy.init_node("RoverController")

        cloud_sub = rospy.Subscriber("joy", String, cloud_cb)

    def cloud_cb(msg):
        print(msg)

