#!/usr/bin/env python3
import math

import numpy as np
from torch import float64
import rospy
import actionlib
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import pymap3d as pm
import mirv_control.msg as ASmsg
import placement


class DetectLanes():

    def __init__(self):
        rospy.init_node('DetectLanes', anonymous=True)
        # sub = rospy.Subscriber("gps/fix", NavSatFix, self.callBack)
        # sub = rospy.Subscriber("Start/Heading", Float64, self.setStartingHeading)
        self._action_name = "PlacementLocationGenerator"
        self.result = ASmsg.DetectLanesResult()
        self._as = actionlib.SimpleActionServer(
            self._action_name, ASmsg.mirv_control.msg.DetectLanesAction, auto_start=False)
        self._as.register_goal_callback(self.execute_cb)
        self._as.start()

    def execute_cb(self):
        goal = self._as.accept_new_goal()
        latitude = goal.latitude
        longitude = goal.latitude
        heading = goal.heading
        lane_type = goal.formation_type

        placements = placement.generate_pi_lit_formation(
            (latitude, longitude), heading, lane_width, lane_type)
        # print(placsements)
        msg = Float64MultiArray()
        msg.data = placements

        self.result.placement_locations = msg
        self._as.set_succeeded(self.result)
