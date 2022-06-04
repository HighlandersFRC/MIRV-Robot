#!/usr/bin/env python2
from __future__ import print_function
import roslib
import sys
import rospy
import numpy as np
import datetime
import time
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from std_msgs msg import Int64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation as R
import tf_conversions
import tf2_ros
import message_filters
import copy
from threading import Thread, Lock

roslib.load_manifest('mirv_simulation')

class wheel_encoder:

    # Define initial/setup values
    def __init__(self):

        # Get parameters from launch file
        # <xacro:sensor_rotary_encoder ns="/" parent="wheel_fl" joint="wheel_fl_joint" d="${wheel_radius}" topic="/wheel_fl_encoder" update_rate="100.0" ppr="150"/>
        self.topic_name = rospy.get_param('~topic')
        self.joint_name = rospy.get_param('~joint')
        self.update_rate = rospy.get_param('~update_rate', 100)
        self.pulse_per_rev = rospy.get_param('~ppr', 150)

        self.timer = rospy.Rate(self.update_rate)
        self.current_tick_count = 0

        # self.${publisher_name} = rospy.Publisher(${string topic_name}, ${message_type}, ${other_parameters...})
        self.tick_pub = rospy.Publisher(self.topic_name, Int64, queue_size=5)
    
        # self.${subscriber_name} = rospy.Subscriber(${string topic_name}, ${message_type}, ${self.callback_function}, ${other_parameters...})

    # Call this function at the update rate
    # Compute the difference between the current wheel angle and the last wheel angle (assumes that we update faster than the wheel accelerates)
    # That angle difference * the number of pulses per rev / 2*pi radians = tick delta
    # Add that delta to the running count, publish the new running count
    def get_and_send_tick_count(self):


        new_ticks = 0
        self.current_tick_count += new_ticks
        self.tick_pub.Publish(self.current_tick_count)


def main(args):
    rospy.init_node('encoder', anonymous=True)
    encoder = wheel_encoder()
    try:
        while True:
            encoder.timer.sleep()
            encoder.get_and_send_tick_count()
    
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)