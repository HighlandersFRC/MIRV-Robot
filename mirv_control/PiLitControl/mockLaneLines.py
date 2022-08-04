#!/usr/bin/env python3
from socket import MSG_CONFIRM
import roslib
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import ros_numpy
from mirv_description.msg import depth_and_color_msg as depthAndColorFrame

from mirv_description.msg import laneInformation

# from placement import placement
import placement

def convertToOneD(TwoDArray):
        temp = []
        for i in range(len(TwoDArray)):
            try:
                temp.append(TwoDArray[i][0])
                temp.append(TwoDArray[i][1])
            except:
                raise Exception("Invalid points entered")
        return temp

rospy.init_node("mockLaneLines")

placementList = rospy.Publisher("placementLocation", Float64MultiArray, queue_size = 5)

lat = 40.4740658
long = -104.9695155
heading = 40
lane_width = 3
lane_type = "right-lane"
placements = placement.generate_pi_lit_formation((lat, long), heading, lane_width, lane_type)
# print(placsements)
oneDimensionalPlacements = convertToOneD(placements)
msg = Float64MultiArray()
msg.data = oneDimensionalPlacements
print(type(placements))
print(msg.data)
# placementList.publish(placement.generate_pi_lit_formation((lat, long), heading, lane_width, lane_type))
while True:
    time.sleep(1)
    # print(msg)
    placementList.publish(msg)