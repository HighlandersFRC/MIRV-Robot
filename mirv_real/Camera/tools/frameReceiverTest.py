#!/usr/bin/env python3
import cv2
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
# from rospy_tutorials.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from custom_msg_python.msg import depth_and_color_msg as depthAndColorFrame
import ros_numpy

def gotFrames(data):
    print("GOT A FRAME")
    depthFrame = ros_numpy.numpify(data.depth_frame)
    colorFrame = ros_numpy.numpify(data.color_frame)

    if cv2.waitKey(1) == ord('q'):
        cv2.destroyAllWindows
    else:
        cv2.imshow("depth", depthFrame)
        cv2.imshow("color", colorFrame)
    


rospy.init_node('piLitDetector')
rospy.Subscriber("CameraFrames", depthAndColorFrame, gotFrames)

rospy.spin()