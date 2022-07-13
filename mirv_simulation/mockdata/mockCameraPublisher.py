#!/usr/bin/env python
import math
import cv2
import os, sys
import rospy
import numpy as np
from numpy import asarray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

from cv_bridge import CvBridge

#BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
#sys.path.append(BASE_DIR+'/../')
#print(sys.path)


from mirv_description.msg import depth_and_color_msg as depthAndColorFrame
br = CvBridge()

# imgPub = rospy.Publisher('CameraFrames', numpy_msg(Floats),queue_size=10)
imgPub = rospy.Publisher('CameraFrames', depthAndColorFrame, queue_size=1)
rospy.init_node('CameraPublisher', anonymous=True)

imuPub = rospy.Publisher('CameraIMU', Float64, queue_size=1)

pitch = 0

#cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
height = 480
width = 640

while True:
    
    pitch = (pitch +1) %360
    imuPub.publish(pitch)

    blank_image = np.zeros((height,width,3), np.uint8)
    blank_image[:,0:width//2] = (255,0,0)      # (B, G, R)
    blank_image[:,width//2:width] = (0,255,0)

    frame = cv2.cvtColor(blank_image, cv2.COLOR_RGB2BGR)

    framesMessage = depthAndColorFrame()
    framesMessage.depth_frame = br.cv2_to_imgmsg(frame)
    framesMessage.color_frame = br.cv2_to_imgmsg(frame)
    imgPub.publish(framesMessage)




        
    
    if cv2.waitKey(1) == ord('q'):
            break
