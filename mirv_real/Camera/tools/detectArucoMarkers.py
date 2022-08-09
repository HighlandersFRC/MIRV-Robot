#!/usr/bin/env python3
from cmath import pi
import math
import queue
import threading
import cv2
import argparse
import os, sys
import shutil
import time
from pathlib import Path
from faster_RCNN import get_faster_rcnn_resnet

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

import torch
import torch.backends.cudnn as cudnn
from numpy import random
import numpy as np

from typing import Optional, List, Tuple
from dataclasses import field

from lib.config import cfg
from lib.config import update_config
from lib.utils.utils import create_logger, select_device, time_synchronized
from lib.models import get_net
from lib.dataset import LoadImages, LoadStreams
from lib.core.general import bbox_iou, non_max_suppression, scale_coords
from lib.utils import plot_one_box,show_seg_result
from lib.core.function import AverageMeter
from lib.core.postprocess import morphological_process, connect_lane
from tqdm import tqdm
import depthai

from faster_RCNN import get_faster_rcnn_resnet
from transformations import ComposeDouble
from transformations import ComposeSingle
from transformations import FunctionWrapperDouble
from transformations import FunctionWrapperSingle
from transformations import apply_nms, apply_score_threshold
from transformations import normalize_01

from backbone_resnet import ResNetBackbones
from PIL import Image

from numpy import asarray

# import fiftyone as fo

import matplotlib as plt
import torchvision.transforms as transforms

from torchvision.models.detection.faster_rcnn import FastRCNNPredictor

import torchvision

import torch.multiprocessing as mp

import roslib
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import ros_numpy
from mirv_control.msg import depth_and_color_msg as depthAndColorFrame

br = CvBridge()

normalize = transforms.Normalize(
            mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
        )

transform=transforms.Compose([
            transforms.ToTensor(),
            normalize,
        ])

detections = 0

hFOV = 63
horizontalPixels = 640
verticalPixels = 480
degreesPerPixel = hFOV/horizontalPixels

cameraMatrix = np.array([[402.62901912,   0,         312.7377781],
 [  0,         415.62202789, 225.59830764],
 [  0,           0,           1        ]])

DEPTH_SCALING_FACTOR = 1.27/0.829

distCoefficients = np.array([[-0.02239511,  0.03570026, -0.00733071, -0.00355913,  0.00224717]])

# callback function when receiving a frame
def gotFrame(data):
    print("GOT A FRAME")
    initTime = time.time()
    frame = ros_numpy.numpify(data.color_frame)
    depthFrame = ros_numpy.numpify(data.depth_frame)
    detections = detectArUcoMarkers(frame, depthFrame)
    # cv2.imshow("detections", detections)
    # cv2.waitKey(1)
    # print("TIMEDIFF: ", time.time() - initTime)
    # cv2.destroyAllWindows()

def detectArUcoMarkers(image, depthFrame):
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

    distance = 0
    angle = 0

    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2, 1))
            ret = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.09906,cameraMatrix=cameraMatrix,distCoeffs=distCoefficients)
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)

            angle = (cX - horizontalPixels/2) * degreesPerPixel
            depth = (depthFrame[cY][cX])/1000

            tvec[2] = tvec[2] * DEPTH_SCALING_FACTOR

            print("ANGLE: ", rvec, " DEPTH: ", tvec)

            if(depth != 0):
                location = [depth, angle]

                locationMsg = Float64MultiArray()
                locationMsg.data = location

                garageLocationPub.publish(locationMsg)

rospy.init_node('arUcoDetector')
rospy.Subscriber("IntakeCameraFrames", depthAndColorFrame, gotFrame)

garageLocationPub = rospy.Publisher('garageLocation', Float64MultiArray, queue_size=1)

try:
    rospy.spin()
except KeyboardInterrupt:
    cv2.destroyAllWindows()