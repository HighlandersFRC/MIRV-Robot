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

runningNeuralNetwork = True

hFOV = 52
horizontalPixels = 640
verticalPixels = 480
degreesPerPixel = hFOV/horizontalPixels

def allowNeuralNetRun(msg):
    cmd = msg.data
    global runningNeuralNetwork
    if(cmd == "lanes" or cmd == "both"):
        runningNeuralNetwork = True
    else:
        runningNeuralNetwork = False

# callback function when receiving a frame
def gotFrame(data):
    print("GOT A FRAME")
    if(runningNeuralNetwork):
        initTime = time.time()
        frame = ros_numpy.numpify(data.color_frame)
        depthFrame = ros_numpy.numpify(data.depth_frame)
        # print(frame.shape)
        tensorImg = transform(frame).to(device)
        if tensorImg.ndimension() == 3:
            tensorImg = tensorImg.unsqueeze(0)
        detections = laneLineDetect(tensorImg, frame, depthFrame)
    # cv2.imshow("detections", detections)
    # cv2.waitKey(1)
    # print("TIMEDIFF: ", time.time() - initTime)
    # cv2.destroyAllWindows()

# detect lane lines and determine which lane MIRV is in
def laneLineDetect(img, frame, depthFrame):
    # model evaluates on the image
    det_out, da_seg_out,ll_seg_out= model(img)

    _, _, height, width = img.shape
    pad_w, pad_h = shapes[1][1]
    pad_w = int(pad_w)
    pad_h = int(pad_h)
    ratio = shapes[1][0][1]

    da_predict = da_seg_out[:, :, pad_h:(height-pad_h),pad_w:(width-pad_w)]
    da_seg_mask = torch.nn.functional.interpolate(da_predict, scale_factor=int(1/ratio), mode='bilinear')
    _, da_seg_mask = torch.max(da_seg_mask, 1)
    da_seg_mask = da_seg_mask.int().squeeze().cpu().numpy()
    # da_seg_mask = morphological_process(da_seg_mask, kernel_size=7)
    
    ll_predict = ll_seg_out[:, :,pad_h:(height-pad_h),pad_w:(width-pad_w)]
    ll_seg_mask = torch.nn.functional.interpolate(ll_predict, scale_factor=int(1/ratio), mode='bilinear')
    _, ll_seg_mask = torch.max(ll_seg_mask, 1)
    ll_seg_mask = ll_seg_mask.int().squeeze().cpu().numpy()
    # Lane line post-processing
    ll_seg_mask = morphological_process(ll_seg_mask, kernel_size=7, func_type=cv2.MORPH_OPEN)
    # ll_seg_mask = connect_lane(ll_seg_mask)
    # print("DA: ", da_seg_mask.shape)
    # print("FOUND LANE LINE MASK")

    img = cv2.resize(frame, (1280,720), interpolation=cv2.INTER_LINEAR)

    img_det = show_seg_result(img, (da_seg_mask, ll_seg_mask), _, _, is_demo=True)

    ll_seg_mask_resized = cv2.resize(ll_seg_mask, (640, 480), interpolation = cv2.INTER_LINEAR)

    # use hough lines to help determine how many lines exist
    lines = cv2.HoughLinesP(ll_seg_mask_resized,3,np.pi/180, 50, 35, 100)

    img = cv2.resize(frame, (640,480), interpolation=cv2.INTER_LINEAR)

    numLines = 0
    piLitLocations = None
    laneType = "CENTER"
    try:
        if(len(lines)%2 == 0):
            mirvLeftLaneNum = (len(lines)/2)
            mirvRightLaneNum = (len(lines)/2) + 1
        else:
            mirvLeftLaneNum = math.floor(len(lines)/2)
            mirvRightLaneNum = math.ceil(len(lines)/2)
        # print("LEFT LANE NUM: ", mirvLeftLaneNum)
        # print("RIGHT LANE NUM: ", mirvRightLaneNum)

        if(mirvLeftLaneNum > 2):
            if(len(lines) - mirvRightLaneNum > 2):
                laneType = "CENTER"
            else:
                laneType = "RIGHT"
        else:
            if(len(lines) - mirvRightLaneNum > 2):
                laneType = "CENTER"
            else:
                laneType = "LEFT"
        
        print(laneType)
    except:
        print("COULD NOT FIND LANE LINES, TRYING AGAIN")

    leftLane = lines[mirvLeftLaneNum]
    rightLane = lines[mirvRightLaneNum]

    generateLines(depthFrame, leftLane, rightLane)

    # piLitLocations = calculatePiLitPlacements(depthFrame, ll_seg_mask_resized, laneType)

    # if(piLitLocations != None):
    #     locations = Float64MultiArray()
    #     locations.data = piLitLocations
    #     placementPublisher.publish(locations)

def generateLines(depthFrame, leftLane, rightLane):
    leftLineSlope = 0
    rightLineSlope = 0
    for x1,y1,x2,y2 in leftLane:
        closeDepthLeft = depthFrame[y1][x1]
        closeLeftX = closeDepthLeft * math.cos(math.radians((x1 - horizontalPixels/2) * degreesPerPixel))
        closeLeftY = closeDepthLeft * math.sin(math.radians((x1 - horizontalPixels/2) * degreesPerPixel))

        closeLeftX = (closeLeftX * math.cos(theta)) - (closeLeftY * math.sin(theta))
        closeLeftY = (closeLeftX * math.sin(theta)) + (closeLeftY * math.cos(theta))

        farDepthLeft = depthFrame[y2][x2]
        farLeftX = farDepthLeft * math.cos(math.radians((x1 - horizontalPixels/2) * degreesPerPixel))
        farLeftY = farDepthLeft * math.sin(math.radians((x1 - horizontalPixels/2) * degreesPerPixel))

        farLeftX = (farLeftX * math.cos(theta)) - (farLeftY * math.sin(theta))
        farLeftY = (farLeftX * math.sin(theta)) + (farLeftY * math.cos(theta))

        leftLineSlope = (farLeftY - closeLeftY)/(farLeftX - closeLeftX)

        leftYIntercept = (-(leftLineSlope) * closeLeftX) + closeleftY

        leftSideLaneWidth = -leftYIntercept/leftLineSlope

    for x1,y1,x2,y2 in rightLane:
        closeDepthRight = depthFrame[y1][x1]
        closeRightX = closeDepthRight * math.cos(math.radians((x1 - horizontalPixels/2) * degreesPerPixel))
        closeRightY = closeDepthRight * math.sin(math.radians((x1 - horizontalPixels/2) * degreesPerPixel))

        closeRightX = (closeRightX * math.cos(theta)) - (closeRightY * math.sin(theta))
        closeRightY = (closeRightX * math.sin(theta)) + (closeRightY * math.cos(theta))

        farDepthRight = depthFrame[y2][x2]
        farRightX = farDepthRight * math.cos(math.radians((x1 - horizontalPixels/2) * degreesPerPixel))
        farRightY = farDepthRight * math.sin(math.radians((x1 - horizontalPixels/2) * degreesPerPixel))

        farRightX = (farRightX * math.cos(theta)) - (farRightY * math.sin(theta))
        farRightY = (farRightX * math.sin(theta)) + (farRightY * math.cos(theta))

        rightLineSlope = (farRightY - closeRightY)/(farRightX - closeRightX)
        
        rightYIntercept = (-(rightLineSlope) * closeRightX) + closeRightY

        rightSideLaneWidth = rightYIntercept/rightLineSlope

    laneWidth = leftSideLaneWidth + rightSideLaneWidth

    boundingInformation = [leftLineSlope, leftYIntercept, rightLineSlope, rightYIntercept]

    boundingMsg = Float64MultiArray()
    boundingMsg.data = boundingInformation

    laneBoundPublisher.publish(boundingMsg)

# # based on which lane, and width of lane, determine locations pi lits should be placed
# def calculatePiLitPlacements(depthFrame, laneLineMask, laneType):
#     i = 0
#     laneLineDepthLeft = 0
#     laneLineDepthRight = 0
#     laneLineAngleInFrameLeft = 0
#     laneLineAngleInFrameRight = 0
#     for row in laneLineMask:
#         nonZero = np.nonzero(row)
#         for column in nonZero:
#             for pixel in column:
#                 laneLineDepthLeft = (depthFrame[i][pixel])/1000
#                 laneLineAngleInFrameLeft = math.radians((pixel - horizontalPixels/2) * degreesPerPixel)
#                 # print("Left Depth: ", laneLineDepthLeft)
#                 if(laneLineDepthLeft != 0):
#                     break
#             if(laneLineDepthLeft != 0):
#                 break
#             for pixel in reversed(column):
#                 laneLineDepthRight = (depthFrame[i][pixel])/1000
#                 laneLineAngleInFrameRight = math.radians((pixel - horizontalPixels/2) * degreesPerPixel)
#                 # print("Right Depth: ", laneLineDepthRight)
#                 if(laneLineDepthRight != 0):
#                     break
#             if(laneLineDepthRight != 0 and laneLineDepthLeft != 0):
#                 break
#         if(laneLineDepthRight != 0 and laneLineDepthLeft != 0):
#             break
#         i += 1

#     if(laneLineDepthLeft != 0 or laneLineDepthRight != 0):    
#         laneOffsetFromCenterLeft = laneLineDepthLeft * math.sin(laneLineAngleInFrameLeft)
#         laneOffsetFromCenterRight = laneLineDepthRight * math.sin(laneLineAngleInFrameRight)

#         depthToFarthestPoint = 237.74

#         laneWidth = laneOffsetFromCenterLeft + laneOffsetFromCenterRight
#         piLitPlacementLineLength = math.sqrt(math.pow(depthToFarthestPoint, 2) + math.pow(laneWidth, 2))

#         # print("LINE LENGTH: ", piLitPlacementLineLength, " LANE WIDTH: ", laneWidth)

#         piLitIntervalX = laneWidth/7
#         lineSlope = piLitPlacementLineLength/laneWidth

#         piLitPlacementList = []

#         yTruckOffset = 3.048 #meters, converts to 10 ft

#         if(laneType == "LEFT"):
#             # 8 loop throughs for each pi lit
#             for i in range(8):
#                 piLitLocationX = laneWidth - (i * piLitIntervalX)
#                 piLitLocationY = (-lineSlope * (piLitLocationX)) + depthToFarthestPoint

#                 piLitPlacementList.append([piLitLocationX, piLitLocationY])        
#         elif(laneType == "RIGHT"):
#             # 8 loop throughs for each pi lit
#             for i in range(8):
#                 piLitLocationX = i * piLitIntervalX
#                 piLitLocationY = (lineSlope * (piLitLocationX))

#                 piLitPlacementList.append([piLitLocationX, piLitLocationY])
#         elif(laneType == "CENTER"):
#             spearSidesDistance = depthToFarthestPoint/2
#             lineSlope = ((depthToFarthestPoint - spearSidesDistance)/laneWidth)
#             for i in range(4):
#                 piLitLocationX = i * piLitIntervalX
#                 piLitLocationY = (lineSlope * piLitLocationX) + spearSidesDistance

#                 piLitPlacementList.append([piLitLocationX, piLitLocationY])
#             for i in range(4, 8):
#                 piLitLocationX = i * piLitIntervalX
#                 piLitLocationY = (-lineSlope * (piLitLocationX)) + depthToFarthestPoint
            
#                 piLitPlacementList.append([piLitLocationX, piLitLocationY])
#         theta = 0

#         for i in range(len(piLitPlacementList)):
#             location = piLitPlacementList[i]
#             x = (location[0] * math.cos(theta)) - (y * math.sin(theta))
#             y = (location[0] * math.sin(theta)) + (y * math.cos(theta))
#             piLitPlacementList[i] = [x - laneOffsetFromCenterLeft, y + yTruckOffset]
        
#         print(piLitPlacementList)

#         return piLitPlacementList
#     else:
#         return None

shapes = ((720, 1280), ((0.5333333333333333, 0.5), (0.0, 12.0)))
img_det_shape = (720, 1280, 3)

device = torch.device('cuda:0')
half = device.type != 'cpu'  # half precision only supported on CUDA

# Load model
model = get_net(cfg)

model.eval()
model.cuda()

print("loaded model")

rospy.init_node('laneLineDetector')
rospy.Subscriber("IntakeCameraFrames", depthAndColorFrame, gotFrame)

placementPublisher = rospy.Publisher('pathingPointInput', Float64MultiArray, queue_size=1)

laneBoundPublisher = rospy.Publisher("laneBound", Float64MultiArray, queue_size = 1)
rospy.Subscriber("neuralNetworkSelector", String, allowNeuralNetRun)

try:
    rospy.spin()
except KeyboardInterrupt:
    cv2.destroyAllWindows()