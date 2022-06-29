#!/usr/bin/env python3
import math
import queue
import threading
import cv2
import argparse
import os, sys
import shutil
import time
from pathlib import Path
# from faster_RCNN import get_faster_rcnn_resnet

# BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# sys.path.append(BASE_DIR)

# import torch
# import torch.backends.cudnn as cudnn
# from numpy import random
# import numpy as np

# from typing import Optional, List, Tuple
# from dataclasses import field

# from lib.config import cfg
# from lib.config import update_config
# from lib.utils.utils import create_logger, select_device, time_synchronized
# from lib.models import get_net
# from lib.dataset import LoadImages, LoadStreams
# from lib.core.general import bbox_iou, non_max_suppression, scale_coords
# from lib.utils import plot_one_box,show_seg_result
# from lib.core.function import AverageMeter
# from lib.core.postprocess import morphological_process, connect_lane
# from tqdm import tqdm
import depthai

# from faster_RCNN import get_faster_rcnn_resnet
# from transformations import ComposeDouble
# from transformations import ComposeSingle
# from transformations import FunctionWrapperDouble
# from transformations import FunctionWrapperSingle
# from transformations import apply_nms, apply_score_threshold
# from transformations import normalize_01

# from backbone_resnet import ResNetBackbones
# from PIL import Image

from numpy import asarray

# import fiftyone as fo

# import matplotlib as plt
# import torchvision.transforms as transforms

# from torchvision.models.detection.faster_rcnn import FastRCNNPredictor

# import torchvision

# import torch.multiprocessing as mp
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
# from rospy_tutorials.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from mirv_description.msg import depth_and_color_msg as depthAndColorFrame
from cv_bridge import CvBridge

def quat_2_radians(x, y, z, w):
    pitch = math.atan2(2*x*w - 2*y*z, 1-2*x*x - 2* z*z)
    yaw = math.asin(2*x*y + 2*z*w)
    roll = math.atan2(2*x*w - 2*x*z, 1-2*y*y - 2*z*z)
    return pitch, yaw, roll

cameraX = 640
cameraY = 480

def gotPiLitLocations(data):
    locations = data.data
    print("GOT A LOCATION")
    print(data)
    # for location in location:
    #     topLeftX = (location[0])/(cameraX)
    #     topLeftY = (location[1])/(cameraY)
    #     bottomRightX = (location[2])/(cameraX)
    #     bottomRightY = (location[3])/(cameraY)
    #     print(location)



# def publishCameraInformation(fjadksljf):
    # initTime = time.time()
    # in_rgb = q_rgb.tryGet()

    # imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived
    # inDepth = depthQueue.get()

    # imuPackets = imuData.packets
    # for imuPacket in imuPackets:
    #     rVvalues = imuPacket.rotationVector

    #     rotationI = rVvalues.i
    #     rotationJ = rVvalues.j
    #     rotationK = rVvalues.k
    #     rotationReal = rVvalues.real
        
    #     pitch, yaw, roll = quat_2_radians(rotationI, rotationJ, rotationK, rotationReal)

    #     pitch = pitch * 180/math.pi
    #     yaw = yaw * 180/math.pi
    #     roll = roll * 180/math.pi


    #     if(pitch < 0):
    #         pitch = abs(pitch)
    #     elif(pitch > 0):
    #         pitch = 360 - pitch

    #     if(pitch > 90):
    #         pitch = pitch - 90
    #     else:
    #         pitch = pitch + 270

    #     # print("PITCH: ", pitch)

    #     imuPub.publish(pitch)
    #     # rate.sleep()

    # if in_rgb is not None:
    #     print("IN RGB IS NOT NONE")
    #     frame = in_rgb.getCvFrame()
    #     print("got rgb frame")
    #     depthFrame = inDepth.getFrame() # depthFrame values are in millimeters

    #     depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
    #     depthFrameColor = cv2.equalizeHist(depthFrameColor)
    #     depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_COOL)
    #     print("ajdlkfjalsdkjf")
    #     cv2.imshow("depth", depthFrameColor)
    #     cv2.waitKey(1)
    #     # frameArray = Float64MultiArray()
    #     # frameArray.data = asarray(frame)

    #     # frameArray = asarray(frame)
    #     imgPub.publish(br.cv2_to_imgmsg(frame))
    #     endTime = time.time()
    #     print("TIME DIFF: ", endTime - initTime)

br = CvBridge()

# imgPub = rospy.Publisher('CameraFrames', numpy_msg(Floats),queue_size=10)
imgPub = rospy.Publisher('CameraFrames', depthAndColorFrame, queue_size=1)
rospy.init_node('talker', anonymous=True)

imuPub = rospy.Publisher('CameraIMU', Float64, queue_size=1)
rospy.init_node('talker', anonymous=True)

# rospy.init_node('piLitLocationSubscriber', anonymous=True)
# rospy.Subscriber("piLitLocations", Float64MultiArray, gotPiLitLocations)

pipeline = depthai.Pipeline()

cam_rgb = pipeline.create(depthai.node.ColorCamera)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
# cam_rgb.setIspScale(2, 3)
# cam_rgb.setPreviewSize(cameraResolutionWidth, cameraResolutionHeight)
cam_rgb.setPreviewKeepAspectRatio(True)

imu = pipeline.createIMU()
xlinkOut = pipeline.createXLinkOut()
xlinkOut.setStreamName("imu")
imu.enableIMUSensor(depthai.IMUSensor.ROTATION_VECTOR, 400)
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)
imu.out.link(xlinkOut.input)

xout_rgb = pipeline.create(depthai.node.XLinkOut)
configIn = pipeline.create(depthai.node.XLinkIn)
configIn.setStreamName('config')
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

controlIn = pipeline.create(depthai.node.XLinkIn)
controlIn.setStreamName('control')
controlIn.out.link(cam_rgb.inputControl)

# Define a source - two mono (grayscale) cameras
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()
stereo = pipeline.createStereoDepth()
spatialLocationCalculator = pipeline.createSpatialLocationCalculator()

xoutDepth = pipeline.createXLinkOut()
xoutSpatialData = pipeline.createXLinkOut()
xinSpatialCalcConfig = pipeline.createXLinkIn()

xoutDepth.setStreamName("depth")
xoutSpatialData.setStreamName("spatialData")
xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

# MonoCamera
monoLeft.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_720_P)
monoLeft.setBoardSocket(depthai.CameraBoardSocket.LEFT)
monoRight.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_720_P)
monoRight.setBoardSocket(depthai.CameraBoardSocket.RIGHT)

outputDepth = True
outputRectified = False
lrcheck = True
subpixel = False
extended = True

# StereoDepth
stereo.setOutputDepth(outputDepth)
stereo.setOutputRectified(outputRectified)
stereo.setConfidenceThreshold(255)
stereo.setDepthAlign(depthai.CameraBoardSocket.RGB)
# setRectifyEdgeFillColor(-1)
stereo.setOutputSize(640, 480)

stereo.setLeftRightCheck(lrcheck)
stereo.setSubpixel(subpixel)
# stereo.setExtendedDisparity(extended)

monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

# spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
stereo.depth.link(xoutDepth.input)

topLeft = depthai.Point2f(0.4, 0.4)
bottomRight = depthai.Point2f(0.6, 0.6)

spatialLocationCalculator.setWaitForConfigInput(False)
config = depthai.SpatialLocationCalculatorConfigData()
config.depthThresholds.lowerThreshold = 0
config.depthThresholds.upperThreshold = 10000
config.roi = depthai.Rect(topLeft, bottomRight)
spatialLocationCalculator.initialConfig.addROI(config)
spatialLocationCalculator.out.link(xoutSpatialData.input)
xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

expTime = 7000
sensIso = 500

depthaiDevice = depthai.Device(pipeline)
depthaiDevice.startPipeline()

q_rgb = depthaiDevice.getOutputQueue("rgb", maxSize=1, blocking=False)
depthQueue = depthaiDevice.getOutputQueue(name="depth", maxSize=1, blocking=False)
spatialCalcQueue = depthaiDevice.getOutputQueue(name="spatialData", maxSize=1, blocking=False)
spatialCalcConfigInQueue = depthaiDevice.getInputQueue("spatialCalcConfig")
frame = None

imuQueue = depthaiDevice.getOutputQueue(name="imu", maxSize=1, blocking=False)
disparityMultiplier = 255 / stereo.getMaxDisparity()

controlQueue = depthaiDevice.getInputQueue('control')
ctrl = depthai.CameraControl()
ctrl.setManualExposure(expTime, sensIso)
# ctrl.setAutoFocusMode(depthai.CameraControl.AutoFocusMode.AUTO)
ctrl.setAutoFocusMode(depthai.CameraControl.AutoFocusMode.AUTO)
# ctrl.setAutoFocusMode(depthai.RawCameraControl.AutoFocusMode.ON)
# ctrl.setManualFocus(0)
controlQueue.send(ctrl)

while True:
    initTime = time.time()
    in_rgb = q_rgb.tryGet()

    imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived
    inDepth = depthQueue.get()
    print("PAST QUEUE GET")
    depthFrame = inDepth.getFrame()
    # depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
    # depthFrameColor = cv2.equalizeHist(depthFrame)
    # depthFrameColor = cv2.applyColorMap(depthFrame, cv2.COLORMAP_OCEAN)

    imuPackets = imuData.packets
    for imuPacket in imuPackets:
        rVvalues = imuPacket.rotationVector

        rotationI = rVvalues.i
        rotationJ = rVvalues.j
        rotationK = rVvalues.k
        rotationReal = rVvalues.real
        
        pitch, yaw, roll = quat_2_radians(rotationI, rotationJ, rotationK, rotationReal)

        pitch = pitch * 180/math.pi
        yaw = yaw * 180/math.pi
        roll = roll * 180/math.pi


        if(pitch < 0):
            pitch = abs(pitch)
        elif(pitch > 0):
            pitch = 360 - pitch

        if(pitch > 90):
            pitch = pitch - 90
        else:
            pitch = pitch + 270

        # print("PITCH: ", pitch)

        imuPub.publish(pitch)
        # rate.sleep()

    if in_rgb is not None and inDepth is not None:
        # print("IN RGB IS NOT NONE")
        frame = in_rgb.getCvFrame()
        framesMessage = depthAndColorFrame()
        framesMessage.depth_frame = br.cv2_to_imgmsg(depthFrame)
        framesMessage.color_frame = br.cv2_to_imgmsg(frame)
        # print(framesMessage)
        imgPub.publish(framesMessage)
        endTime = time.time()
        print(depthFrame.shape)
        # print("TIME DIFF: ", endTime - initTime)
    
        cv2.imshow("frame", frame)
        cv2.imshow("depth", depthFrame)
    
    if cv2.waitKey(1) == ord('q'):
            break
