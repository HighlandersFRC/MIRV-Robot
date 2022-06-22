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
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
# from rospy_tutorials.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def quat_2_radians(x, y, z, w):
    pitch = math.atan2(2*x*w - 2*y*z, 1-2*x*x - 2* z*z)
    yaw = math.asin(2*x*y + 2*z*w)
    roll = math.atan2(2*x*w - 2*x*z, 1-2*y*y - 2*z*z)
    return pitch, yaw, roll

def gotPiLitLocations(locations):
    print("GOT A LOCATION")
    print(locations)

def publishCameraInformation(fjadksljf):
    initTime = time.time()
    in_rgb = q_rgb.tryGet()

    imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived

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

    if in_rgb is not None:
        frame = in_rgb.getCvFrame()
        # frameArray = Float64MultiArray()
        # frameArray.data = asarray(frame)

        # frameArray = asarray(frame)
        imgPub.publish(br.cv2_to_imgmsg(frame))
        endTime = time.time()
        print("TIME DIFF: ", endTime - initTime)

br = CvBridge()

# imgPub = rospy.Publisher('CameraFrames', numpy_msg(Floats),queue_size=10)
imgPub = rospy.Publisher('CameraFrames', Image, queue_size=1)
rospy.init_node('talker', anonymous=True)

imuPub = rospy.Publisher('CameraIMU', Float64, queue_size=1)
rospy.init_node('talker', anonymous=True)

# rospy.init_node('piLitLocationSubscriber')
rospy.Subscriber("piLitLocations", Float64MultiArray, gotPiLitLocations)

pipeline = depthai.Pipeline()

cam_rgb = pipeline.create(depthai.node.ColorCamera)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
# cam_rgb.setIspScale(2, 3)
# cam_rgb.setPreviewSize(cameraResolutionWidth, cameraResolutionHeight)
cam_rgb.setPreviewKeepAspectRatio(True)

monoLeft = pipeline.create(depthai.node.MonoCamera)
monoRight = pipeline.create(depthai.node.MonoCamera)
stereo = pipeline.create(depthai.node.StereoDepth)
spatialLocationCalculator = pipeline.create(depthai.node.SpatialLocationCalculator)

xoutDepth = pipeline.create(depthai.node.XLinkOut)
xoutSpatialData = pipeline.create(depthai.node.XLinkOut)
xinSpatialCalcConfig = pipeline.create(depthai.node.XLinkIn)

xoutDepth.setStreamName("depth")
xoutSpatialData.setStreamName("spatialData")
xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

# Properties
monoLeft.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(depthai.CameraBoardSocket.LEFT)
monoRight.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(depthai.CameraBoardSocket.RIGHT)

lrcheck = False
subpixel = False

stereo.setDefaultProfilePreset(depthai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setLeftRightCheck(lrcheck)
stereo.setSubpixel(subpixel)

# Config
topLeft = depthai.Point2f(0.4, 0.4)
bottomRight = depthai.Point2f(0.6, 0.6)

config = depthai.SpatialLocationCalculatorConfigData()
config.depthThresholds.lowerThreshold = 100
config.depthThresholds.upperThreshold = 10000
config.roi = depthai.Rect(topLeft, bottomRight)

spatialLocationCalculator.inputConfig.setWaitForMessage(False)
spatialLocationCalculator.initialConfig.addROI(config)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
stereo.depth.link(spatialLocationCalculator.inputDepth)

spatialLocationCalculator.out.link(xoutSpatialData.input)
xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

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


expTime = 5000
sensIso = 500

depthaiDevice = depthai.Device(pipeline)
depthaiDevice.startPipeline()

q_rgb = depthaiDevice.getOutputQueue("rgb", maxSize=1, blocking=False)
frame = None

controlQueue = depthaiDevice.getInputQueue('control')
ctrl = depthai.CameraControl()
ctrl.setManualExposure(expTime, sensIso)
ctrl.setAutoFocusMode(depthai.CameraControl.AutoFocusMode.AUTO)
# ctrl.setAutoFocusMode(depthai.RawCameraControl.AutoFocusMode.ON)
ctrl.setManualFocus(0)
controlQueue.send(ctrl)

imuQueue = depthaiDevice.getOutputQueue(name="imu", maxSize=50, blocking=False)

timer = rospy.Timer(rospy.Duration(0.03), publishCameraInformation)

# # while True:
    
    
rospy.spin()

# while True:
    

    # key = cv2.waitKey(1)
    # if key == ord('q'):
    #     break