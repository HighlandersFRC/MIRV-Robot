#!/usr/bin/env python3
import math
import cv2
import argparse
import os, sys
import time
from faster_RCNN import get_faster_rcnn_resnet

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

import torch
import torch.backends.cudnn as cudnn
from numpy import random
import numpy as np

from faster_RCNN import get_faster_rcnn_resnet

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
from mirv_description.msg import depth_and_color_msg as depthAndColorFrame

br = CvBridge()

normalize = transforms.Normalize(
            mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
        )

transform=transforms.Compose([
            transforms.ToTensor(),
            normalize,
        ])

detections = 0

hFOV = 52
horizontalPixels = 640
verticalPixels = 480
degreesPerPixel = hFOV/horizontalPixels

latestOdometryX = 0
latestOdometryY = 1
latestOdometryTheta = 0

def gotOdometry(data):
    x = data[0]
    y = data[1]
    odometry = data[2]
    updateLatestOdometry([x, y, odometry])

def updateLatestOdometry(odometry):
    latestOdometryX = odometry[0]
    latestOdometryY = odometry[1]
    latestOdometryTheta = odometry[2]

def convertPiLitLocations(relativeLocation):
    relativeX = relativeLocation[0]
    relativeY = relativeLocation[1]


    truckRelativeX = relativeX*math.cos(latestOdometryTheta) + relativeY*-math.sin(latestOdometryTheta) + latestOdometryX
    yTBody = relativeX*math.sin(latestOdometryTheta) + relativeY*math.cos(latestOdometryTheta) - yBRef*math.cos(latestOdometryTheta) + xBRef*math.sin(latestOdometryTheta)

    return [latestOdometryX, latestOdometryY]

def gotFrame(data):
    print("GOT A FRAME")
    initTime = time.time()
    frame = ros_numpy.numpify(data.color_frame)
    depthFrame = ros_numpy.numpify(data.depth_frame)
    # print(frame.shape)
    tensorImg = transform(frame).to(device)
    if tensorImg.ndimension() == 3:
        tensorImg = tensorImg.unsqueeze(0)
    piLitDetect(tensorImg, frame, depthFrame)

def piLitDetect(img, frame, depthFrame):
    piLitPrediction = piLitModel(img)[0]
    # print(piLitPrediction)
    # print(piLitPrediction["scores"])
    bboxList = []
    # print(depthFrame)
    # print(depthFrame[0][0])

    intakeSide = "RIGHT"
    
    for bbox, score in zip(piLitPrediction["boxes"], piLitPrediction["scores"]):
        if(score > 0.75):
            print("GOT A PI LIT")    
            x0,y0,x1,y1 = bbox
            centerX = int((x0 + x1)/2)
            centerY = int((y0 + y1)/2)
            bboxList.append(bbox)
            frame = cv2.rectangle(frame, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 3)

            angleToPiLit = math.radians((centerX - horizontalPixels/2) * degreesPerPixel)

            depth = (depthFrame[centerY][centerX])/1000

            if(intakeSide == "RIGHT"):
                if(angleToPiLit > 0):
                    intakeOffset = -0.0889
                else:
                    intakeOffset = 0.0889
            else:
                if(angleToPiLit > 0):
                    intakeOffset = 0.0889
                else:
                    intakeOffset = -0.0889

            complementaryAngle = 90 - angleToPiLit

            horizontalOffsetToPiLit = depth * math.cos(math.radians(complementaryAngle))

            verticalOffsetToPiLit = math.sqrt((math.pow(depth, 2) - math.pow(horizontalOffsetToPiLit, 2)))

            angleToPiLitFromIntake = math.atan2(verticalOffsetToPiLit, horizontalOffsetToPiLit + intakeOffset)

            print("DEPTH: ", depth, "ANGLE: ", angleToPiLitFromIntake)

            piLitLocation = [depth, angleToPiLitFromIntake]

            locations = Float64MultiArray()
            locations.data = piLitLocation

            piLitLocationPub.publish(locations)
    # return frame

parser = argparse.ArgumentParser()
parser.add_argument('--weights', nargs='+', type=str, default='weights/End-to-end.pth', help='model.pth path(s)')
parser.add_argument('--source', type=str, default='inference/videos', help='source')  # file/folder   ex:inference/images
parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
parser.add_argument('--conf-thres', type=float, default=0.35, help='object confidence threshold')
parser.add_argument('--iou-thres', type=float, default=0.75, help='IOU threshold for NMS')
parser.add_argument('--device', default='0', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
parser.add_argument('--save-dir', type=str, default='inference/output', help='directory to save results')
parser.add_argument('--augment', action='store_true', help='augmented inference')
parser.add_argument('--update', action='store_true', help='update all models')

api_key = "eyJhcGlfYWRkcmVzcyI6Imh0dHBzOi8vYXBwLm5lcHR1bmUuYWkiLCJhcGlfdXJsIjoiaHR0cHM6Ly9hcHAubmVwdHVuZS5haSIsImFwaV9rZXkiOiI4NjBjY2ZkZC1kMjNmLTQwM2MtYTMwNi1mMDExNDZjNWJhYjMifQ=="

opt = parser.parse_args()

mp.set_start_method('spawn', force=True)

shapes = ((720, 1280), ((0.5333333333333333, 0.5), (0.0, 12.0)))
img_det_shape = (720, 1280, 3)

device = torch.device('cuda:0')  # make new dir
half = device.type != 'cpu'  # half precision only supported on CUDA

piLitModel = torch.load("mirv_real/Camera/weights/piLitModel.pth")
piLitModel.eval()
piLitModel = piLitModel.to(device)

rospy.init_node('piLitDetector')
rospy.Subscriber("CameraFrames", depthAndColorFrame, gotFrame)
rospy.Subscriber("fusedOdometry", Float64MultiArray, gotOdometry)
piLitLocationPub = rospy.Publisher('piLitLocation', Float64MultiArray, queue_size=1)

try:
    rospy.spin()
except KeyboardInterrupt:
    cv2.destroyAllWindows()
