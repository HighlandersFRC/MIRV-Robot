#!/usr/bin/env python3
import math
from operator import truediv
import cv2
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

import rospy
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import ros_numpy
from mirv_control.msg import depth_and_color_msg as depthAndColorFrame

rospy.init_node('piLitDetector')
br = CvBridge()

normalize = transforms.Normalize(
            mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
        )

transform=transforms.Compose([
            transforms.ToTensor(),
            normalize,
        ])

intakeSide = "switch_right"

runningNeuralNetwork = True
# global intakeSide 
# intakeSide = "switch_right"

detections = 0

hFOV = 63
horizontalPixels = 640
verticalPixels = 480
degreesPerPixel = hFOV/horizontalPixels

def intakeCommandCallback(msg):
    cmd = msg.data
    global intakeSide
    if(cmd == "switch_right"):
        # print("RIGHT SIDE INTAKE")
        intakeSide = cmd
    elif(cmd == "switch_left"):
        # print("LEFT SIDE INTAKE")
        intakeSide = cmd

def allowNeuralNetRun(msg):
    cmd = msg.data
    global runningNeuralNetwork
    if(cmd == "piLit" or cmd == "piLitAndLanes"):
        runningNeuralNetwork = True
    else:
        runningNeuralNetwork = False

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
        piLitDetect(tensorImg, frame, depthFrame)

def piLitDetect(img, frame, depthFrame):
    piLitPrediction = piLitModel(img)[0]
    bboxList = []
    print("DETECTING...")
    
    for bbox, score in zip(piLitPrediction["boxes"], piLitPrediction["scores"]):
        if(score > 0.5):
            # print("GOT A PI LIT")
            print(intakeSide)    
            x0,y0,x1,y1 = bbox
            centerX = int((x0 + x1)/2)
            centerY = int((y0 + y1)/2)
            bboxList.append(bbox)
            frame = cv2.rectangle(frame, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 3)

            angleToPiLit = math.radians((centerX - horizontalPixels/2) * degreesPerPixel)

            depth = (depthFrame[centerY][centerX])/1000

            if(intakeSide == "switch_right"):
                intakeOffset = -0.0635
            else:
                intakeOffset = 0.0635

            complementaryAngle = math.pi/2 - angleToPiLit

            horizontalOffsetToPiLit = (depth * math.cos(complementaryAngle))

            verticalOffsetToPiLit = math.sqrt((math.pow(depth, 2) - math.pow(horizontalOffsetToPiLit, 2)))

            if(depth < 3 and depth != 0):
                # angleToPiLitFromIntake = math.degrees(angleToPiLit)
                angleToPiLitFromIntake = math.degrees(math.atan2(horizontalOffsetToPiLit + intakeOffset, verticalOffsetToPiLit))
                piLitLocation = [depth, angleToPiLitFromIntake]

                locations = Float64MultiArray()
                locations.data = piLitLocation

                piLitLocationPub.publish(locations)
            else:
                angleToPiLitFromIntake = math.degrees(angleToPiLit)
            print("DEPTH: ", depth, " ORIGINAL ANGLE: ", math.degrees(angleToPiLit), "ANGLE: ", (angleToPiLitFromIntake), " SCORE: ", score)

shapes = ((720, 1280), ((0.5333333333333333, 0.5), (0.0, 12.0)))
img_det_shape = (720, 1280, 3)

device = torch.device('cuda:0')
half = device.type != 'cpu'

piLitModel = torch.load(os.path.expanduser("~/mirv_ws/src/MIRV-Robot/mirv_real/Camera/weights/piLitModel.pth"))
piLitModel.eval()
piLitModel = piLitModel.to(device)

rospy.Subscriber("IntakeCameraFrames", depthAndColorFrame, gotFrame)
piLitLocationPub = rospy.Publisher('piLitLocation', Float64MultiArray, queue_size=1)
rospy.Subscriber("intake/command", String, intakeCommandCallback)
rospy.Subscriber("neuralNetworkSelector", String, allowNeuralNetRun)

try:
    rospy.spin()
except KeyboardInterrupt:
    cv2.destroyAllWindows()