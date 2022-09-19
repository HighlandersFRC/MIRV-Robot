#!/usr/bin/env python3
from mirv_control.msg import depth_and_color_msg as depthAndColorFrame
import ros_numpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, String
import rospy
import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
import torchvision.transforms as transforms
import matplotlib as plt
from numpy import asarray
import numpy as np
from numpy import random
import torch.backends.cudnn as cudnn
import torch
import math
from operator import truediv
import cv2
import os
import sys
import time
from faster_RCNN import get_faster_rcnn_resnet

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)


# import fiftyone as fo


rospy.init_node('piLitDetector')
br = CvBridge()

normalize = transforms.Normalize(
    mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
)

transform = transforms.Compose([
    transforms.ToTensor(),
    normalize,
])


NN_SCORE_THRESHOLD = 0.0

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
    if (cmd == "switch_right"):
        # print("RIGHT SIDE INTAKE")
        intakeSide = cmd
    elif (cmd == "switch_left"):
        # print("LEFT SIDE INTAKE")
        intakeSide = cmd


def allowNeuralNetRun(msg):
    cmd = msg.data
    global runningNeuralNetwork
    if (cmd == "piLit" or cmd == "piLitAndLanes"):
        runningNeuralNetwork = True
    else:
        runningNeuralNetwork = False


def gotFrame(data):
    #print("GOT A FRAME")
    if (runningNeuralNetwork):
        initTime = time.time()
        frame = ros_numpy.numpify(data.color_frame)
        depthFrame = ros_numpy.numpify(data.depth_frame)
        # print(frame.shape)
        piLitDetect(frame, depthFrame)


i = 0
startTime = round(time.time())


def filter_bbox(bbox):
    x0, y0, x1, y1 = bbox
    if x0 == 0 or x1 == horizontalPixels or y0 == 0 or y1 == verticalPixels:
        return False

    if abs(x1 - x0) < 30 or abs(y1 - y0) < 8:  # abs((x1 - x0) * (y1 - y0)) < 200
        return False
    return True


def piLitDetect(frame, depthFrame):
    global i

    print("DETECTING...")
    closest_track_location = None
    closest_track_distance = None
    image_dir = "/mnt/SSD/pilit_pictures"
    cv2.imwrite(f'{image_dir}/img_{startTime}_{i}.png', frame)
    print("Saved Image")
    i += 1

    transforms = ['']

    for transformation in transforms:
        print(f"USING TRANSFORM: {transformation}")
        transformed_frame = frame
        if 'h' in transformation:
            transformed_frame = cv2.flip(transformed_frame, 1)
        if 'v' in transformation:
            transformed_frame = cv2.flip(transformed_frame, 0)

        img = transform(transformed_frame).to(device)
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        prevTime = time.time()
        piLitPrediction = piLitModel(img)[0]
        print("DELTA FOR PI-LIT MODEL:", time.time() - prevTime)

        prevTime = time.time()
        for bbox, score in zip(piLitPrediction["boxes"], piLitPrediction["scores"]):
            if (score > NN_SCORE_THRESHOLD and filter_bbox(bbox)):
                depth, angleToPiLitFromIntake = processDetection(
                    bbox, depthFrame, intakeSide, transformation)

                if not closest_track_distance or depth < closest_track_distance:
                    closest_track_distance = depth
                    piLitLocation = [depth, angleToPiLitFromIntake]

                    locations = Float64MultiArray()
                    locations.data = piLitLocation
                    closest_track_location = locations

                print("DEPTH: ", depth, "ANGLE: ",
                      (angleToPiLitFromIntake), " SCORE: ", score)
        print("DELTA FOR DETECTION ANALYSIS:", time.time() - prevTime)

        if closest_track_location is not None:
            piLitLocationPub.publish(closest_track_location)
            return


def processDetection(bbox, depthFrame, intakeSide, transformation=''):
    x0, y0, x1, y1 = bbox
    if 'h' in transformation:
        x0 = horizontalPixels - x0
        x1 = horizontalPixels - x1
    if 'v' in transformation:
        y0 = verticalPixels - y0
        y1 = verticalPixels - y1
    centerX = int((x0 + x1)/2)
    centerY = int((y0 + y1)/2)
    # frame = cv2.rectangle(frame, (int(x0), int(y0)),
    #                       (int(x1), int(y1)), (0, 255, 0), 3)

    angleToPiLit = math.radians(
        (centerX - horizontalPixels/2) * degreesPerPixel)

    depth = (depthFrame[centerY][centerX])/1000

    if (intakeSide == "switch_right"):
        intakeOffset = -0.0635
    else:
        intakeOffset = 0.0635

    complementaryAngle = math.pi/2 - angleToPiLit

    horizontalOffsetToPiLit = (depth * math.cos(complementaryAngle))

    if (depth < 3 and depth != 0):

        verticalOffsetToPiLit = math.sqrt(
            (math.pow(depth, 2) - math.pow(horizontalOffsetToPiLit, 2)))

        angleToPiLitFromIntake = math.degrees(math.atan2(
            horizontalOffsetToPiLit + intakeOffset, verticalOffsetToPiLit))
    else:
        angleToPiLitFromIntake = math.degrees(angleToPiLit)

    return depth, angleToPiLitFromIntake


shapes = ((720, 1280), ((0.5333333333333333, 0.5), (0.0, 12.0)))
img_det_shape = (720, 1280, 3)
device = torch.device('cuda:0')

piLitModel = torch.load(os.path.expanduser(
    "~/mirv_ws/src/MIRV-Robot/mirv_real/Camera/weights/pi_lit_model_9.pth"))  # pi_lit_model_2, pi_lit_model_9 , map_location=torch.device('cpu')
piLitModel.eval()
piLitModel = piLitModel.to(device)

rospy.Subscriber("IntakeCameraFrames", depthAndColorFrame,
                 gotFrame, queue_size=1)
piLitLocationPub = rospy.Publisher(
    'piLitLocation', Float64MultiArray, queue_size=1)
rospy.Subscriber("intake/command", String, intakeCommandCallback)
rospy.Subscriber("neuralNetworkSelector", String, allowNeuralNetRun)

try:
    rospy.spin()
except KeyboardInterrupt:
    cv2.destroyAllWindows()
