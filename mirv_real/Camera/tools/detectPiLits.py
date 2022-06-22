#!/usr/bin/env python3
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
from std_msgs.msg import Float64


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

br = CvBridge()

normalize = transforms.Normalize(
            mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
        )

transform=transforms.Compose([
            transforms.ToTensor(),
            normalize,
        ])

detections = 0

def gotFrame(img):
    print("GOT A FRAME")
    initTime = time.time()
    frame = ros_numpy.numpify(img)
    print(frame.shape)
    tensorImg = transform(frame).to(device)
    if tensorImg.ndimension() == 3:
        tensorImg = tensorImg.unsqueeze(0)
    detections = piLitDetect(tensorImg, frame)
    # cv2.imshow("detections", detections)
    # cv2.waitKey(1)
    print("TIMEDIFF: ", time.time() - initTime)
    # cv2.destroyAllWindows()

def piLitDetect(img, frame):
    piLitPrediction = piLitModel(img)[0]
    print(piLitPrediction)
    # print(piLitPrediction["scores"])
    bboxList = []
    for bbox, score in zip(piLitPrediction["boxes"], piLitPrediction["scores"]):
        x0,y0,x1,y1 = bbox
        bboxList.append(bbox)
        frame = cv2.rectangle(frame, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 3)

        # print("Looking in predictions")

        # if(score > 0.75):
        #     frame = cv2.rectangle(frame, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 3)
        #     # print("GOT ONE")
        # elif(score > 0.5):
        #     frame = cv2.rectangle(frame, (int(x0), int(y0)), (int(x1), int(y1)), (0, 0, 255), 3)
        #     # print("MAYBE?")
    bboxMsg = Float64MultiArray()
    bboxMsg.data = bboxList
    piLitLocationPub.publish(bboxMsg)
    print("PUBLISHED LOCATIONS")
    return frame

    # cv2.imshow("frame", frame)

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
logger, _, _ = create_logger(
    cfg, cfg.LOG_DIR, 'demo')

device = select_device(logger,opt.device)
if os.path.exists(opt.save_dir):  # output dir
    shutil.rmtree(opt.save_dir)  # delete dir
os.makedirs(opt.save_dir)  # make new dir
half = device.type != 'cpu'  # half precision only supported on CUDA

piLitModel = torch.load("weights/piLitModel.pth")
piLitModel.eval()
piLitModel = piLitModel.to(device)

rospy.init_node('piLitDetector')
rospy.Subscriber("CameraFrames", Image, gotFrame)
piLitLocationPub = rospy.Publisher('piLitLocations', Float64MultiArray, queue_size=1)

try:
    rospy.spin()
except KeyboardInterrupt:
    cv2.destroyAllWindows()
