import cv2
import argparse
import os, sys
import shutil
import time
from pathlib import Path
from faster_RCNN import get_faster_rcnn_resnet
import math

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

import matplotlib as plt
import torchvision.transforms as transforms

from torchvision.models.detection.faster_rcnn import FastRCNNPredictor

import torchvision

normalize = transforms.Normalize(
            mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
        )

transform=transforms.Compose([
            transforms.ToTensor(),
            normalize,
        ])

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

shapes = ((720, 1280), ((0.5333333333333333, 0.5), (0.0, 12.0)))
img_det_shape = (720, 1280, 3)
logger, _, _ = create_logger(
    cfg, cfg.LOG_DIR, 'demo')

device = select_device(logger,opt.device)

hFOV = 63
horizontalPixels = 640
verticalPixels = 480
degreesPerPixel = hFOV/horizontalPixels

piLitModel = torch.load("weights/piLitModel.pth")

print(type(piLitModel))

piLitModel.eval()
piLitModel.to(device)

# frame = cv2.imread("tools/metrics/004.jpg")
# while True:
#     cv2.imshow("img", frame)

#     key = cv2.waitKey(1)
#     if key == ord('q'):
#         break
frame = cv2.imread("cameraFrame.jpg")

# frame = cv2.resize(frame, (320, 640))
# frame = cv2.imread("test.jpg")

frameArray = asarray(frame)
img = transform(frameArray).to(device)

if img.ndimension() == 3:
    img = img.unsqueeze(0)

# img = img.half()

piLitPrediction = piLitModel(img)[0]

print(piLitPrediction)

for bbox in piLitPrediction["boxes"]:
    x0,y0,x1,y1 = bbox

    frame = cv2.rectangle(frame, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 3)

for bbox, score in zip(piLitPrediction["boxes"], piLitPrediction["scores"]):
        if(score > 0.9):
            print("GOT A PI LIT")    
            x0,y0,x1,y1 = bbox
            centerX = int((x0 + x1)/2)
            centerY = int((y0 + y1)/2)
            # bboxList.append(bbox)
            frame = cv2.rectangle(frame, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 3)

            angleToPiLit = math.radians((centerX - horizontalPixels/2) * degreesPerPixel)

            print("angle:", angleToPiLit)

frame = cv2.resize(frame, (640, 480))

result=cv2.imwrite(r'detected.jpg', frame)
if result==True:
    print("SAVED!")
    firstLoop = False
else:
    print("DIDN'T SAVE")

# while True:
#     cv2.imshow("img", frame)

#     key = cv2.waitKey(1)
#     if key == ord('q'):
#         break