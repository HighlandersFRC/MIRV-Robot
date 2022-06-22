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
    detections = laneLineDetect(tensorImg, frame)
    # cv2.imshow("detections", detections)
    # cv2.waitKey(1)
    print("TIMEDIFF: ", time.time() - initTime)
    # cv2.destroyAllWindows()

def laneLineDetect(img, frame):
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
    #ll_seg_mask = morphological_process(ll_seg_mask, kernel_size=7, func_type=cv2.MORPH_OPEN)
    #ll_seg_mask = connect_lane(ll_seg_mask)
    # print("DA: ", da_seg_mask.shape)
    # print("FOUND LANE LINE MASK")

    img = cv2.resize(frame, (1280,720), interpolation=cv2.INTER_LINEAR)

    # print("RESIZED IMAGE")

    # img_det = show_seg_result(img, (da_seg_mask, ll_seg_mask), _, _,)

    # return img_det

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

# Load model
model = get_net(cfg)
checkpoint = torch.load(opt.weights, map_location= device)
model.load_state_dict(checkpoint['state_dict'])
model = model.to(device)

model.eval()
model.cuda()

rospy.init_node('laneLineDetector')
rospy.Subscriber("CameraFrames", Image, gotFrame)

try:
    rospy.spin()
except KeyboardInterrupt:
    cv2.destroyAllWindows()
