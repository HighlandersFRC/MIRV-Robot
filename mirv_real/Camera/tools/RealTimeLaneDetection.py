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

# from multiprocessing import Process

class TrainParams:
    BATCH_SIZE: int = 2
    OWNER: str = "adarsh.payyakkil"  # set your name here, e.g. johndoe22
    SAVE_DIR: Optional[
        str
    ] = None  # checkpoints will be saved to cwd (current working directory)
    LOG_MODEL: bool = True   # whether to log the model to neptune after training
    GPU: Optional[int] = 1  # set to None for cpu training
    LR: float = 0.001
    PRECISION: int = 32
    CLASSES: int = 2
    SEED: int = 42
    PROJECT: str = "pi-LitDetection"
    EXPERIMENT: str = "pi-LitDetection"
    MAXEPOCHS: int = 500
    PATIENCE: int = 50
    BACKBONE: ResNetBackbones = ResNetBackbones.RESNET34
    FPN: bool = False
    ANCHOR_SIZE: Tuple[Tuple[int, ...], ...] = ((32, 64, 128, 256, 512),)
    ASPECT_RATIOS: Tuple[Tuple[float, ...]] = ((0.5, 1.0, 2.0),)
    MIN_SIZE: int = 1024
    MAX_SIZE: int = 1025
    IMG_MEAN: List = field(default_factory=lambda: [0.485, 0.456, 0.406])
    IMG_STD: List = field(default_factory=lambda: [0.229, 0.224, 0.225])
    IOU_THRESHOLD: float = 0.5

print(torch.cuda.is_available())
normalize = transforms.Normalize(
            mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
        )

transform=transforms.Compose([
            transforms.ToTensor(),
            normalize,
        ])



def detectLaneLines(cfg, opt, img):
    # frame = asarray(frame)
    # img = transform(frame).to(device)

    # frameArray = asarray(frame)
    # img = transform(frameArray).to(device)

    # if img.ndimension() == 3:
    #     img = img.unsqueeze(0)
    
    # img = img.half() if half else img.float()  # uint8 to fp16/32

    det_out, da_seg_out,ll_seg_out= model(img)

    # print(ll_seg_out)

    

    inf_out, _ = det_out

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

    img_det = show_seg_result(img, (da_seg_mask, ll_seg_mask), _, _, is_demo=True)

    return img_det

    # if len(det):
    #     det[:,:4] = scale_coords(img.shape[2:],det[:,:4],img_det.shape).round()
    #     for *xyxy,conf,cls in reversed(det):
    #         label_det_pred = f'{names[int(cls)]} {conf:.2f}'
    #         plot_one_box(xyxy, img_det , label=label_det_pred, color=colors[int(cls)], line_thickness=2)
    # cv2.imshow('image', img_det)

def testThread1():
    while True:
        print("Hello")

def testThread2():
    while True:
        print("Bye")
# Start defining a pipeline
pipeline = depthai.Pipeline()

cam_rgb = pipeline.create(depthai.node.ColorCamera)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
# cam_rgb.setIspScale(2, 3)
# cam_rgb.setPreviewSize(cameraResolutionWidth, cameraResolutionHeight)
cam_rgb.setPreviewKeepAspectRatio(True)

# manipConfig = depthai.ImageManipConfig()
# manipConfig.setCropRect(0.2, 0.2, 0, 0)

# configQueue.send(manipConfig)
# manip = pipeline.create(depthai.node.ImageManip)

# manip.setResizeThumbnail(200,200, 200, 200, 200)

xout_rgb = pipeline.create(depthai.node.XLinkOut)
configIn = pipeline.create(depthai.node.XLinkIn)
configIn.setStreamName('config')
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

controlIn = pipeline.create(depthai.node.XLinkIn)
controlIn.setStreamName('control')
controlIn.out.link(cam_rgb.inputControl)

expTime = 7500
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

def detectPiLits(frameQueue):
    while True:
        try:
            print("aaaaaaaaaaaaaaa")
            frame = frameQueue.get()
            frameArray = asarray(frame)
            tensorImg = transform(frameArray).to(device)

            if tensorImg.ndimension() == 3:
                tensorImg = tensorImg.unsqueeze(0)
            print("Inside Threaded Pi Lit Detection")
            # img = imgQueue.get(timeout = 1)
            # frame = frameQueue.get(timeout = 1)
            # piLitPrediction = piLitModel(img)[0]
            # print(piLitPrediction)
            
            # for bbox in piLitPrediction["boxes"]:
            #     x0,y0,x1,y1 = bbox

            #     frame = cv2.rectangle(frame, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 3)
            cv2.imshow("frame", frame)
        except queue.Empty:
            print("QUEUE IS EMPTY")
            pass
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

# def piLitDetect(cfg, opt, img, frame):
#     piLitPrediction = piLitModel(img)[0]
#     # print(piLitPrediction)
#     # print(piLitPrediction["scores"])
#     for bbox, score in zip(piLitPrediction["boxes"], piLitPrediction["scores"]):
#         x0,y0,x1,y1 = bbox

#         if(score > 0.75):
#             frame = cv2.rectangle(frame, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 3)
#         elif(score > 0.5):
#             frame = cv2.rectangle(frame, (int(x0), int(y0)), (int(x1), int(y1)), (0, 0, 255), 3)

#     cv2.imshow("frame", frame)

# if half:
#     piLitModel.half()

# # Load model
# model = get_net(cfg)
# checkpoint = torch.load(opt.weights, map_location= device)
# model.load_state_dict(checkpoint['state_dict'])
# model = model.to(device)
# # if half:
# #     model.half()  # to FP16

# # Set Dataloader
# if opt.source.isnumeric():
#     cudnn.benchmark = True  # set True to speed up constant image size inference
#     dataset = LoadStreams(opt.source, img_size=opt.img_size)
#     bs = len(dataset)  # batch_size
# else:
#     dataset = LoadImages(opt.source, img_size=opt.img_size)
#     bs = 1  # batch_size

# # Get names and colors
# names = model.module.names if hasattr(model, 'module') else model.names
# colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]

# img = torch.zeros((1, 3, opt.img_size, opt.img_size), device=device)  # init img
# # _ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once
# model.eval()
# model.cuda()


# piLitModel.eval()
# piLitModel.cuda()

if __name__ == '__main__':
    mp.set_start_method('spawn', force=True)
    tensorImgQueue = mp.Queue()
    frameQueue = mp.Queue()


    piLitModel = torch.load("weights/piLitModel.pth")

    print(type(piLitModel))
    piLitModel.eval()
    piLitModel = piLitModel.to(device)

    # thread1 = mp.Process(target = testThread1, args=())
    # thread2 = mp.Process(target = testThread2, args=())

    # thread1.start()
    # thread2.start()

    # thread1.join()
    # thread2.join()

    frame = cv2.imread("PXL_20220614_204008153.jpg")
    piLitThread = mp.Process(target = detectPiLits, args=(frameQueue, ))

    piLitThread.start()

    while True:
        initTime = time.time()
        in_rgb = q_rgb.tryGet()
        if in_rgb is not None:
            frame = in_rgb.getCvFrame()
            frameQueue.put(frame)
            piLitThread.join()


            # piLitDetect(cfg, opt, tensorImg, frame)

            # laneDetection = detectLaneLines(cfg, opt, tensorImg)
            # piLitDetection = detectPiLits(cfg, opt, tensorImgQueue)
            endTime = time.time()
            print("TIME DIFF: ", endTime - initTime)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break