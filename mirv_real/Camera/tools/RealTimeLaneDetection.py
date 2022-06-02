import cv2
import argparse
import os, sys
import shutil
import time
from pathlib import Path
import imageio

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

import torch
import torch.backends.cudnn as cudnn
from numpy import random
import scipy.special
import numpy as np
import PIL.Image as image

from lib.config import cfg
from lib.config import update_config
from lib.utils.utils import create_logger, select_device, time_synchronized
from lib.models import get_net
from lib.dataset import LoadImages, LoadStreams
from lib.core.general import non_max_suppression, scale_coords
from lib.utils import plot_one_box,show_seg_result
from lib.core.function import AverageMeter
from lib.core.postprocess import morphological_process, connect_lane
from tqdm import tqdm
import depthai

from numpy import asarray

import matplotlib as plt
import torchvision.transforms as transforms



print(torch.cuda.is_available())
normalize = transforms.Normalize(
            mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
        )

transform=transforms.Compose([
            transforms.ToTensor(),
            normalize,
        ])


def detectRealTime(cfg, opt, frame):
    frame = asarray(frame)
    img = transform(frame).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32

    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    det_out, da_seg_out,ll_seg_out= model(img)
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



# Start defining a pipeline
pipeline = depthai.Pipeline()

cam_rgb = pipeline.create(depthai.node.ColorCamera)
cam_rgb.setPreviewSize(640, 384)
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
opt = parser.parse_args()

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
if half:
    model.half()  # to FP16

# Set Dataloader
if opt.source.isnumeric():
    cudnn.benchmark = True  # set True to speed up constant image size inference
    dataset = LoadStreams(opt.source, img_size=opt.img_size)
    bs = len(dataset)  # batch_size
else:
    dataset = LoadImages(opt.source, img_size=opt.img_size)
    bs = 1  # batch_size

# Get names and colors
names = model.module.names if hasattr(model, 'module') else model.names
colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]

img = torch.zeros((1, 3, opt.img_size, opt.img_size), device=device)  # init img
_ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once
model.eval()
model.cuda()

# engine, context = build_engine('./weights/yolop-640-640.onnx')

while True:
    initTime = time.time()
    in_rgb = q_rgb.tryGet()
    if in_rgb is not None:
        frame = in_rgb.getCvFrame()
        # cv2.imshow("detection", frame)
        # cv2.imshow("depth", depthFrameColor)
        # print("---------------")
        detection = detectRealTime(cfg, opt, frame)
        endTime = time.time()
        print("TIME DIFF: ", endTime - initTime)
        print("SHOWING FRAME")
        cv2.imshow("faksdljf", detection)


    # newConfig = False
    key = cv2.waitKey(1)
    if key == ord('q'):
        break