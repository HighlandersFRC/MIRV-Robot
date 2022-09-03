#!/usr/bin/env python3
import placement
from std_msgs.msg import Float64
import actionlib
import mirv_control.msg as ASmsg
from mirv_control.msg import depth_and_color_msg as depthAndColorFrame
from mirv_control.msg import single_lane_detection
from mirv_control.msg import lane_detections
from mirv_control.msg import DetectLanesResult
import ros_numpy
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray, String
import rospy
import torchvision.transforms as transforms
from numpy import asarray
import depthai

from lib.core.postprocess import morphological_process, connect_lane
from lib.utils import show_seg_result
from lib.models import get_net
from lib.config import cfg
import numpy as np
import torch
from cmath import pi
import math
import cv2
import os
import sys
import time
import pixel_angles

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)


# import fiftyone as fo


br = CvBridge()

# os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

normalize = transforms.Normalize(
    mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
)

transform = transforms.Compose([
    transforms.ToTensor(),
    normalize,
])

img_scale = 0.5

img_height = 480/img_scale
img_width = 640/img_scale

# TODO: verify this value
shapes = ((img_height, img_width),
          ((0.5333333333333333, img_scale), (0.0, 12.0)))
device = torch.device('cuda')
weights = "/home/nvidia/mirv_ws/src/MIRV-Robot/mirv_real/Camera/tools/weights/End-to-end.pth"

CAMERA_HEIGHT = 0.175
CAMERA_ANGLE = -15

# Load model
model = get_net(cfg)
checkpoint = torch.load(weights, map_location=device)
model.load_state_dict(checkpoint['state_dict'])
model = model.to(device)

model.eval()
# model.cuda()

detections = 0

runningNeuralNetwork = True

i = 0
startTime = round(time.time())

frame = None
depthFrame = None


def allowNeuralNetRun(msg):
    cmd = msg.data
    global runningNeuralNetwork
    if(cmd == "lanes" or cmd == "piLitAndLanes"):
        runningNeuralNetwork = True
    else:
        runningNeuralNetwork = False


def updateTruckCoordinates(msg):
    global currentTruckCoordX
    global currentTruckCoordY
    currentTruckCoordX = msg.pose.pose.position.x
    currentTruckCoordY = msg.pose.pose.position.y


def execute_cb():
    print("GOT ACTION SERVER GOAL FOR LANES")
    goal = actionServer.accept_new_goal()
    position_x = goal.position_x
    position_y = goal.position_y
    heading = goal.heading
    lane_type = goal.formation_type

    lane_heading, detections, lane_width = process_frame(
        frame, position_x, position_y, heading)

    print(f"LANE LINE DETECTIONS: {detections}")
    print(f"LANE HEADING DETECTION: {lane_heading}")
    print(f"LANE WIDTH DETECTION: {lane_width}")

    # placements = placement.generate_pi_lit_formation(
    #     detections, lane_heading, lane_width, lane_type)
    result.net_heading = lane_heading
    result.width = lane_width
    result.lane_detections = detections
    actionServer.set_succeeded(result)


# callback function when receiving a frame
def gotFrame(data):
    # print("GOT A FRAME")
    # print(cv2.imwrite(r'camera_image.jpg', ros_numpy.numpify(data.color_frame)))
    # print(print(ros_numpy.numpify(data.color_frame)))
    # print(f"WRITING IMAGE")

    if(runningNeuralNetwork):

        global frame, depthFrame
        initTime = time.time()
        frame = ros_numpy.numpify(data.color_frame)
        depthFrame = ros_numpy.numpify(data.depth_frame)


def format_lane_detection(type, angle, x0, y0, x1, y1):
    lane = single_lane_detection()
    lane.heading = angle
    lane.start_x = x0
    lane.start_y = y0
    lane.end_x = x1
    lane.end_y = y1
    lane.lane_type = type
    return lane


def process_frame(frame, rover_position_x, rover_position_y, rover_heading):
    lane_points = get_lane_points(frame)
    left_lane, right_lane = retrieve_lanes(
        lane_points, (rover_position_x, rover_position_y), rover_heading)
    angle = 0
    lanes = []
    if left_lane and right_lane:
        angle = (left_lane[2] + right_lane[2]) / 2
        lanes.append(format_lane_detection('left', *left_lane[2:]))
        lanes.append(format_lane_detection('right', *right_lane[2:]))
        lanes['left'] = (left_lane[3], left_lane[4])
        lanes['right'] = (right_lane[3], right_lane[4])
    elif left_lane:
        angle = left_lane[2]
        lanes.append(format_lane_detection('left', *left_lane[2:]))
    elif right_lane:
        angle = right_lane[2]
        lanes.append(format_lane_detection('right', *right_lane[2:]))
    else:
        angle = None

    lane_heading = angle
    detections = lanes
    lane_width = 3

    return lane_heading, detections, lane_width


def get_lane_points(img):
    frame = img
    frame_orig = img
    img = transform(img).to(device)
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    # Inference
    det_out, da_seg_out, ll_seg_out = model(img)

    _, _, height, width = img.shape
    pad_w, pad_h = shapes[1][1]
    pad_w = int(pad_w)
    pad_h = int(pad_h)
    ratio = shapes[1][0][1]

    da_predict = da_seg_out[:, :, pad_h:(
        height-pad_h), pad_w:(width-pad_w)]
    da_seg_mask = torch.nn.functional.interpolate(
        da_predict, scale_factor=int(1/ratio), mode='bilinear')
    _, da_seg_mask = torch.max(da_seg_mask, 1)
    da_seg_mask = da_seg_mask.int().squeeze().cpu().numpy()
    # da_seg_mask = morphological_process(da_seg_mask, kernel_size=7)

    ll_predict = ll_seg_out[:, :, pad_h:(
        height-pad_h), pad_w:(width-pad_w)]
    ll_seg_mask = torch.nn.functional.interpolate(
        ll_predict, scale_factor=int(1/ratio), mode='bilinear')
    _, ll_seg_mask = torch.max(ll_seg_mask, 1)
    ll_seg_mask = ll_seg_mask.int().squeeze().cpu().numpy()

    img_det = show_seg_result(
        frame, (da_seg_mask, ll_seg_mask), _, _, is_demo=True)

    # Lane line post-processing
    ll_seg_mask = morphological_process(
        ll_seg_mask, kernel_size=7, func_type=cv2.MORPH_OPEN)
    ll_seg_mask, lines = connect_lane(ll_seg_mask)
    global i
    image_dir = "/media/nvidia/SSD/lane_pictures"
    print(cv2.imwrite(f'{image_dir}/img_{startTime}_{i}.png', frame_orig))
    print(cv2.imwrite(f'{image_dir}/img_det_{startTime}_{i}.png', img_det))
    print(cv2.imwrite(
        f'{image_dir}/da_seg_mask_{startTime}_{i}.png', da_seg_mask))
    print(cv2.imwrite(
        f'{image_dir}/ll_seg_mask_{startTime}_{i}.png', ll_seg_mask))
    i += 1
    return lines


# Identify and generate left and right lane line positions + angles
def retrieve_lanes(lines, rover_position, rover_heading):
    left_lane = None
    right_lane = None
    for line in lines:
        line = list(line)
        x_intercept, dx_sign, angle, x0, y0 = pixel_angles.get_line_equations(
            line, rover_heading, rover_position, (0, CAMERA_ANGLE), CAMERA_HEIGHT)

        print(x_intercept, dx_sign, angle, x0, y0)

        if not angle or not x_intercept:  # No points on road, or horizontal
            continue

        elif x_intercept <= 320/img_scale and dx_sign >= 0:  # Left side, pointing right
            if left_lane:
                if x_intercept > left_lane[0]:  # Find closest to center
                    left_lane = (x_intercept, dx_sign, angle, x0, y0)
            else:
                left_lane = (x_intercept, dx_sign, angle, x0, y0)
        if x_intercept > 320/img_scale and dx_sign <= 0:  # Right side, pointing left
            if right_lane:
                if x_intercept < right_lane[0]:  # Find closest to center
                    right_lane = (x_intercept, dx_sign, angle, x0, y0)
            else:
                right_lane = (x_intercept, dx_sign, angle, x0, y0)

    return left_lane, right_lane


rospy.init_node('laneLineDetector')
rospy.Subscriber("IntakeCameraFrames", depthAndColorFrame, gotFrame)

placementPublisher = rospy.Publisher(
    'pathingPointInput', Float64MultiArray, queue_size=1)

laneBoundPublisher = rospy.Publisher(
    "laneBound", Float64MultiArray, queue_size=1)
rospy.Subscriber("neuralNetworkSelector", String, allowNeuralNetRun)

_action_name = "LaneLineAS"
result = ASmsg.DetectLanesResult()
global actionServer
actionServer = actionlib.SimpleActionServer(
    _action_name, ASmsg.DetectLanesAction, auto_start=False)
actionServer.register_goal_callback(execute_cb)
actionServer.start()

try:
    rospy.spin()
except KeyboardInterrupt:
    cv2.destroyAllWindows()
