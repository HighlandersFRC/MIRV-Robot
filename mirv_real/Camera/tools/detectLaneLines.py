#!/usr/bin/env python3
import placement
from std_msgs.msg import Float64
import actionlib
import mirv_control.msg as ASmsg
from mirv_control.msg import depth_and_color_msg as depthAndColorFrame
from mirv_control.msg import single_lane_detection
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


class detectLaneLines():
    def __init__(self):
        rospy.init_node("laneLineDetector", anonymous=True)
        self._feedback = ASmsg.DetectLanesFeedback()
        self._result = ASmsg.DetectLanesResult()

        self._action_name = "LaneLineAS"
        self._as = actionlib.SimpleActionServer(
            self._action_name, ASmsg.DetectLanesAction, auto_start=False)
        self._as.register_goal_callback(self.detectLanes)
        self._as.register_preempt_callback(self.preemptedAS)
        self._as.start()

        self.intakeCameraSub = rospy.Subscriber("IntakeCameraFrames",
                                                depthAndColorFrame, self.gotFrame)
        self.networkSub = rospy.Subscriber("neuralNetworkSelector",
                                           String, self.allowNeuralNetRun)

        self.normalize = transforms.Normalize(
            mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
        )

        self.transform = transforms.Compose([
            transforms.ToTensor(),
            self.normalize,
        ])

        self.img_scale = 0.5

        self.img_height = 480/self.img_scale
        self.img_width = 640/self.img_scale

        # TODO: verify this value
        self.shapes = ((self.img_height, self.img_width),
                       ((0.5333333333333333, self.img_scale), (0.0, 12.0)))
        self.device = torch.device('cuda')
        self.weights = "/home/nvidia/mirv_ws/src/MIRV-Robot/mirv_real/Camera/tools/weights/End-to-end.pth"

        self.CAMERA_HEIGHT = 0.175
        self.CAMERA_ANGLE = -15

        # Load model
        self.model = get_net(cfg)
        self.checkpoint = torch.load(self.weights, map_location=self.device)
        self.model.load_state_dict(self.checkpoint['state_dict'])
        self.model = self.model.to(self.device)

        self.model.eval()

        self.detections = 0

        self.runningNeuralNetwork = True

        self.i = 0
        self.startTime = round(time.time())

        self.frame = None
        self.depthFrame = None

    def allowNeuralNetRun(self, msg):
        cmd = msg.data
        if(cmd == "lanes" or cmd == "piLitAndLanes"):
            self.runningNeuralNetwork = True
        else:
            self.runningNeuralNetwork = False

    def detectLanes(self):
        print("GOT ACTION SERVER GOAL FOR LANES")
        goal = self._as.accept_new_goal()
        position_x = goal.position_x
        position_y = goal.position_y
        heading = goal.heading
        lane_type = goal.formation_type

        lane_heading, detections, lane_width = self.process_frame(
            self.frame, position_x, position_y, heading)

        print(f"LANE LINE DETECTIONS: {detections}")
        print(f"LANE HEADING DETECTION: {lane_heading}")
        print(f"LANE WIDTH DETECTION: {lane_width}")

        self._result.net_heading = float(lane_heading)
        self._result.width = float(lane_width)
        self._result.lane_detections = detections

        self._as.set_succeeded(self._result)
        self.setAllZeros()

    def gotFrame(self, data):
        if(runningNeuralNetwork):

            self.initTime = time.time()
            self.frame = ros_numpy.numpify(data.color_frame)
            self.depthFrame = ros_numpy.numpify(data.depth_frame)

    def format_lane_detection(self, type, angle, x0, y0, x1, y1):
        lane = single_lane_detection()
        lane.heading = float(angle)
        lane.start_x = float(x0)
        lane.start_y = float(y0)
        lane.end_x = float(x1)
        lane.end_y = float(y1)
        lane.lane_type = type
        return lane

    def process_frame(self, frame, rover_position_x, rover_position_y, rover_heading):
        lane_points = self.get_lane_points(frame)
        left_lane, right_lane = self.retrieve_lanes(
            lane_points, (rover_position_x, rover_position_y), rover_heading)
        angle = 0
        lanes = []
        if left_lane and right_lane:
            angle = (left_lane[2] + right_lane[2]) / 2
            lanes.append(self.format_lane_detection('left', *left_lane[2:]))
            lanes.append(self.format_lane_detection('right', *right_lane[2:]))
        elif left_lane:
            angle = left_lane[2]
            lanes.append(self.format_lane_detection('left', *left_lane[2:]))
        elif right_lane:
            angle = right_lane[2]
            lanes.append(self.format_lane_detection('right', *right_lane[2:]))

        lane_heading = angle
        detections = lanes
        lane_width = 3

        return lane_heading, detections, lane_width

    def get_lane_points(self, img):
        frame = img
        frame_orig = img
        img = self.transform(img).to(self.device)
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        det_out, da_seg_out, ll_seg_out = self.model(img)

        _, _, height, width = img.shape
        pad_w, pad_h = self.shapes[1][1]
        pad_w = int(pad_w)
        pad_h = int(pad_h)
        ratio = self.shapes[1][0][1]

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
        print(cv2.imwrite(
            f'{image_dir}/img_{self.startTime}_{self.i}.png', frame_orig))
        print(cv2.imwrite(
            f'{image_dir}/img_det_{self.startTime}_{self.i}.png', img_det))
        print(cv2.imwrite(
            f'{image_dir}/da_seg_mask_{self.startTime}_{self.i}.png', da_seg_mask))
        print(cv2.imwrite(
            f'{image_dir}/ll_seg_mask_{self.startTime}_{self.i}.png', ll_seg_mask))
        i += 1
        return lines

    # Identify and generate left and right lane line positions + angles

    def retrieve_lanes(self, lines, rover_position, rover_heading):
        left_lane = None
        right_lane = None
        for line in lines:
            line = list(line)
            x_intercept, dx_sign, angle, x0, y0, x1, y1 = pixel_angles.get_line_equations(
                line, rover_heading, rover_position, (0, self.CAMERA_ANGLE), self.CAMERA_HEIGHT)
            print(x_intercept, dx_sign, angle, x0, y0)

            if not angle or not x_intercept:  # No points on road, or horizontal
                continue

            elif x_intercept <= 320/self.img_scale and dx_sign >= 0:  # Left side, pointing right
                if left_lane:
                    if x_intercept > left_lane[0]:  # Find closest to center
                        left_lane = (x_intercept, dx_sign,
                                     angle, x0, y0, x1, y1)
                else:
                    left_lane = (x_intercept, dx_sign, angle, x0, y0, x1, y1)
            if x_intercept > 320/self.img_scale and dx_sign <= 0:  # Right side, pointing left
                if right_lane:
                    if x_intercept < right_lane[0]:  # Find closest to center
                        right_lane = (x_intercept, dx_sign,
                                      angle, x0, y0, x1, y1)
                else:
                    right_lane = (x_intercept, dx_sign, angle, x0, y0, x1, y1)

        return left_lane, right_lane

    def preemptedAS(self):
        if(self._as.is_new_goal_available()):
            self._as.set_preempted()
        else:
            self._as.set_aborted()

    def run(self):
        rospy.spin()


cls = detectLaneLines()
if __name__ == '__main__':
    print("RUNNING")
    cls.run()
