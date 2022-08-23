#!/usr/bin/env python3
from __future__ import print_function
import os
import roslib
import sys
import rospy
import numpy as np
import cv2
from cv2 import aruco
import tf2_ros
import pickle
import datetime
import time
from sensor_msgs.msg import Image
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Pose
# from dse_msgs.msg import PoseMarkers
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R

from mirv_control.msg import depth_and_color_msg as depthAndColorFrame

import mirv_control.helpful_functions_lib as conversion_lib

import ros_numpy


class aruco_pose:

    def __init__(self):
        self.cameraMatrix = np.array([[402.62901912,   0,         312.7377781],
                                      [0,         415.62202789, 225.59830764],
                                      [0,           0,           1]])

        self.distCoeffs = np.array(
            [[-0.02239511,  0.03570026, -0.00733071, -0.00355913,  0.00224717]])
        self.arucoTagId = 0
        self.markerSize = 8 * 0.0254  # Full size of marker, in meters
        self.arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.arucoParameters = aruco.DetectorParameters_create()

        self.poseZero = PoseStamped()
        self.poseZero.pose.position.x = 0
        self.poseZero.pose.position.y = 0
        self.poseZero.pose.position.z = 0
        self.poseZero.pose.orientation.x = 0
        self.poseZero.pose.orientation.y = 0
        self.poseZero.pose.orientation.z = 0
        self.poseZero.pose.orientation.w = 1

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            self.ros_prefix + "IntakeCameraFrames", depthAndColorFrame, self.callback)
        self.pose_pub = rospy.Publisher(
            self.ros_prefix + "/aruco/garage_tag", PoseStamped, queue_size=1)
        self.debug_pub = rospy.Publisher(
            self.ros_prefix + "/aruco/tag0_in_camera_frame", PoseStamped, queue_size=1)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def invertPerspective(rvec, tvec):
        # Generate camera position relative to aruco tag
        # Copied from https://aliyasineser.medium.com/calculation-relative-positions-of-aruco-markers-eee9cc4036e3
        R, _ = cv2.Rodrigues(rvec)
        R = np.matrix(R).T
        invTvec = np.dot(-R, np.matrix(tvec))
        invRvec, _ = cv2.Rodrigues(R)
        return invRvec, invTvec

    def getCameraPositionFromFrame(self, frame):
        # Generate camera and aruco poses from camera frame
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            frame, self.arucoDict, parameters=self.arucoParameters)

        if np.all(ids is not None):  # If there are markers found by detector
            ids = ids.flatten()
            marker_r_vec = None
            marker_t_vec = None
            for id, corner in zip(ids, corners):  # Iterate in markers
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corner, self.markerSize, self.cameraMatrix,
                                                                           self.distCoeffs)

                if id == self.arucoTagId:
                    marker_r_vec = rvec
                    marker_t_vec = tvec
            if 0 in ids:
                rvec, tvec = marker_r_vec.reshape(
                    (3, 1)), marker_t_vec.reshape((3, 1))
                pose_from_camera = self.getPoseFromVectors(
                    rvec, tvec, "pose_from_camera")
                pose_from_aruco = self.getPoseFromVectors(
                    *self.invertPerspective(rvec, tvec), "pose_from_aruco")
                return pose_from_camera, pose_from_aruco
        return None, None

    def getPoseFromVectors(rvec, tvec, id=""):
        # Generate PoseStamped obj from rvec and tvec
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = id

        # Apply linear bias to the translation estimates
        # x = tvecs[0][0][2]/1.184 + 0.110
        # y = -tvecs[0][0][0]/1.032 + 0.243
        # z = -tvecs[0][0][1]/1.151 - 0.297
        # dist = np.sqrt(x**2 + y**2 + z**2)
        # x = x - 0.008*dist + 0.031
        # y = y + 0.049*dist - 0.222
        # z = z - 0.062*dist + 0.281
        pose.pose.position.x = tvec[2][0]
        pose.pose.position.y = -tvec[0][0]
        pose.pose.position.z = -tvec[1][0]

        # Swap the angles around to correctly represent our coordinate system
        # Aruco puts zero at the tag, with z facing out...
        # We want x forward, y left, z up, euler order zyx = ypr
        rvecs_reordered = [rvec[2][0], -rvec[0][0], -rvec[1][0]]
        r = R.from_rotvec(rvecs_reordered)
        est_ypr = r.as_euler('zyx')
        quat = conversion_lib.eul2quat(est_ypr[:, None])
        # r = R.from_euler('zyx', est_ypr + [np.pi, 0, np.pi])
        # quat = r.as_quat()
        # print(quat[:])
        # print(pose.pose.orientation)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

    def callback(self, data):
        frame = ros_numpy.numpify(data.color_frame)
        depthFrame = ros_numpy.numpify(data.depth_frame)

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        pose_from_camera, pose_from_aruco = self.getCameraPositionFromFrame(
            frame)

        # Should always both be invalid, or both be valid
        if pose_from_camera != None and pose_from_aruco != None:
            self.pose_pub.publish(pose_from_camera)
            self.debug_pub.publish(pose_from_aruco)


def main(args):
    rospy.init_node('aruco_pose_estimation_node', anonymous=True)
    ic = aruco_pose()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
