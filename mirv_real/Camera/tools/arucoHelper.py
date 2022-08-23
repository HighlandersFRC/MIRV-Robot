#!/usr/bin/env python3
import cv2
import numpy as np

import rospy
from tf2_geometry_msgs import PoseStamped
import mirv_control.helpful_functions_lib as conversion_lib
from scipy.spatial.transform import Rotation as R


def invertCameraPerspective(rvec, tvec):
    # Generate camera position relative to aruco tag
    # Copied from https://aliyasineser.medium.com/calculation-relative-positions-of-aruco-markers-eee9cc4036e3
    R, _ = cv2.Rodrigues(rvec)
    R = np.matrix(R).T
    invTvec = np.dot(-R, np.matrix(tvec))
    invRvec, _ = cv2.Rodrigues(R)
    return invRvec, invTvec


def getPoseFromVectors(self, rvec, tvec, id=""):
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
