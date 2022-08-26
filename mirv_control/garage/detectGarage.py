#!/usr/bin/env python3
import cv2
from cv2 import aruco

import numpy as np

import rospy
from std_msgs.msg import String
import ros_numpy

from mirv_control.msg import depth_and_color_msg as depthAndColorFrame
from mirv_control.msg import garage_position as GaragePosition
from mirv_control.msg import camera_calibration as CameraCalibrationMsg



class GarageDetection:

    def __init__(self):
        rospy.init_node("garageDetection", anonymous=True)
        self.runningDetection = False

        self.hFOV = 63
        self.horizontalPixels = 1280
        self.verticalPixels = 720
        self.degreesPerPixel = self.hFOV/self.horizontalPixels

        self.calibration = CameraCalibrationMsg()
        self.calibration.matrix = np.array([[ 5.3056286139388271e+02, 0., 1280/2], [0.,
            5.4078105552029069e+02, 720/2], [0., 0., 1.]])
        self.calibration.distortion = np.array([ 1.0776936782086934e+00, -3.0943308900397768e+00,
            2.8548442856634527e-02, 5.6194055495202615e-02,
            3.3301192908663362e+00])
        self.calibration.hFOV = self.hFOV
        self.calibration.horizontalPixels = self.horizontalPixels
        self.calibration.verticalPixels = self.verticalPixels

        self.arucoTagId = 0
        self.markerSize = 6 * 0.0254  # Full size of marker, in meters
        self.arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.arucoParameters = aruco.DetectorParameters_create()

        self.SEARCH_TIME = 2  # seconds

        self.image_sub = rospy.Subscriber(
            "IntakeCameraFrames", depthAndColorFrame, self.gotFrame)
        self.network_sub = rospy.Subscriber(
            "neuralNetworkSelector", String, self.allowNeuralNetRun)
        self.garage_position_pub = rospy.Publisher(
            "GaragePosition", GaragePosition, queue_size=1)

    def movingaverage(self, interval, window_size):
        window = np.ones(int(window_size))/float(window_size)
        return np.convolve(interval, window, 'same')

    def calibrationCallback(self, calibrationMsg):
        self.calibration = calibrationMsg

    def invertCameraPerspective(self, rvec, tvec):
        # Generate camera position relative to aruco tag
        # Copied from https://aliyasineser.medium.com/calculation-relative-positions-of-aruco-markers-eee9cc4036e3
        R, _ = cv2.Rodrigues(rvec)
        R = np.matrix(R).T
        invTvec = np.dot(-R, np.matrix(tvec))
        invRvec, _ = cv2.Rodrigues(R)
        return invRvec, invTvec

    def getPositionWithinFrame(self, corners, depthFrame):
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)

        angle = (cX - self.calibration.horizontalPixels/2) * \
            self.degreesPerPixel
        depth = (depthFrame[cY][cX])/1000

        return angle, depth

    def getCameraPositionFromFrame(self, frame, depthFrame):
        # Generate camera and aruco poses from camera frame
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            frame, self.arucoDict, parameters=self.arucoParameters)

        if np.all(ids is not None):  # If there are markers found by detector
            ids = ids.flatten()
            marker_r_vec = None
            marker_t_vec = None
            for id, corner in zip(ids, corners):  # Iterate in markers
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corner, self.markerSize, self.calibration.matrix,
                                                                           self.calibration.distortion)

                if id == self.arucoTagId:
                    marker_r_vec = rvec
                    marker_t_vec = tvec
            if 0 in ids:
                rvec, tvec = marker_r_vec.reshape(
                    (3, 1)), marker_t_vec.reshape((3, 1))
                rvec_inv, tvec_inv = self.invertCameraPerspective(rvec, tvec)
                rover_position_x_from_garage = tvec_inv[2][0]
                rover_position_y_from_garage = -tvec_inv[0][0]
                angle, depth = self.getPositionWithinFrame(corner.reshape(4, 2, 1), depthFrame)
                return rover_position_x_from_garage, rover_position_y_from_garage, angle, depth
        return None, None, None, None

    def allowNeuralNetRun(self, msg):
        cmd = msg.data
        if(cmd == "aruco"):
            self.runningDetection = True
        else:
            self.runningDetection = False

    def detect_markers(self, frame, depthFrame):
        pos_x, pos_y, angle, depth = self.getCameraPositionFromFrame(
            frame, depthFrame)

        # Should always both be invalid, or both be valid
        if pos_x != None and pos_y != None:

            garagePosition = GaragePosition()
            garagePosition.rover_position_x_from_garage = pos_x
            garagePosition.rover_position_y_from_garage = pos_y
            garagePosition.angle_to_garage = angle
            garagePosition.depth_to_garage = depth

            self.garage_position_pub.publish(garagePosition)

    def gotFrame(self, data):
        if self.calibration is None:
            print("Camera is not calibrated")
            return
        if self.runningDetection:
            # print("Running Detection")
            frame = ros_numpy.numpify(data.color_frame)
            depthFrame = ros_numpy.numpify(data.depth_frame)
            self.detect_markers(frame, depthFrame)

    # def getPoseFromVectors(self, rvec, tvec, id=""):
    #     # Generate PoseStamped obj from rvec and tvec
    #     pose = PoseStamped()
    #     pose.header.stamp = rospy.Time.now()
    #     pose.header.frame_id = id

    #     # Apply linear bias to the translation estimates
    #     # x = tvecs[0][0][2]/1.184 + 0.110
    #     # y = -tvecs[0][0][0]/1.032 + 0.243
    #     # z = -tvecs[0][0][1]/1.151 - 0.297
    #     # dist = np.sqrt(x**2 + y**2 + z**2)
    #     # x = x - 0.008*dist + 0.031
    #     # y = y + 0.049*dist - 0.222
    #     # z = z - 0.062*dist + 0.281
    #     pose.pose.position.x = tvec[2][0]
    #     pose.pose.position.y = -tvec[0][0]
    #     pose.pose.position.z = -tvec[1][0]

    #     # Swap the angles around to correctly represent our coordinate system
    #     # Aruco puts zero at the tag, with z facing out...
    #     # We want x forward, y left, z up, euler order zyx = ypr
    #     rvecs_reordered = [rvec[2][0], -rvec[0][0], -rvec[1][0]]
    #     r = R.from_rotvec(rvecs_reordered)
    #     est_ypr = r.as_euler('zyx')
    #     quat = conversion_lib.eul2quat(est_ypr[:, None])
    #     # r = R.from_euler('zyx', est_ypr + [np.pi, 0, np.pi])
    #     # quat = r.as_quat()
    #     # print(quat[:])
    #     # print(pose.pose.orientation)
    #     pose.pose.orientation.x = quat[0]
    #     pose.pose.orientation.y = quat[1]
    #     pose.pose.orientation.z = quat[2]
    #     pose.pose.orientation.w = quat[3]

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    print("RUNNING GARAGE DETECTION")
    garage = GarageDetection()
    garage.run()
