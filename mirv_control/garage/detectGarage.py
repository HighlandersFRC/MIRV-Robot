#!/usr/bin/env python3
import cv2
from cv2 import aruco

import numpy as np
import math

import rospy
from std_msgs.msg import String
import ros_numpy

from mirv_control.msg import depth_and_color_msg as depthAndColorFrame
from mirv_control.msg import garage_position as GaragePosition
from mirv_control.msg import aruco_detections as ArucoDetections
from mirv_control.msg import aruco_detection as ArucoDetection
from mirv_control.msg import camera_calibration as CameraCalibrationMsg

INCHES_TO_METERS = 0.0254


class GarageDetection:

    def __init__(self):
        rospy.init_node("garageDetection", anonymous=True)
        self.runningDetection = False

        self.hFOV = 63
        self.horizontalPixels = 640
        self.verticalPixels = 480
        self.degreesPerPixel = self.hFOV/self.horizontalPixels

        # Transformation from ArUco tag coordinate systems to garage coordinates, keyed by tag ID, in meters
        # Garage coordinates are zeroed at the back of the garage, pointing out (towards the rover, when stowed)
        # Not totally sure why the x_scale and y_scale are necessary, but when tuned correctly the position estimations are spot on
        self.transformations = {
            0: {
                'angle': 0,
                'x_offset': 0,
                'y_offset': 0,
                'x_scale': 1,
                'y_scale': 1.35,
            },
            1: {
                'angle': -76.5 * math.pi/180,
                'x_offset': 22.5 * INCHES_TO_METERS,
                'y_offset': -(25.5/2 - 2) * INCHES_TO_METERS - 0.02,
                'x_scale': 0.91,
                'y_scale': 0.84,
            },
            2: {
                'angle': 80.1 * math.pi/180,
                'x_offset': 22.5 * INCHES_TO_METERS,
                'y_offset': 25.5/2 * INCHES_TO_METERS + 0.05,
                'x_scale': 0.91,
                'y_scale': 0.84,
            }
        }

        self.calibration = CameraCalibrationMsg()
        self.calibration.matrix = np.array([[5.3056286139388271e+02, 0., 3.4914697544458500e+02],
                                            [0., 5.4078105552029069e+02,
                                                2.0340805917068764e+02],
                                            [0., 0., 1.]])
        self.calibration.distortion = np.array([1.0776936782086934e+00, -3.0943308900397768e+00,
                                                2.8548442856634527e-02, 5.6194055495202615e-02,
                                                3.3301192908663362e+00])
        self.calibration.hFOV = self.hFOV
        self.calibration.horizontalPixels = self.horizontalPixels
        self.calibration.verticalPixels = self.verticalPixels

        self.arucoTagId = 0
        self.markerSize = 6 * INCHES_TO_METERS  # Full size of marker, in meters
        self.arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.arucoParameters = aruco.DetectorParameters_create()

        self.SEARCH_TIME = 2  # seconds

        self.image_sub = rospy.Subscriber(
            "IntakeCameraFrames", depthAndColorFrame, self.gotFrame)
        self.network_sub = rospy.Subscriber(
            "neuralNetworkSelector", String, self.allowNeuralNetRun)
        self.garage_detections_pub = rospy.Publisher(
            "GarageArucoDetections", ArucoDetections, queue_size=1)
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

    def rotateAxes2d(self, x, y, theta):
        x_r_x = x * math.sin(theta)
        x_r_y = x * math.cos(theta)
        y_r_x = y * math.sin(math.pi/2 + theta)
        y_r_y = y * math.cos(math.pi/2 + theta)
        x_r = x_r_x + y_r_x
        y_r = x_r_y + y_r_y
        return x_r, y_r

    def getArucoDetections(self, frame, depthFrame):
        detections = []

        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            frame, self.arucoDict, parameters=self.arucoParameters)

        if np.all(ids is not None):  # If there are markers found by detector
            ids = ids.flatten()
            for id, corner in zip(ids, corners):  # Iterate in markers
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corner, self.markerSize, self.calibration.matrix,
                                                                           self.calibration.distortion)

                rvec, tvec = rvec.reshape((3, 1)), tvec.reshape((3, 1))

                rvec_t, tvec_t = self.invertCameraPerspective(rvec, tvec)
                detection = {}
                detection['id'] = id
                detection['rvec'] = rvec
                detection['tvec'] = tvec
                detection['rvec_t'] = rvec_t
                detection['tvec_t'] = tvec_t
                detection['corners'] = corner
                if id == 0:
                    angle, depth = self.getPositionWithinFrame(
                        corner.reshape(4, 2, 1), depthFrame)
                    detection['angle_within_frame'] = angle
                    detection['depth_distance'] = depth
                detections.append(detection)
        return detections

    def combineDetections(self, detections):
        angle = None
        depth = None
        results = {}
        aruco_detections = []
        for d in detections:
            id = d['id']
            rvec = d['rvec']
            tvec = d['tvec']
            rvec_t = d['rvec_t']
            tvec_t = d['tvec_t']
            corner = d['corners']
            angle_d = d.get('angle_within_frame')
            depth_d = d.get('depth_distance')

            x = tvec_t[2][0]
            y = -tvec_t[0][0]

            t = self.transformations.get(id)
            if not t:
                continue

            x_r, y_r = self.rotateAxes2d(y, x, t['angle'])
            x_r += t['x_offset']
            y_r += t['y_offset']
            x_r *= t['x_scale']
            y_r *= t['y_scale']

            results[id] = {
                'id': id,
                'x': x_r,
                'y': y_r,
                'angle': angle_d,
                'depth': depth_d,
            }
            detection = ArucoDetection()
            detection.id = id
            detection.x = x_r
            detection.y = y_r
            aruco_detections.append(detection)

        aruco_detections_obj = ArucoDetections()
        aruco_detections_obj.detections = aruco_detections
        self.garage_detections_pub.publish(aruco_detections_obj)

        positions = []
        if results.get(1):
            positions.append(results[1])
        if results.get(2):
            positions.append(results[2])
        if results.get(0):
            angle = results[0]['angle']
            depth = results[0]['depth']
            delta_x = 0
            delta_y = 0
            for i in positions:
                delta_x = results[0]['x'] - i['x']
                delta_y = results[0]['y'] - i['y']
            if abs(delta_x) < 0.2 and abs(delta_y) < 0.4:
                positions.append(results[0])
        x = 0.0
        y = 0.0
        ids = []
        l = len(positions)
        for i in positions:
            ids.append(i['id'])
            x += i['x']/l
            y += i['y']/l

        if x == 0 and y == 0:
            x = None
            y = None

        return x, y, angle, depth

    def getCameraPositionFromFrame(self, frame, depthFrame):

        detections = self.getArucoDetections(frame, depthFrame)

        return self.combineDetections(detections)

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
        if pos_x and pos_y and angle and depth:
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
            frame = ros_numpy.numpify(data.color_frame)
            depthFrame = ros_numpy.numpify(data.depth_frame)
            self.detect_markers(frame, depthFrame)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    print("RUNNING GARAGE DETECTION")
    garage = GarageDetection()
    garage.run()
