import datetime
import signal
import sys
import depthai
import time
import math
import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import argparse


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


firstMarkerID = 0
secondMarkerID = 1


CALIBRATION_DIRETORY = "../auto_calibration_images"


#!/usr/bin/env python3
# import rospy
# from std_msgs.msg import Float64
# from mirv_control.msg import depth_and_color_msg as depthAndColorFrame
# from cv_bridge import CvBridge


def quat_2_radians(x, y, z, w):
    pitch = math.atan2(2*x*w - 2*y*z, 1-2*x*x - 2 * z*z)
    yaw = math.asin(2*x*y + 2*z*w)
    roll = math.atan2(2*x*w - 2*x*z, 1-2*y*y - 2*z*z)
    return pitch, yaw, roll


cameraX = 640
cameraY = 480

# br = CvBridge()

# calibrationFeedbackPub = rospy.Publisher('CameraCalibrationFeedback', str)
# rospy.init_node('CameraCalibrator', anonymous=True)

pipeline = depthai.Pipeline()

# creating rgb camera
cam_rgb = pipeline.create(depthai.node.ColorCamera)
cam_rgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_12_MP)
cam_rgb.setInterleaved(False)
cam_rgb.setIspScale(1, 5)
cam_rgb.setPreviewSize(812, 608)
cam_rgb.setColorOrder(depthai.ColorCameraProperties.ColorOrder.RGB)

# full fov frame
xoutIsp = pipeline.create(depthai.node.XLinkOut)
xoutIsp.setStreamName("isp")
cam_rgb.isp.link(xoutIsp.input)

# creating imu stream and enabling imu sensor
imu = pipeline.createIMU()
xlinkOut = pipeline.createXLinkOut()
xlinkOut.setStreamName("imu")
imu.enableIMUSensor(
    depthai.IMUSensor.ARVR_STABILIZED_GAME_ROTATION_VECTOR, 400)
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)
imu.out.link(xlinkOut.input)

# creating stream for rgb frames and camera config
xout_rgb = pipeline.create(depthai.node.XLinkOut)
configIn = pipeline.create(depthai.node.XLinkIn)
configIn.setStreamName('config')
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)
controlIn = pipeline.create(depthai.node.XLinkIn)
controlIn.setStreamName('control')
controlIn.out.link(cam_rgb.inputControl)

# Define a source - two mono (grayscale) cameras
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()
stereo = pipeline.createStereoDepth()
spatialLocationCalculator = pipeline.createSpatialLocationCalculator()

# create streams for depth data
xoutDepth = pipeline.createXLinkOut()
xoutSpatialData = pipeline.createXLinkOut()
xinSpatialCalcConfig = pipeline.createXLinkIn()
xoutDepth.setStreamName("depth")
xoutSpatialData.setStreamName("spatialData")
xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

# MonoCamera
monoLeft.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_720_P)
monoLeft.setBoardSocket(depthai.CameraBoardSocket.LEFT)
monoRight.setResolution(
    depthai.MonoCameraProperties.SensorResolution.THE_720_P)
monoRight.setBoardSocket(depthai.CameraBoardSocket.RIGHT)

# depth settings
outputDepth = True
outputRectified = False
lrcheck = True
subpixel = False
extended = True

# StereoDepth
stereo.setOutputDepth(outputDepth)
stereo.setOutputRectified(outputRectified)
stereo.setConfidenceThreshold(255)
stereo.setDepthAlign(depthai.CameraBoardSocket.RGB)
stereo.setOutputSize(640, 480)
stereo.setLeftRightCheck(lrcheck)
stereo.setSubpixel(subpixel)
# stereo.setExtendedDisparity(extended)
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
stereo.depth.link(xoutDepth.input)

topLeft = depthai.Point2f(0.4, 0.4)
bottomRight = depthai.Point2f(0.6, 0.6)

spatialLocationCalculator.setWaitForConfigInput(False)
config = depthai.SpatialLocationCalculatorConfigData()
config.depthThresholds.lowerThreshold = 0
config.depthThresholds.upperThreshold = 10000
config.roi = depthai.Rect(topLeft, bottomRight)
spatialLocationCalculator.initialConfig.addROI(config)
spatialLocationCalculator.out.link(xoutSpatialData.input)
xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

expTime = 1
sensIso = 0

# finding device by specific ip
found, device_info = depthai.Device.getDeviceByMxId("10.0.20.2")
depthaiDevice = depthai.Device(pipeline, device_info)
depthaiDevice.startPipeline()

# create queues
q_rgb = depthaiDevice.getOutputQueue("rgb", maxSize=1, blocking=False)
depthQueue = depthaiDevice.getOutputQueue(
    name="depth", maxSize=1, blocking=False)
spatialCalcQueue = depthaiDevice.getOutputQueue(
    name="spatialData", maxSize=1, blocking=False)
spatialCalcConfigInQueue = depthaiDevice.getInputQueue("spatialCalcConfig")
qIsp = depthaiDevice.getOutputQueue("isp", maxSize=1, blocking=False)
frame = None
imuQueue = depthaiDevice.getOutputQueue(name="imu", maxSize=1, blocking=False)
disparityMultiplier = 255 / stereo.getMaxDisparity()
controlQueue = depthaiDevice.getInputQueue('control')
ctrl = depthai.CameraControl()
ctrl.setAutoFocusMode(depthai.CameraControl.AutoFocusMode.CONTINUOUS_PICTURE)
ctrl.setAutoWhiteBalanceMode(
    depthai.CameraControl.AutoWhiteBalanceMode.WARM_FLUORESCENT)
controlQueue.send(ctrl)


def interrupt_handler(signal, frame):
    # print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, interrupt_handler)


def calibrate():
    n = 7
    m = 7
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    objp = np.zeros((m*n, 3), np.float32)
    objp[:, :2] = np.mgrid[0:n, 0:m].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob.glob(f'{CALIBRATION_DIRETORY}/*.jpg')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (n, m), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (n, m), corners2, ret)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)

    return [ret, mtx, dist, rvecs, tvecs]


def saveCoefficients(mtx, dist):
    cv_file = cv2.FileStorage(
        f"{CALIBRATION_DIRETORY}/calibrationCoefficients.yaml", cv2.FILE_STORAGE_WRITE)
    cv_file.write("camera_matrix", mtx)
    cv_file.write("dist_coeff", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()


def loadCoefficients():
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(
        f"{CALIBRATION_DIRETORY}/calibrationCoefficients.yaml", cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("camera_matrix").mat()
    dist_matrix = cv_file.getNode("dist_coeff").mat()

    # Debug: print the values
    # print("camera_matrix : ", camera_matrix.tolist())
    # print("dist_matrix : ", dist_matrix.tolist())

    cv_file.release()
    return [camera_matrix, dist_matrix]


def inversePerspective(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    R = np.matrix(R).T
    invTvec = np.dot(-R, np.matrix(tvec))
    invRvec, _ = cv2.Rodrigues(R)
    return invRvec, invTvec


def relativePosition(rvec1, tvec1, rvec2, tvec2):
    rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape(
        (3, 1))
    rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))

    # Inverse the second marker, the right one in the image
    invRvec, invTvec = inversePerspective(rvec2, tvec2)
    # print(float(invRvec[0][0]))
    print(get_padded_num(invTvec[0][0]), get_padded_num(
        invTvec[1][0]), get_padded_num(invTvec[2][0]))

    orgRvec, orgTvec = inversePerspective(invRvec, invTvec)
    # print("rvec: ", rvec2, "tvec: ", tvec2, "\n and \n", orgRvec, orgTvec)

    info = cv2.composeRT(rvec1, tvec1, invRvec, invTvec)
    composedRvec, composedTvec = info[0], info[1]

    composedRvec = composedRvec.reshape((3, 1))
    composedTvec = composedTvec.reshape((3, 1))
    # print(get_padded_num(composedRvec[0][0]), get_padded_num(composedRvec[1][0]), get_padded_num(
    #     composedRvec[2][0]), get_padded_num(composedTvec[0][0]), get_padded_num(composedTvec[1][0]), get_padded_num(composedTvec[2][0]))
    return composedRvec, composedTvec


def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1, 2)
    # draw ground floor in green
    # img = cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)
    # draw pillars in blue color
    for i, j in zip(range(4), range(4)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255), 3)
    # draw top layer in red color
    return img


def track(matrix_coefficients, distortion_coefficients):
    pointCircle = (0, 0)
    markerTvecList = []
    markerRvecList = []
    composedRvec, composedTvec = None, None
    j = 0
    while True:
        j += 1
        ret, frame = cap.read()
        # operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
        # Use 5x5 dictionary to find markers
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()  # Marker detection parameters

        # lists of ids and the corners beloning to each id
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
                                                                parameters=parameters)

        if np.all(ids is not None):  # If there are markers found by detector
            del markerTvecList[:]
            del markerRvecList[:]
            zipped = zip(ids, corners)
            ids, corners = zip(*(sorted(zipped)))
            axis = np.float32([[-0.01, -0.01, 0], [-0.01, 0.01, 0],
                              [0.01, -0.01, 0], [0.01, 0.01, 0]]).reshape(-1, 3)
            for i in range(0, len(ids)):  # Iterate in markers
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 9, matrix_coefficients,
                                                                           distortion_coefficients)

                if ids[i] == firstMarkerID:
                    firstRvec = rvec
                    firstTvec = tvec
                    isFirstMarkerCalibrated = True
                    firstMarkerCorners = corners[i]
                elif ids[i] == secondMarkerID:
                    secondRvec = rvec
                    secondTvec = tvec
                    isSecondMarkerCalibrated = True
                    secondMarkerCorners = corners[i]

                # print(markerPoints)
                (rvec - tvec).any()  # get rid of that nasty numpy value array error
                markerRvecList.append(rvec)
                markerTvecList.append(tvec)

                # Draw A square around the markers
                aruco.drawDetectedMarkers(frame, corners)

            if len(ids) > 1 and composedRvec is not None and composedTvec is not None:
                info = cv2.composeRT(
                    composedRvec, composedTvec, secondRvec.T, secondTvec.T)
                TcomposedRvec, TcomposedTvec = info[0], info[1]

                # 3D point for projection
                objectPositions = np.array([(0, 0, 0)], dtype=np.float)
                imgpts, jac = cv2.projectPoints(axis, TcomposedRvec, TcomposedTvec, matrix_coefficients,
                                                distortion_coefficients)

                # frame = draw(frame, corners[0], imgpts)
                cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, TcomposedRvec, TcomposedTvec,
                                  0.01)  # Draw Axis
                # aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, TcomposedRvec, TcomposedTvec,
                #                0.01)  # Draw Axis
                relativePoint = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))
                cv2.circle(frame, relativePoint, 2, (255, 255, 0))

        # Display the resulting frame
        cv2.imshow('frame', frame)
        # Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
        key = cv2.waitKey(3) & 0xFF
        if key == ord('q'):  # Quit
            break
        elif ids and 0 in ids and j % 5 == 0:
            # elif key == ord('c'):  # Calibration
            # If there are two markers, reverse the second and get the difference
            # if ids and len(ids) > 1:

            rvec1, tvec1 = firstRvec.reshape(
                (3, 1)), firstTvec.reshape((3, 1))
            rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape((3, 1))
            invRvec, invTvec = inversePerspective(rvec1, tvec1)
            # print(float(invRvec[0][0]))
            print(get_padded_num(invTvec[0][0]), get_padded_num(
                invTvec[1][0]), get_padded_num(invTvec[2][0]))
            # secondRvec, secondTvec = secondRvec.reshape(
            #     (3, 1)), secondTvec.reshape((3, 1))

            # composedRvec, composedTvec = relativePosition(
            #     firstRvec, firstTvec, secondRvec, secondTvec)

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


def get_relative_offset(frame, aruco_dict, parameters, matrix_coefficients, distortion_coefficients):
    # operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale

    # lists of ids and the corners beloning to each id
    corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
                                                            parameters=parameters)

    firstRvec = None
    firstTvec = None

    if np.all(ids is not None):  # If there are markers found by detector
        ids = ids.flatten()
        for id, corner in zip(ids, corners):  # Iterate in markers
            # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corner, 8, matrix_coefficients,
                                                                       distortion_coefficients)

            # 12/5 -> 2.4
            # 18/8 -> 2.25

            if id == firstMarkerID:
                firstRvec = rvec
                firstTvec = tvec
                isFirstMarkerCalibrated = True
                firstMarkerCorners = corner
        if 0 in ids:
            rvec, tvec = firstRvec.reshape(
                (3, 1)), firstTvec.reshape((3, 1))
            rvec, tvec = rvec.reshape((3, 1)), tvec.reshape((3, 1))
            return inversePerspective(rvec, tvec)
    return None


def run(matrix_coefficients, distortion_coefficients):
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()  # Marker detection parameters
    j = 0
    x = 0
    y = 0
    z = 0
    a = 0.1
    while True:
        in_rgb = q_rgb.tryGet()
        if in_rgb is not None:
            frame = qIsp.get().getCvFrame()
            frame = cv2.resize(frame, (640, 480),
                               interpolation=cv2.INTER_LINEAR)
            cv2.imshow('frame', frame)

            key = cv2.waitKey(3)
            if key == ord('q'):  # Quit
                break
            if j % 50 == 0:
                response = get_relative_offset(
                    frame, aruco_dict, parameters, matrix_coefficients, distortion_coefficients)
                if response != None:
                    rvec, tvec = response
                    x = x * (1-a) + tvec[0][0] * a
                    y = y * (1-a) + tvec[1][0] * a
                    z = z * (1-a) + tvec[2][0] * a
                    print(get_padded_num(x), get_padded_num(
                        y), get_padded_num(z), get_padded_num(rvec[0][0]), get_padded_num(rvec[1][0]), get_padded_num(rvec[2][0]))


def get_padded_num(val):
    return str(round(float(val), 2)).zfill(5)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Aruco Marker Tracking')
    parser.add_argument('--coefficients', metavar='bool', required=True,
                        help='File name for matrix coefficients and distortion coefficients')
    parser.add_argument('--firstMarker', metavar='int', required=True,
                        help='Marker ID for the first marker')
    parser.add_argument('--secondMarker', metavar='int', required=True,
                        help='Marker ID for the second marker')

    # Parse the arguments and take action for that.
    args = parser.parse_args()
    firstMarkerID = int(args.firstMarker)
    secondMarkerID = int(args.secondMarker)

    if args.coefficients == '1':
        mtx, dist = loadCoefficients()
        ret = True
    else:
        ret, mtx, dist, rvecs, tvecs = calibrate()
        saveCoefficients(mtx, dist)
    print("Calibration is completed. Starting tracking sequence.")
    if ret:
        run(mtx, dist)
