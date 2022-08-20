#!/usr/bin/env python3
import math
import cv2
import time
import depthai
# import rospy
# from std_msgs.msg import Float64
# from mirv_control.msg import depth_and_color_msg as depthAndColorFrame
# from cv_bridge import CvBridge
import sys
import signal
import numpy as np
import datetime


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


def is_checkerboard(img):
    n = 9
    m = 7
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    objp = np.zeros((m*n, 3), np.float32)
    objp[:, :2] = np.mgrid[0:n, 0:m].T.reshape(-1, 2)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (n, m), None)

    # If found, add object points, image points (after refining them)
    if ret:
        return True
    else:
        return False


signal.signal(signal.SIGINT, interrupt_handler)

try:
    j = 0
    while True:
        if j % 10 != 0:
            continue
        in_rgb = q_rgb.tryGet()
        if in_rgb is not None:
            frame = qIsp.get().getCvFrame()
            frame = cv2.resize(frame, (640, 480),
                               interpolation=cv2.INTER_LINEAR)
            cv2.imshow('frame', frame)

            key = cv2.waitKey(3)
            if key == ord('q'):  # Quit
                break
            if key == ord('t'):
                if is_checkerboard(frame):
                    date_str = datetime.datetime.now().strftime('%Y%m%d-%M%H%S-%f')
                    cv2.imwrite(
                        f"../auto_calibration_images_small/calibration_{date_str}.jpg", frame)
                    print(
                        f'recorded image - {date_str}')
                else:
                    print('no checkerboard')

except KeyboardInterrupt:
    print("KEYBOARD INTERRUPT")
    # break
    # except:
    #     print("here")
    #     break
