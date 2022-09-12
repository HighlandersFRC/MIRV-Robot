'''
This software only collects and stores the information needed for the calibration.
Allows for RPi camera to be calibrated on a more capable computer.
'''
#!/usr/bin/env python3
import math
import cv2
import time
import depthai
import rospy
from std_msgs.msg import Float64
from mirv_control.msg import depth_and_color_msg as depthAndColorFrame
from cv_bridge import CvBridge
import sys
import signal
import numpy as np
import pickle
def quat_2_radians(x, y, z, w):
    pitch = math.atan2(2*x*w - 2*y*z, 1-2*x*x - 2* z*z)
    yaw = math.asin(2*x*y + 2*z*w)
    roll = math.atan2(2*x*w - 2*x*z, 1-2*y*y - 2*z*z)
    return pitch, yaw, roll

cameraX = 640
cameraY = 480

br = CvBridge()

imgPub = rospy.Publisher('IntakeCameraFrames', depthAndColorFrame, queue_size=1)
rospy.init_node('intakeCameraPublisher', anonymous=True)

imuPub = rospy.Publisher('CameraIMU', Float64, queue_size=1)

pipeline = depthai.Pipeline()

# creating rgb camera
cam_rgb = pipeline.create(depthai.node.ColorCamera)
cam_rgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_12_MP)
cam_rgb.setInterleaved(False)
cam_rgb.setIspScale(1,5)
cam_rgb.setPreviewSize(812, 608)
cam_rgb.setColorOrder(depthai.ColorCameraProperties.ColorOrder.RGB)

# full fov frame
xoutIsp = pipeline.create(depthai.node.XLinkOut)
xoutIsp.setStreamName("isp")
cam_rgb.isp.link(xoutIsp.input)

# creating stream for rgb frames and camera config
xout_rgb = pipeline.create(depthai.node.XLinkOut)
configIn = pipeline.create(depthai.node.XLinkIn)
configIn.setStreamName('config')
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)
controlIn = pipeline.create(depthai.node.XLinkIn)
controlIn.setStreamName('control')
controlIn.out.link(cam_rgb.inputControl)

expTime = 1
sensIso = 0

# finding device by specific ip
found, device_info = depthai.Device.getDeviceByMxId("10.0.20.2")
depthaiDevice = depthai.Device(pipeline, device_info)
depthaiDevice.startPipeline()

# create queues
q_rgb = depthaiDevice.getOutputQueue("rgb", maxSize=1, blocking=False)
qIsp = depthaiDevice.getOutputQueue("isp", maxSize=1, blocking=False)
frame = None
controlQueue = depthaiDevice.getInputQueue('control')
ctrl = depthai.CameraControl()
ctrl.setAutoFocusMode(depthai.CameraControl.AutoFocusMode.CONTINUOUS_PICTURE)
ctrl.setAutoWhiteBalanceMode(depthai.CameraControl.AutoWhiteBalanceMode.WARM_FLUORESCENT)
controlQueue.send(ctrl)

def interrupt_handler(signal, frame):
    # print('You pressed Ctrl+C!')
    depthaiDevice.close()
    sys.exit(0)


signal.signal(signal.SIGINT, interrupt_handler)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(5,8,.025,.0125,dictionary)
img = board.draw((200*3,200*3))

#Dump the calibration board to a file
cv2.imwrite('charuco.png',img)

allCorners = []
allIds = []
decimator = 0
for i in range(400):
    # if in_rgb is not None:
    #print("got a frame")
    frame = qIsp.get().getCvFrame()

    # resize frame for neural nets
    resizedFrame = cv2.resize(frame, (640, 480), interpolation = cv2.INTER_LINEAR)

    gray = cv2.cvtColor(resizedFrame, cv2.COLOR_BGR2GRAY)
    res = cv2.aruco.detectMarkers(gray,dictionary)

    if len(res[0])>0:
        res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board)
        if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%4==0:
            allCorners.append(res2[1])
            allIds.append(res2[2])

        cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])
    decimator+=1

# imsize = gray.shape

calibrationData = [allCorners, allIds, [640,480]]
pickle.dump(calibrationData, open( "calibrationData.p", "wb" ))
