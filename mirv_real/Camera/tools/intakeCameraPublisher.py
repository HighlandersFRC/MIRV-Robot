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

# creating imu stream and enabling imu sensor
imu = pipeline.createIMU()
xlinkOut = pipeline.createXLinkOut()
xlinkOut.setStreamName("imu")
imu.enableIMUSensor(depthai.IMUSensor.ARVR_STABILIZED_GAME_ROTATION_VECTOR, 400)
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
monoRight.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_720_P)
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
depthQueue = depthaiDevice.getOutputQueue(name="depth", maxSize=1, blocking=False)
spatialCalcQueue = depthaiDevice.getOutputQueue(name="spatialData", maxSize=1, blocking=False)
spatialCalcConfigInQueue = depthaiDevice.getInputQueue("spatialCalcConfig")
qIsp = depthaiDevice.getOutputQueue("isp", maxSize=1, blocking=False)
frame = None
imuQueue = depthaiDevice.getOutputQueue(name="imu", maxSize=1, blocking=False)
disparityMultiplier = 255 / stereo.getMaxDisparity()
controlQueue = depthaiDevice.getInputQueue('control')
ctrl = depthai.CameraControl()
ctrl.setAutoFocusMode(depthai.CameraControl.AutoFocusMode.AUTO)
ctrl.setAutoWhiteBalanceMode(depthai.CameraControl.AutoWhiteBalanceMode.AUTO)
controlQueue.send(ctrl)

def interrupt_handler(signal, frame):
    # print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, interrupt_handler)

try:
    while(not rospy.is_shutdown()):
        # while not rospy.is_shutdown():
        initTime = time.time()

        # get imu and rgb queue data
        in_rgb = q_rgb.tryGet()
        imuData = imuQueue.get()
        inDepth = depthQueue.get()
        #print("PAST QUEUE GET")
        depthFrame = inDepth.getFrame()
        depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        depthFrameColor = cv2.equalizeHist(depthFrameColor)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_OCEAN)

        # get imu values and publish them
        imuPackets = imuData.packets
        #print("IMU PACKET LENGTH: ", len(imuPackets))
        i = 0
        for imuPacket in imuPackets:
            i += 1
            rVvalues = imuPacket.rotationVector

            rotationI = rVvalues.i
            rotationJ = rVvalues.j
            rotationK = rVvalues.k
            rotationReal = rVvalues.real
            
            # rospy.loginfo(rVvalues)

            pitch, yaw, roll = quat_2_radians(rotationI, rotationJ, rotationK, rotationReal)

            pitch = pitch * 180/math.pi
            yaw = yaw * 180/math.pi
            roll = roll * 180/math.pi

            pitch = pitch - 270

            pitch = pitch + 360
            pitch = pitch%360

            pitch = pitch + 180
            pitch = pitch%360

            # print("PITCH: ", pitch)

            if(i == len(imuPackets) - 1):
                imuPub.publish(pitch)


        if in_rgb is not None and inDepth is not None:
            frame = qIsp.get().getCvFrame()

            # resize frame for neural nets
            resizedFrame = cv2.resize(frame, (640, 480), interpolation = cv2.INTER_LINEAR)

            result=cv2.imwrite(r'src/cameraFrame.jpg', frame)
            if result==True:
                print("SAVED!")
                firstLoop = False
            else:
                print("DIDN'T SAVE")

            # create custom frame message to publish
            framesMessage = depthAndColorFrame()
            framesMessage.depth_frame = br.cv2_to_imgmsg(depthFrame)
            framesMessage.color_frame = br.cv2_to_imgmsg(resizedFrame)
            imgPub.publish(framesMessage)
            endTime = time.time()

        # if cv2.waitKey(1) == ord('q'):
        #         break

except KeyboardInterrupt:
    print("KEYBOARD INTERRUPT")
    # break
    # except:
    #     print("here")
    #     break
