#!/usr/bin/env python3

import cv2
import depthai as dai
import time
import math
import rospy
from std_msgs.msg import Float64

pub = rospy.Publisher('CameraIMU', Float64, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate  = rospy.Rate(50)

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
imu = pipeline.createIMU()
xlinkOut = pipeline.createXLinkOut()

xlinkOut.setStreamName("imu")

# enable ROTATION_VECTOR at 400 hz rate
imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 400)
# above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
imu.setBatchReportThreshold(1)
# maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
# if lower or equal to batchReportThreshold then the sending is always blocking on device
# useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
imu.setMaxBatchReports(10)

# Link plugins IMU -> XLINK
imu.out.link(xlinkOut.input)


# Pipeline is defined, now we can connect to the device
with dai.Device(pipeline) as device:

    def quat_2_radians(x, y, z, w):
        pitch = math.atan2(2*x*w - 2*y*z, 1-2*x*x - 2* z*z)
        yaw = math.asin(2*x*y + 2*z*w)
        roll = math.atan2(2*x*w - 2*x*z, 1-2*y*y - 2*z*z)
        return pitch, yaw, roll

    def timeDeltaToMilliS(delta) -> float:
        return delta.total_seconds()*1000

    yAxisI = 0
    yAxisJ = 1

    yAxisMagnitude = 1

    # Output queue for imu bulk packets
    imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
    baseTs = None
    while True:
        imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived

        imuPackets = imuData.packets
        for imuPacket in imuPackets:
            rVvalues = imuPacket.rotationVector

            rvTs = rVvalues.timestamp.get()
            if baseTs is None:
                baseTs = rvTs
            rvTs = rvTs - baseTs

            imuF = "{:.06f}"
            tsF  = "{:.03f}"

            rotationI = rVvalues.i
            rotationJ = rVvalues.j
            rotationK = rVvalues.k
            rotationReal = rVvalues.real
            
            pitch, yaw, roll = quat_2_radians(rotationI, rotationJ, rotationK, rotationReal)

            pitch = pitch * 180/math.pi
            yaw = yaw * 180/math.pi
            roll = roll * 180/math.pi


            if(pitch < 0):
                pitch = abs(pitch)
            elif(pitch > 0):
                pitch = 360 - pitch

            if(pitch > 90):
                pitch = pitch - 90
            else:
                pitch = pitch + 270

            print("PITCH: ", pitch)

            pub.publish(pitch)
            rate.sleep()

        if cv2.waitKey(1) == ord('q'):
            break