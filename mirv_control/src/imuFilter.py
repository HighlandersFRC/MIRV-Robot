#!/usr/bin/env python3

import roslib
import rospy
import sys
import time
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

import mirv_control.helpful_functions_lib as conversion_lib


class imuFilter:

    def __init__(self):

        self.yaw_zero = 0
        self.t = 0

        self.imu_pub = rospy.Publisher("/imu/filtered", Imu, queue_size=5)
        self.imu_sub = rospy.Subscriber("/imu_raw", Imu, self.imu_cb)
        self.imu_dumb_sub = rospy.Subscriber("/CameraIMU", Float64, self.imu_dumb_cb)

    def imu_cb(self, msg):
        if self.t == 0:
            self.yaw_zero = conversion_lib.quat_from_pose2eul((msg.orientation+180)%360)[0]
            self.t = 1
        orientation = conversion_lib.quat_from_pose2eul(msg.orientation)
        orientation[0] -= self.yaw_zero
        msg.orientation = conversion_lib.euler2quat_from_pose(msg.orientation, orientation[:, None])
        self.imu_pub.publish(msg)

    def imu_dumb_cb(self, data):
        if self.t == 0:
            self.yaw_zero =(((-data.data +180)%360) * np.pi/180)
            self.t = 1
        msg = Imu()
        orientation = np.array([-data.data * np.pi/180, 0, 0])
        orientation[0] -= self.yaw_zero
        msg.orientation = conversion_lib.euler2quat_from_pose(msg.orientation, orientation[:, None])
        rospy.loginfo(msg)
        self.imu_pub.publish(msg)


def main():
    rospy.init_node('imu_filter', anonymous=True)
    imu = imuFilter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        quit()


if __name__ == '__main__':
    main()
