!/usr/bin/env python

import roslib
import rospy
import sys
import time
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32
from scipy import linalg
import conversion_lib


class AckEkf:

    def __init__(self):

        # Get params from launch file
        self.update_frequency = rospy.get_param('~frequency', 10)
        self.sensor_timeout = 1 / self.update_frequency
        # Define vehicle dimensions - wheel radius
        self.r = rospy.get_param('~wheel_radius', 0.1905)
        # Distance between axels
        self.L = rospy.get_param('~wheel_base_width', 0.483)
        self.xf = np.array(rospy.get_param('~inital_state', [0, 0, 0]))

        # # Get params from launch file
        # self.update_frequency = 10
        # self.sensor_timeout = 2.0
        # # Define vehicle dimensions - wheel radius
        # self.r = 0.07
        # # Distance between axels
        # self.L = 0.36
        # self.xf = [0, 0, 0, 0, 0, 0, 0, 0]

        # GPS reference point and time variable to set GPS_zero to the first measured coordinate
        self.gps_x = 0
        self.gps_y = 0

        # Vehicle wheel angle
        self.p = 0
        # Vehicle forward velocity
        self.LeftWheelVel = 0
        self.RightWheelVel = 0
        # magnetometer yaw measurement
        self.mag_yaw = 0
        self.mag_zero = 0

        # Measured state
        self.z = np.zeros(3)

        # Measurement matrix
        self.H = None
        self.H_all = np.eye(3)
        self.H_gps = np.zeros((2, 3))
        self.H_gps[0, 0] = 1
        self.H_gps[1, 1] = 1
        self.H_imu = np.zeros((1, 3))
        self.H_imu[0, 2] = 1
        self.H_none = np.eye(3)

        # Measurement covariance
        gps_std = 0.25
        # gps_vel_std = 0.25
        magnetometer_std = 0.05
        self.R = None
        self.R_all = np.diag([gps_std**2, gps_std**2, magnetometer_std])
        self.R_gps = np.diag([gps_std**2, gps_std**2])
        self.R_imu = np.diag([magnetometer_std])
        self.R_none = np.diag([gps_std**2, gps_std**2, magnetometer_std])

        # Vehicle motion model covariance
        xy_std = 0.05
        yaw_std = 0.05
        self.Q = np.diag([xy_std**2, xy_std**2, yaw_std**2])

        # Estimate covariance
        self.P = np.diag([xy_std**2, xy_std**2, yaw_std**2])

        # Estimated state
        self.xp = np.zeros(3)

        # Store last time for dt calculation
        self.last_time = rospy.Time.now().to_sec()
        self.last_imu_time = rospy.Time.now().to_sec()
        self.last_gps_time = rospy.Time.now().to_sec()

        time.sleep(5)
        # Define publishers and subscribers
        # Publishes the current [x, y, theta] state estimate
        self.pose_pub = rospy.Publisher("/EKF/Odometry", Odometry, queue_size=5)

        # Computes the current state estimate from the gps data
        self.gps_sub = rospy.Subscriber("/gps/odom", Odometry, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_cb)

        # Updates the local speed and wheel angle
        self.velocity_sub = rospy.Subscriber("/encoder/velocity", JointState, self.velocity_callback)

    def kalman_update(self):
        # Compute time delta between last measurement
        dt = rospy.Time.now().to_sec() - self.last_time
        self.last_time = rospy.Time.now().to_sec()
        dd = self.r * dt

        # Check sensor last measurement times
        if self.last_time - self.last_gps_time < self.sensor_timeout:
            if self.last_time - self.last_imu_time < self.sensor_timeout:
                # We have data from both sensors
                self.H = self.H_all
                self.z = np.zeros(3)
                self.z[0] = self.gps_x
                self.z[1] = self.gps_y
                self.R = self.R_all
                if np.abs(self.mag_yaw - self.xf[2]) < np.pi:
                    self.z[2] = self.mag_yaw
                else:
                    if self.mag_yaw - self.xf[2] > np.pi:
                        self.z[2] = self.mag_yaw - 2 * np.pi
                    else:
                        self.z[2] = self.mag_yaw + 2 * np.pi
            else:
                # We only have GPS data
                self.H = self.H_gps
                self.z = np.zeros(2)
                self.z[0] = self.gps_x
                self.z[1] = self.gps_y
                self.R = self.R_gps
        elif self.last_time - self.last_imu_time < self.sensor_timeout:
            # We only have IMU data
            self.H = self.H_imu
            self.z = np.zeros(1)
            self.R = self.R_imu
            if np.abs(self.mag_yaw - self.xf[2]) < np.pi:
                self.z[0] = self.mag_yaw
            else:
                if self.mag_yaw - self.xf[2] > np.pi:
                    self.z[0] = self.mag_yaw - 2 * np.pi
                else:
                    self.z[0] = self.mag_yaw + 2 * np.pi
        else:
            print('no sensor data')
            # We have no sensor data
            self.H = self.H_none
            self.z = None
            self.R = self.R_none

        if self.last_time - self.last_gps_time > self.sensor_timeout:
            print('bad GPS data')

        # Compute one step of Kalman filter
        # State prediction
        fwd_vel = self.LeftWheelVel + self.RightWheelVel
        ang_vel = self.RightWheelVel - self.LeftWheelVel
        self.xp[0] = self.xf[0] + r/2 * dt * fwd_vel * np.cos(self.xf[2])
        self.xp[1] = self.xf[1] + r/2 * dt * fwd_vel * np.sin(self.xf[2])
        self.xp[2] = self.xf[2] + r/2 * dt/L * ang_vel
        # print(self.xp)
        # print(self.xf)
        # print()

        # Compute the motion jacobian H
        F1 = [1, 0, -r/2 * dt * fwd_vel * np.sin(self.xf[2])]
        F2 = [0, 1, r/2 * dt * fwd_vel * np.cos(self.xf[2])]
        F = np.array([F1, F2, [0, 0, 1]])


        if self.z is None:
            # We had no measurements, just do the motion update
            pp = np.dot(F, np.dot(self.P, F.T)) + self.Q
            self.xf = self.xp
            self.P = pp
        else:
            # We have measurements, compute the filter normally
            pp = np.dot(F, np.dot(self.P, F.T)) + self.Q
            y = self.z - np.dot(self.H, self.xp)
            S = np.dot(self.H, np.dot(pp, self.H.T)) + self.R
            SI = linalg.inv(S)
            kal = np.dot(pp, np.dot(self.H.T, SI))
            self.xf = self.xp + np.dot(kal, y)
            self.P = pp - np.dot(kal, np.dot(self.H, pp))

        # Wrap the angle between -pi, pi
        while self.xf[2] > np.pi:
            self.xf[2] -= 2*np.pi
        while self.xf[2] < -np.pi:
            self.xf[2] += 2*np.pi

        # Publish the Pose estimate
        odom = Odometry()
        odom.header.frame_id = 'world'
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose = conversion_lib.pose_from_state_3D(self.xf[:, None])
        cov_6x6 = np.zeros((6, 6))
        cov_6x6[0:3, 0:3] = self.P
        cov = conversion_lib.state_cov_to_covariance_matrix(cov_6x6)
        odom.pose.covariance = list(conversion_lib.covariance_to_ros_covariance(cov))
        self.pose_pub.publish(odom)


    def gps_callback(self, data):
        self.last_gps_time = rospy.Time.now().to_sec()
        # Convert the GPS coordinates to meters away from the first recorded coordinate
        self.gps_x = data.pose.pose.position.x
        self.gps_y = data.pose.pose.position.y
            # print(self.gps_x, self.gps_y)

    # W is the vehicle's forward velocity in m/s
    def velocity_callback(self, data):
        # Update w from data
        self.LeftWheelVel = data.velocity[0]
        self.RightWheelVel = data.velocity[1]


    def imu_cb(self, msg):
        self.last_imu_time = rospy.Time.now().to_sec()
        if self.t == 0:
            self.mag_zero = conversion_lib.quat_from_pose2eul(msg.orientation)[0]
            self.t = 1
        self.mag_yaw = conversion_lib.quat_from_pose2eul(msg.orientation)[0] - self.mag_zero
        print(self.mag_yaw, self.mag_zero)


def main():
    rospy.init_node('ack_ekf', anonymous=True)
    ekf = AckEkf()
    r = rospy.Rate(ekf.update_frequency)
    ekf.sensor_timeout = 1
    try:
        while True:
            r.sleep()
            ekf.kalman_update()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()