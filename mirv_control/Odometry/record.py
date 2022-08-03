#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import sys
sys.path.append('./')
sys.path.append('../')
sys.path.append('../../')
import mirv_description.helpful_functions_lib
from odometrystate import State

state = State()


def encoder_velocity_callback(velocity):
    state.encoder_vel_left = velocity.velocity[1]
    state.encoder_vel_right = velocity.velocity[0]


def gps_callback(pose):
    state.gps_x = pose.pose.pose.position.x
    state.gps_y = pose.pose.pose.position.y
    state.imu_angle = pose.data[2] + math.pi / 2


def imu_callback(msg):
    imu_quat = msg.orientation
    euler_zyx = mirv_description.helpful_functions_lib.quat_from_pose2eul(msg.pose)
    state.imu_angle = euler_zyx[0]


def velocity_drive_callback(velocities):
    state.target_vel = velocities.linear.x
    state.target_angular_vel = velocities.angular.z


def record_callback(arg):
    state.record()


def run():
    rospy.init_node("record")

    encoder_velocity_sub = rospy.Subscriber("encoder/velocity", JointState, encoder_velocity_callback)
    gps_sub = rospy.Subscriber("gps/odom", Odometry, gps_callback)
    imu_sub = rospy.Subscriber("imu_raw", Imu, imu_callback)
    velocity_drive_sub = rospy.Subscriber("cmd_vel", Twist, velocity_drive_callback)

    record_timer = rospy.Timer(rospy.Duration(1.0 / 50.0), record_callback)

    rospy.spin()
    state.close_file()


if __name__ == "__main__":
    run()
