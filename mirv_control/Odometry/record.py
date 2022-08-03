#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState, Imu, NavSatFix
from nav_msgs.msg import _Odometry
from geometry_msgs.msg import Twist
import math
import sys
sys.path.append('./')
sys.path.append('../')
sys.path.append('../../')
import helpful_functions_lib
from odometrystate import State

state = State()


def encoder_velocity_callback(velocity):
    state.encoder_vel_left = velocity.velocity[1]
    state.encoder_vel_right = velocity.velocity[0]


def gps_callback(fix):
    state.gps_lat = fix.latitude
    state.gps_long = fix.longitude

def rtk_callback(fix):
    state.rtk_lat = fix.latitude
    state.rtk_long = fix.longitude

def imu_callback(msg):
    imu_quat = msg.orientation
    euler_zyx = helpful_functions_lib.quat_from_pose2eul(msg.pose)
    state.imu_angle = euler_zyx[0]


def velocity_drive_callback(velocities):
    state.target_vel = velocities.linear.x
    state.target_angular_vel = velocities.angular.z


def record_callback(arg):
    state.record()


def run():
    rospy.init_node("record")

    #encoder_velocity_sub = rospy.Subscriber("encoder/velocity", JointState, encoder_velocity_callback)
    gps_sub = rospy.Subscriber("gps/fix", NavSatFix, gps_callback)
    rtk_sub = rospy.Subscriber("ublox_gps/fix", NavSatFix, rtk_callback)
    #imu_sub = rospy.Subscriber("imu_raw", Imu, imu_callback)
    #velocity_drive_sub = rospy.Subscriber("cmd_vel", Twist, velocity_drive_callback)

    record_timer = rospy.Timer(rospy.Duration(1 / 2), record_callback)

    while not rospy.is_shutdown():
        pass

    state.close_file()


if __name__ == "__main__":
    run()
