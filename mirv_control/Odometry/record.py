#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from odometrystate import State
import math

state = State()

def encoder_velocity_callback(velocity):
    v = velocity.data
    state.encoder_vel = v

def gps_callback(pose):
    state.gps_x = pose.data[0]
    state.gps_y = pose.data[1]
    state.imu_angle = pose.data[2]

def run():
    rospy.init_node("record")

    encoder_velocity_sub = rospy.Subscriber("encoder/velocity", Float64, encoder_velocity_callback)
    gps_sub = rospy.Subscriber("GPS/IMU/POS", Float64MultiArray, gps_callback)

    rospy.spin()
    state.close_file()

if __name__ == "__main__":
    run()