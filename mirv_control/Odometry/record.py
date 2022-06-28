#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from odometrystate import State
import math

state = State()

def encoder_velocity_callback(velocity):
    state.encoder_vel_left = velocity.data[0]
    state.encoder_vel_right = -velocity.data[1]
    state.update_xy_vel()

def gps_callback(pose):
    state.gps_x = pose.data[0]
    state.gps_y = pose.data[1]
    state.imu_angle = pose.data[2] + math.pi / 2

def velocity_drive_callback(velocities):
    state.target_vel_left = velocities.data[0]
    state.target_vel_right = velocities.data[1]

def record_callback(arg):
    state.record()

def run():
    rospy.init_node("record")

    encoder_velocity_sub = rospy.Subscriber("encoder/velocity", Float64MultiArray, encoder_velocity_callback)
    gps_sub = rospy.Subscriber("GPS/IMUPOS", Float64MultiArray, gps_callback)
    velocity_drive_sub = rospy.Subscriber("VelocityDrive", Float64MultiArray, velocity_drive_callback)

    record_timer = rospy.Timer(rospy.Duration(1.0 / 50.0), record_callback)

    rospy.spin()
    state.close_file()

if __name__ == "__main__":
    run()