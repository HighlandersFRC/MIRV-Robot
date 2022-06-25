#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from odometrystate import State

state = State()

def encoder_velocity_callback(velocity):
    state.encoder_vel = velocity.data
    state.update_xy_vel()

def gps_callback(pose):
    state.gps_x = pose.data[0]
    state.gps_y = pose.data[1]
    state.imu_angle = pose.data[2]

def record_callback(idk):
    state.record()

def run():
    rospy.init_node("record")

    encoder_velocity_sub = rospy.Subscriber("encoder/velocity", Float64, encoder_velocity_callback)
    gps_sub = rospy.Subscriber("GPS/IMUPOS", Float64MultiArray, gps_callback)

    record_timer = rospy.Timer(rospy.Duration(1.0 / 50.0), record_callback)

    rospy.spin()
    state.close_file()

if __name__ == "__main__":
    run()