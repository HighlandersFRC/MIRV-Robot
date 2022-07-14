#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Float64MultiArray, String
from mirv_description.msg import depth_and_color_msg as Frames
from roverstate import RoverState
from datetime import datetime
import json

rover_state = RoverState()

def battery_voltage_callback(voltage):
    rover_state.rover_state["battery-voltage"] = voltage.data
    rover_state.timers["battery-voltage"].reset()

def gps_callback(lat_long):
    rover_state.rover_state["telemetry"]["lat"] = lat_long.data[0]
    rover_state.rover_state["telemetry"]["long"] = lat_long.data[1]
    rover_state.timers["gps"].reset()

def encoder_callback(velocities):
    rover_state.rover_state["telemetry"]["speed"] = (velocities.data[0] + velocities.data[1]) / 2
    rover_state.timers["encoders"].reset()

def camera_frames_callback(frames):
    rover_state.timers["camera-frames"].reset()

def camera_imu_callback(angle):
    rover_state.rover_state["telemetry"]["heading"] = angle.data
    rover_state.timers["camera-imu"].reset()

def state_callback(state):
    rover_state.rover_state["state"] = state.data


def publish_status(timer_event):
    status_pub.publish(json.dumps(rover_state.rover_state))

rospy.init_node("StatusManager")

#Subscribers
battery_voltage_sub = rospy.Subscriber("battery/voltage", Float64, battery_voltage_callback)
gps_sub = rospy.Subscriber("GPSCoordinates", Float64MultiArray, gps_callback)
encoder_sub = rospy.Subscriber("encoder/velocity", Float64MultiArray, encoder_callback)
camera_frames_sub = rospy.Subscriber("CameraFrames", Frames, camera_frames_callback)
camera_imu_sub = rospy.Subscriber("CameraIMU", Float64, camera_imu_callback)
#state_sub = rospy.Subscriber("", String, state_callback)

#Status publisher and timer
status_pub = rospy.Publisher("RoverStatus", String, queue_size = 10)
status_pub_timer = rospy.Timer(rospy.Duration(1), publish_status)

while not rospy.is_shutdown():
    #For each timer update the respective health substatus based on if the timer is timed out
    for key in rover_state.timers:
        if rover_state.timers[key].update():
            rover_state.rover_state["health"][key] = "healthy"
        else:
            rover_state.rover_state["health"][key] = "unavailable"