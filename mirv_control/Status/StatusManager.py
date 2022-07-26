#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Float64MultiArray, String
from mirv_description.msg import depth_and_color_msg as Frames
from mirv_description.msg import pilit_status_msg as PilitStatus
from roverstate import RoverState
import json

rover_state = RoverState()

def battery_voltage_callback(voltage):
    rover_state.rover_state["battery_voltage"] = voltage.data
    rover_state.rover_state["battery_percent"] = ((voltage.data - 10.5) / (12.6 - 10.5)) * 100
    rover_state.timers["battery_voltage"].reset()

def gps_callback(lat_long):
    rover_state.rover_state["telemetry"]["location"]["lat"] = lat_long.data[0]
    rover_state.rover_state["telemetry"]["location"]["long"] = lat_long.data[1]
    rover_state.timers["gps"].reset()

def encoder_callback(velocities):
    rover_state.rover_state["telemetry"]["speed"] = (velocities.data[0] + velocities.data[1]) / 2
    rover_state.timers["encoders"].reset()

def camera_frames_callback(frames):
    rover_state.timers["camera_frames"].reset()

def camera_imu_callback(angle):
    rover_state.rover_state["telemetry"]["heading"] = angle.data
    rover_state.timers["camera_imu"].reset()

def state_callback(state):
    rover_state.rover_state["state"] = state.data

def pilit_callback(state):
    rover_state.timers["pilit_table"].reset()
    rover_state.rover_state["pi_lits"]["pi_lits_stowed_right"] = state.right_count.data
    rover_state.rover_state["pi_lits"]["pi_lits_stowed_left"] = state.left_count.data
    rover_state.rover_state["pi_lits"]["deployed_pi_lits"] = [{"lat": state.latitudes.data[i], "long": state.longitudes.data[i], "elev": state.altitudes.data[i]} for i in range(len(state.latitudes.data))]

def publish_status(timer_event):
    msg = json.dumps(rover_state.rover_state)
    rospy.loginfo(msg)
    status_pub.publish(msg)

rospy.init_node("StatusManager")

#Subscribers
battery_voltage_sub = rospy.Subscriber("battery/voltage", Float64, battery_voltage_callback)
gps_sub = rospy.Subscriber("GPSCoordinates", Float64MultiArray, gps_callback)
encoder_sub = rospy.Subscriber("encoder/velocity", Float64MultiArray, encoder_callback)
camera_frames_sub = rospy.Subscriber("CameraFrames", Frames, camera_frames_callback)
camera_imu_sub = rospy.Subscriber("CameraIMU", Float64, camera_imu_callback)
pilit_state_sub = rospy.Subscriber("pilit/status", PilitStatus, pilit_callback)

#Status publisher and timer
status_pub = rospy.Publisher("RoverStatus", String, queue_size = 10)
status_pub_timer = rospy.Timer(rospy.Duration(5), publish_status)

while not rospy.is_shutdown():
    #For each timer update the respective health substatus based on if the timer is timed out
    for key in rover_state.timers:
        is_healthy = rover_state.timers[key].update()
        if is_healthy:
            state = "healthy"
        else:
            state = "unavailable"
        if key == "battery_voltage":
            rover_state.rover_state["health"]["power"] = state
        elif key == "gps":
            rover_state.rover_state["health"]["sensors"] = state
        elif key == "camera_frames":
            rover_state.rover_state["health"]["sensors"] = state
        elif key == "camera_imu":
            rover_state.rover_state["health"]["sensors"] = state
        elif key == "encoders":
            rover_state.rover_state["health"]["drivetrain"] = state