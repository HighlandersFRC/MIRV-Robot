#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState, NavSatFix
from nav_msgs.msg import Odometry
from mirv_control.msg import depth_and_color_msg as Frames
from mirv_control.msg import pilit_status_msg as PilitStatus
import helpful_functions_lib as conversion
from roverstate import RoverState
import json
import subprocess as sp
import time

rover_state = RoverState()

def battery_voltage_callback(voltage):
    rover_state.rover_state["battery_voltage"] = int(voltage.data)
    rover_state.rover_state["battery_percent"] = int(((voltage.data - 10.5) / (12.6 - 10.5)) * 100)
    if voltage.data < 5:
        rover_state.rover_state["health"]["drivetrain"] = "unavailable"
        rover_state.rover_state["health"]["intake"] = "unavailable"
    rover_state.timers["battery_voltage"].reset()

def gps_callback(gps_pos):
    rover_state.rover_state["telemetry"]["location"]["lat"] = gps_pos.latitude
    rover_state.rover_state["telemetry"]["location"]["long"] = gps_pos.longitude
    rover_state.timers["gps"].reset()

def encoder_callback(velocities):
    rover_state.rover_state["telemetry"]["speed"] = (velocities.velocity[0] + velocities.velocity[1]) / 2
    rover_state.timers["encoders"].reset()

def camera_frames_callback(frames):
    rover_state.timers["camera_frames"].reset()

def heading_callback(odometry):
    rover_state.rover_state["telemetry"]["heading"] = conversion.quat_from_pose2eul(odometry.pose.pose.orientation)[0]
    rover_state.timers["heading"].reset()

def state_callback(state):
    rover_state.rover_state["state"] = state.data

def status_callback(status):
    rover_state.rover_state["state"] = status.data

def pilit_state_callback(state):
    rover_state.timers["pilit_table"].reset()
    rover_state.rover_state["pi_lits"]["pi_lits_stowed_right"] = state.right_count.data
    rover_state.rover_state["pi_lits"]["pi_lits_stowed_left"] = state.left_count.data
    rover_state.rover_state["pi_lits"]["deployed_pi_lits"] = [{"lat": state.latitudes.data[i], "long": state.longitudes.data[i], "elev": state.altitudes.data[i]} for i in range(len(state.latitudes.data))]

def pilit_mode_callback(mode):
    rover_state.rover_state["pi_lits"]["state"] = mode.data

def update_general(timer_event):
    if any(value in rover_state.rover_state["health"].values() for value in ("unhealthy", "degraded", "unavailable")):
        rover_state.rover_state["health"]["general"] = "unhealthy"
    else:
        rover_state.rover_state["health"]["general"] = "healthy"
    if not "OpenMoko" in sp.getoutput("lsusb"):
        rospy.logwarn("CAN adapter is not connected")
        rover_state.rover_state["health"]["drivetrain"] = "degraded"
        rover_state.rover_state["health"]["intake"] = "degraded"
        rover_state.rover_state["health"]["electronics"] = "degraded"
    elif "DOWN" in sp.getoutput("ip link show can0"):
        rospy.logwarn("can0 network is DOWN")
        rover_state.rover_state["health"]["drivetrain"] = "unavailable"
        rover_state.rover_state["health"]["intake"] = "unavailable"

def publish_status(timer_event):
    rover_state.update_timestamp()
    msg = json.dumps(rover_state.rover_state)
    rospy.loginfo(msg)
    status_pub.publish(msg)

rospy.init_node("StatusManager")

#Subscribers
battery_voltage_sub = rospy.Subscriber("battery/voltage", Float64, battery_voltage_callback)
gps_sub = rospy.Subscriber("gps/fix", NavSatFix, gps_callback)
encoder_sub = rospy.Subscriber("encoder/velocity", JointState, encoder_callback)
camera_frames_sub = rospy.Subscriber("CameraFrames", Frames, camera_frames_callback)
heading_sub = rospy.Subscriber("EKF/Odometry", Odometry, heading_callback)
pilit_state_sub = rospy.Subscriber("pilit/status", PilitStatus, pilit_state_callback)
pilit_mode_sub = rospy.Subscriber("pilit/mode", String, pilit_mode_callback)
status_sub = rospy.Subscriber("RoverAvailable", String, status_callback)



#Status publisher and timer
status_pub = rospy.Publisher("RoverStatus", String, queue_size = 10)
status_pub_timer = rospy.Timer(rospy.Duration(5), publish_status)

#General update timer
update_general_timer = rospy.Timer(rospy.Duration(5), update_general)

while not rospy.is_shutdown():
    #For each timer update the respective health substatus based on if the timer is timed out
    for key in rover_state.timers:
        is_healthy = rover_state.timers[key].update()
        if is_healthy:
            state = "healthy"
        else:
            state = "degraded"
        if key == "battery_voltage":
            rover_state.rover_state["health"]["power"] = state
        elif key == "gps":
            rover_state.rover_state["health"]["sensors"] = state
        elif key == "camera_frames":
            rover_state.rover_state["health"]["sensors"] = state
        elif key == "heading":
            rover_state.rover_state["health"]["sensors"] = state
        elif key == "encoders":
            rover_state.rover_state["health"]["drivetrain"] = state
    time.sleep(0.1)
