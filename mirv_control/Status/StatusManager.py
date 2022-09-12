#!/usr/bin/env python3 
import rospy
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState, NavSatFix
from nav_msgs.msg import Odometry
from mirv_control.msg import depth_and_color_msg as Frames
from mirv_control.msg import pilit_status_msg as PilitStatus
from mirv_control.msg import garage_state_msg as garage_state
import helpful_functions_lib as conversion
from roverstate import RoverState
import json
import subprocess as sp
import time
import math

rover_state = RoverState()
voltages = []

def battery_voltage_callback(voltage):
    
    if len(voltages) > 30:
        voltages.pop(0)
    
    voltages.append(voltage.data)
    
    v_avg = 0
    canbus_failure = True
    for v in voltages:
        v_avg +=v
        if v != voltage.data:
            canbus_failure = False
    v_avg = v_avg / len(voltages)
        
    
    rover_state.rover_state["battery_voltage"] = int(v_avg)
    rover_state.rover_state["battery_percent"] = int(((v_avg - 12) / (12.6 - 12)) * 100)
    if rover_state.rover_state["battery_percent"] > 100:
        rover_state.rover_state["battery_percent"] = 100
    elif rover_state.rover_state["battery_percent"] < 0:
        rover_state.rover_state["battery_percent"] = 0
        rover_state.rover_state["subsystems"]["power"] = {"health": "unhealthy", "details": "Battery Voltage is outside of operating thresholds."}
    if voltage.data < 5 or canbus_failure:
        rover_state.rover_state["subsystems"]["drivetrain"] = {"health": "unavailable", "details": "Canbus is Offline."}
        rover_state.rover_state["subsystems"]["intake"] = {"health": "unavailable", "details": "Canbus is Offline."}
        rover_state.rover_state["subsystems"]["power"] = {"health": "unavailable", "details": "Canbus is Offline."}
    else:
        rover_state.rover_state["subsystems"]["drivetrain"] = {"health": "healthy"}
        rover_state.rover_state["subsystems"]["intake"] = {"health": "healthy"}
        rover_state.rover_state["subsystems"]["power"] = {"health": "healthy"}
        
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
    rover_state.rover_state["telemetry"]["heading"] = math.degrees(conversion.quat_from_pose2eul(odometry.pose.pose.orientation)[0])
    rover_state.timers["heading"].reset()

def state_callback(state):
    rover_state.rover_state["state"] = state.data

def status_callback(status):
    rover_state.rover_state["status"] = status.data

def pilit_state_callback(state):
    rover_state.timers["pilit_table"].reset()
    rover_state.rover_state["pi_lits"]["pi_lits_stowed_right"] = state.right_count.data
    rover_state.rover_state["pi_lits"]["pi_lits_stowed_left"] = state.left_count.data
    rover_state.rover_state["pi_lits"]["deployed_pi_lits"] = [{"pi_lit_id": f"PiLit_{state.ids.data[i]}", "location":{"lat": state.latitudes.data[i], "long": state.longitudes.data[i], "elev": state.altitudes.data[i]}} for i in range(len(list(zip(state.latitudes.data, state.longitudes.data, state.altitudes.data))))]

def pilit_mode_callback(mode):
    rover_state.rover_state["pi_lits"]["state"] = mode.data

def garage_callback(pos: NavSatFix):
    rover_state.rover_state["garage"]["location"]["lat"] = pos.latitude
    rover_state.rover_state["garage"]["location"]["long"] = pos.longitude
    
def garage_status_callback(status):
    rover_state.rover_state["subsystems"]["garage"] = {"health": status.health}
    rover_state.timers["garage"].reset()

def update_general(timer_event):
    status = {"health": "healthy", "details":""}
    rover_state.rover_state["subsystems"]["general"] = status
    for key in rover_state.rover_state["subsystems"]:
        health_state = rover_state.rover_state["subsystems"][key]["health"]
        if health_state in ("unhealthy", "degraded", "unavailable"):
            status["health"] = "unhealthy"
            status["details"] += f"{key} subsystem is in state: {health_state}. "
        
        
    if not "OpenMoko" in sp.getoutput("lsusb"):
        rospy.logwarn("CAN adapter is not connected")
        status["health"] = "unhealthy"
        status["details"] += "Can Adapter not found. "
        
    elif "DOWN" in sp.getoutput("ip link show can0"):
        rospy.logwarn("can0 network is DOWN")
        status["health"] = "unhealthy"
        status["details"] += "Can0 network is down. "
        

        
    rover_state.rover_state["subsystems"]["general"] = status

def publish_status(timer_event):
    rover_state.update_timestamp()
    msg = json.dumps(rover_state.rover_state)
    rospy.loginfo(msg)
    status_pub.publish(msg)

rospy.init_node("StatusManager")

#Subscribers
battery_voltage_sub = rospy.Subscriber("battery/voltage", Float64, battery_voltage_callback, queue_size = 1)
gps_sub = rospy.Subscriber("gps/fix", NavSatFix, gps_callback, queue_size = 1)
encoder_sub = rospy.Subscriber("encoder/velocity", JointState, encoder_callback, queue_size = 1)
camera_frames_sub = rospy.Subscriber("CameraFrames", Frames, camera_frames_callback, queue_size = 1)
heading_sub = rospy.Subscriber("EKF/Odometry", Odometry, heading_callback, queue_size = 1)
pilit_state_sub = rospy.Subscriber("pilit/status", PilitStatus, pilit_state_callback, queue_size = 1)
pilit_mode_sub = rospy.Subscriber("pilit/mode", String, pilit_mode_callback, queue_size = 1)
status_sub = rospy.Subscriber("RoverAvailable", String, status_callback, queue_size = 1)
garage_location_sub = rospy.Subscriber("/garage/locations", NavSatFix, garage_callback, queue_size = 1)
garage_status_sub = rospy.Subscriber("GarageStatus", garage_state, garage_status_callback,queue_size=1)

state_sub = rospy.Subscriber("RoverState", String, state_callback, queue_size = 1)



#Status publisher and timer
status_pub = rospy.Publisher("RoverStatus", String, queue_size = 1)
status_pub_timer = rospy.Timer(rospy.Duration(2), publish_status)

#General update timer
update_general_timer = rospy.Timer(rospy.Duration(5), update_general)

while not rospy.is_shutdown():
    #For each timer update the respective health substatus based on if the timer is timed out
    for key in rover_state.timers:
        is_healthy = rover_state.timers[key].update()
        if is_healthy:
            state = {"health": "healthy", "details":""}
        else:
            state = {"health": "degraded", "details":""}
        if key == "battery_voltage":
            if not is_healthy:
                state['details'] += "Unable to retrieve battery voltage. "
                rover_state.rover_state["subsystems"]["power"] = state
        elif key == "gps":
            if not is_healthy:
                state["details"] += "GPS is not broadcasting. "
            rover_state.rover_state["subsystems"]["sensors"] = state
        elif key == "camera_frames":
            if not is_healthy:
                state["details"] += "Forward camera is not broadcasting. "
            rover_state.rover_state["subsystems"]["sensors"] = state
        elif key == "heading":
            if not is_healthy:
                state["details"] += "IMU is not broadcasting. "
            rover_state.rover_state["subsystems"]["sensors"] = state
        elif key == "encoders":
            if not is_healthy:
                state["details"] += "Drivetrain Encoders are not being broadcast"
            rover_state.rover_state["subsystems"]["sensors"] = state
        elif key == "garage":
            if not is_healthy:
                state["details"] += "Cannot get status updates from Garage. "
            rover_state.rover_state["subsystems"]["garage"] = state
    time.sleep(0.1)
