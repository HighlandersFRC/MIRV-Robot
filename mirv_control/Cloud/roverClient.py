#!/usr/bin/env python3
import socketio
import rospy
from std_msgs.msg import Float64
import os

HOST = os.getenv("API_URL")

sio = socketio.Client()


def sendData(data):
    sio.emit("data", data)


def battery_voltage_callback(voltage):
    print("sending voltage")
    sendData({"battery-voltage": voltage.data})


def run():
    rospy.init_node("roverClient")
    try:
        sio.connect(f"{HOST}/ws", headers={"roverID": "mirv_1"},
                    auth={"password": None}, socketio_path="/ws/socket.io")
    except:
        print("Could not connect")
    battery_voltage_sub = rospy.Subscriber(
        "battery/voltage", Float64, battery_voltage_callback)
    rospy.spin()


if __name__ == "__main__":
    run()
