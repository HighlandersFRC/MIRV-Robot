#!/usr/bin/env python3
import socketio
import rospy
from std_msgs.msg import Float64
import os

<<<<<<< HEAD
HOST = os.getenv("API_URL")
=======
HOST = os.getenv("API_HOST")
PORT = os.getenv("API_PORT")
>>>>>>> 927f74c5bd2796ac2a5bdcc61248ee987243a12b

sio = socketio.Client()

def sendData(data):
<<<<<<< HEAD
    sio.emit("data", data)
=======
    try:
        sio.emit("data", data)
    except:
        print("Could not send data")
>>>>>>> 927f74c5bd2796ac2a5bdcc61248ee987243a12b

def battery_voltage_callback(voltage):
    print("sending voltage")
    sendData({"battery-voltage": voltage.data})

def run():
    rospy.init_node("roverClient")
    try:
<<<<<<< HEAD
        sio.connect(f"{HOST}/ws", headers={"roverID": "mirv_1"}, auth={"password": None}, socketio_path="/ws/socket.io")
    except:
        print("Could not connect")
=======
        sio.connect(f"http://{HOST}:{PORT}/ws", headers={"roverID": "mirv_1"}, auth={"password": None}, socketio_path="/ws/socket.io")
    except:
        print("Could not connect")
        print(f"{HOST}, {PORT}")
>>>>>>> 927f74c5bd2796ac2a5bdcc61248ee987243a12b
    battery_voltage_sub = rospy.Subscriber("battery/voltage", Float64, battery_voltage_callback)
    rospy.spin()

if __name__ == "__main__":
    run()