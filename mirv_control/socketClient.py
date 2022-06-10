#!/usr/bin/env python3

import rospy
from std_msgs import Float64MultiArray

import socket
import json


HOST = "172.0.0.1"
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

class Pose:

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def toJSON(self):
        return self.__dict__

def encoderOdometryCallback(msg):
    pose = Pose(msg.data[0], msg.data[1], msg.data[2])
    s.sendall(json.dumps(pose.toJSON()).encode("utf-8"))

def createSub():
    rospy.Subscriber("odometry/encoder", Float64MultiArray, encoderOdometryCallback)

if __name__ == "__main__":
    rospy.init_node("socketClient")
    createSub()
    rospy.spin()