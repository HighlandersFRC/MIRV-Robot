#!/usr/bin/env python
import time
import os
import subprocess
import rospy
from std_msgs.msg import String

# Pin Definitions
DO0=320
DO1=321
DO2=322
DO3=323
outputsDict = {"DO0": DO0, "DO1": DO1, "DO2": DO2, "DO3": DO3}
def digitalWrite(pin, state):
    print("tunring pin {} to {}".format(pin, state))
    subprocess.call(["echo {} >/sys/class/gpio/gpio{}/value".format(state, pin)], shell=True)
def callback(data):
    inputData = data.split(",", 1)
    digitalWrite(outputsDict[inputData[0]], inputData[1])

def listen():
    rospy.Subscriber("DOControl", String, callback)
    while not rospy.is_shutdown():
        rospy.spin()

def main():
    rospy.init_node("DOController", anonymous=True)
    listen()
if __name__ == '__main__':
    main()
