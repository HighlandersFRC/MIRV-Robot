#!/usr/bin/env python
import time
import os
import subprocess
import rospy
from std_msgs.msg import String

# Pin Definitions
#                    function|      HI       |    LO 
DO0=320 ## tp4       reverse | Rev enable    | Rev Disable
DO1=321 ## header 6  inhibit | Flash disable | Flash enable 
DO2=322 ## header 7  reset   | active low reset with low pulse
DO3=323 ## tp5       pattern | Simultaneous  | Wave
## note: Control board is wiered so binary LOW is HIGH, and binary HIGH is LOW 
outputsDict = {"DO0": DO0, "DO1": DO1, "DO2": DO2, "DO3": DO3}
def digitalWrite(pin, state):
    print("tunring pin {} to {}".format(pin, state))
    subprocess.call(["echo {} >/sys/class/gpio/gpio{}/value".format(state, pin)], shell=True)
def callback(data):
    print(str(data.data))
    inputData = str(data.data).split(",", 1)
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
