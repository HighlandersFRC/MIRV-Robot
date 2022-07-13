#!/usr/bin/env python3
import math
import numpy as np
from numpy import array
import time
import rospy
import sys
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


msg = Twist()
pub = rospy.Publisher("cmd_vel", Twist, queue_size = 5)
rospy.init_node('simpleTwist', anonymous=True)
maxASpeed = 0.5
maxLSpeed = 0.5

def makeTwist(forwardDisp, angularDisp):
    Atime = abs(angularDisp/maxASpeed)
    Ltime = abs(forwardDisp/maxLSpeed)
    ASpeed = maxASpeed
    LSpeed = maxLSpeed
    if (forwardDisp < 0):
        LSpeed = -maxLSpeed
    if (angularDisp < 0):
        ASpeed = -maxASpeed
    runTime = 0
    if (Atime <= Ltime ):
        ASpeed = angularDisp/Ltime
        runTime = Ltime
    else:
        LSpeed = forwardDisp/Atime
        runTime = Atime

    msg.linear.x = LSpeed
    msg.angular.z = ASpeed
    pub.publish(msg)
    print(msg)
    time.sleep(runTime)
    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)
    print("Done, ran for {} seconds".format(runTime))

if __name__ == '__main__':
    try:
        while True:
            print("enter a forward target: ")
            Lin = float(input())
            print("enter an angular target: ")
            ang = float(input())
            makeTwist(Lin, ang)
    except:
        print("an error has occurred")


