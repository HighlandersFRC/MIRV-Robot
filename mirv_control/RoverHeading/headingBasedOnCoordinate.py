#!/usr/bin/env python3
import math
import time
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64


class headingCalc():
    trueNorth = [0,1]
    newCord = [0,0]
    prevCord = [0,0]
    vector = []
    theta = 0.0
    pub = rospy.Publisher("GPSHeading", Float64, queue_size = 2)
    rosPubMsg = Float64()
    def __init__(self):
        rospy.init_node('GPSHeading', anonymous=True)
    def calculate(self, data):
        self.newCord = data.data
        self.newCord = [math.radians(self.newCord[0]),math.radians(self.newCord[1])]
        X = math.cos(self.newCord[0])*math.sin(self.newCord[1]-self.prevCord[1])
        Y = math.cos(self.prevCord[0])*math.sin(self.newCord[0])-math.sin(self.prevCord[0])*math.cos(self.newCord[0])*math.cos(self.newCord[1]-self.prevCord[1])
        B = math.atan2(X,Y)
        self.theta = math.degrees(B)
        self.prevCord = self.newCord
        self.rosPubMsg.data = self.theta
        self.pub.publish(self.rosPubMsg)
        print(self.theta)

calc = headingCalc()
def run():
    sub = rospy.Subscriber("GPSCoordinates", Float64MultiArray, callBack)
    rospy.spin()
def callBack(data):
    calc.calculate(data)

if __name__ == '__main__':
    run()