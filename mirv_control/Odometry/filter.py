#!/usr/bin/env python
import time
from turtle import heading
import rospy
import math
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

class EncoderOdomety:
    heading = 0
    xPos = 0
    yPos = 0
    leftVel = 0
    rightVel = 0
    timeSinceUpdate = 0
    lastTheta = 0
    startingTheta = 0
    setStartTheta = False
    WBWidth = 0.483

    def __init__(self):
        rospy.init_node('Filter', anonymous=True)
        encoderSub = rospy.Subscriber("encoder/velocity", Float64MultiArray, self.encoderCallback)
        IMUSub = rospy.Subscriber("CameraIMU", Float64, self.IMUcallback)
        timeSinceUpdate  = time.time()
    def run(self):
        print("one")
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.computePos()
            print("X: {} Y: {}".format(self.xPos, self.yPos))
            rate.sleep()

    def computePos(self):
        runTime = time.time()-self.timeSinceUpdate
        self.timeSinceUpdate = time.time()
        print("{}  ,  {}".format(runTime, self.heading))
        if(abs(self.leftVel - self.rightVel)<0.02):
            xP = self.xPos+(self.leftVel*runTime*math.cos(self.heading))
            yP = self.yPos+(self.leftVel*runTime*math.sin(self.heading))
        else:
            radius = (1/2)*((self.rightVel+self.leftVel)/(self.rightVel-self.leftVel))
            print(radius)
            iCCX = (self.xPos - radius*math.sin(self.heading))
            iCCY = (self.yPos + radius*math.cos(self.heading))
            thetaP = self.heading-self.lastTheta
            xP = (math.cos(thetaP)*(self.xPos-iCCX))-(math.sin(thetaP)*(self.yPos-iCCY))+iCCX
            yP = (math.sin(thetaP)*(self.xPos-iCCX))+(math.cos(thetaP)*(self.yPos-iCCY))+iCCY
        self.xPos = xP
        self.yPos = yP

    def IMUcallback(self, data):
        temp = data.data
        if (not self.setStartTheta):
            self.startingTheta = math.radians(temp)
            self.setStartTheta = True
            self.lastTheta = math.radians(temp)
        self.heading = self.startingTheta-math.radians(temp)+3.14159/2
    def encoderCallback(self,data):
        temp = data.data
        self.leftVel = temp[0]
        self.rightVel = -temp[1]

if __name__ == "__main__":
    test = EncoderOdomety()
    test.run()