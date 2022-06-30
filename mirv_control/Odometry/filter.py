#!/usr/bin/env python
import time
from turtle import heading
import rospy
import math
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

class Filter:
    maxVelocity = 4.8
    GPSTrust = 0.5
    kinematicsTrust = 0.5
    heading = 0
    filterX = 0
    filterY = 0
    xPos = 0
    yPos = 0
    GPSXPos = 0
    GPSYPos = 0
    leftVel = 0
    rightVel = 0
    timeSinceUpdate = 0
    lastTheta = 0
    startingTheta = 0
    setStartTheta = False
    WBWidth = 0.483
    refreshRate = 20
    backlogMemSec = 5 
    sizeOfQueue = refreshRate*backlogMemSec
    velQueue = queue.Queue(maxsize = sizeOfQueue)
    averageVel = 0

    def __init__(self):
        rospy.init_node('Filter', anonymous=True)
        encoderSub = rospy.Subscriber("encoder/velocity", Float64MultiArray, self.encoderCallback)
        GPSSub = rospy.Subscriber("GPS/IMUPOS", Float64MultiArray, self.encoderCallback)
        IMUSub = rospy.Subscriber("CameraIMU", Float64, self.IMUcallback)
        timeSinceUpdate  = time.time()
    def run(self):
        print("one")
        rate = rospy.Rate(self.refreshRate)
        while not rospy.is_shutdown():
            self.computePos()
            self.updateQueue()
            self.kalmanFilter()
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
    def GPSCallback(self, data):
        temp = data.data
        self.GPSXPos = temp[0]
        self.GPSYPos = temp[1]
    def updateQueue(self, data):
        tempVel = self.averageVel*self.velQueue.qsize()
        if(self.velQueue.full())
            tempVel = tempVel - self.velQueue.get()
        currentAvgVel = (self.leftVel+self.rightVel)/2
        velQueue.put(currentAvgVel)
        tempVel = tempVel + currentAvgVel
        self.averageVel =  tempVel
    def kalmanFilter(self):
        kinProb = self.kinematicsConfidence()*self.kinematicsTrust
        GPSProb = self.GPSConfidence()*self.GPSTrust
        NKP = kinProb/(kinProb+GPSProb)
        NGP = GPSProb/(kinProb+GPSProb)
        self.filterX = self.xPos*NKP + self.GPSXPos*NGP
        self.filterY = self.yPos*NKP + self.GPSYPos*NGP
        self.xPos = self.filterX
        self.yPos = self.filterY
    def GPSConfidence(self):
        temp = 1/((abs(self.averageVel)+1)/self.maxVelocity)
        temp = temp/self.maxVelocity
        return temp
    def kinematicsConfidence(self):
        diffTemp = abs(leftVel-rightVel)
        diffTemp = 1/(diffTemp+1)
        moveTemp = 1/(0.5*self.averageVel+1)
        temp = (diffTemp + moveTemp)/2
        return temp
if __name__ == "__main__":
    test = Filter()
    test.run()