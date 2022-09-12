#!/usr/bin/env python3
import queue
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
        GPSSub = rospy.Subscriber("GPS/IMUPOS", Float64MultiArray, self.GPSCallback)
        IMUSub = rospy.Subscriber("pigeonIMU", Float64, self.IMUcallback)
        timeSinceUpdate  = time.time()
    def run(self):
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
        leftVel = self.leftVel
        rightVel = self.rightVel
        if(leftVel == 0 and rightVel == 0):
            print("State: No Change")
            xP = self.xPos
            yP = self.yPos
        elif(abs(leftVel - rightVel)<0.02):
            print("State: move forward or back")
            xP = self.xPos+(leftVel*runTime*math.cos(self.heading))
            yP = self.yPos+(leftVel*runTime*math.sin(self.heading))
            print("{}, {}".format(xP, yP))
        else:
            print("Sate: Turning")
            radius = (1/2)*((rightVel+leftVel)/(rightVel-leftVel))
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
        self.heading = self.startingTheta-math.radians(temp)
    def encoderCallback(self,data):
        temp = data.data
        self.leftVel = temp[0]
        self.rightVel = -temp[1]
    def GPSCallback(self, data):
        temp = data.data
        self.GPSXPos = temp[0]
        self.GPSYPos = temp[1]
    def updateQueue(self):
        tempVel = self.averageVel*self.velQueue.qsize()
        if(self.velQueue.full()):
            tempVel = tempVel - self.velQueue.get()
        currentAvgVel = (self.leftVel+self.rightVel)/2
        self.velQueue.put(currentAvgVel)
        tempVel = tempVel + currentAvgVel
        self.averageVel =  tempVel/self.velQueue.qsize()
    def kalmanFilter(self):
        kinProb = self.kinematicsConfidence()*self.kinematicsTrust
        GPSProb = self.GPSConfidence()*self.GPSTrust
        NKP = kinProb/(kinProb+GPSProb)
        NGP = GPSProb/(kinProb+GPSProb)
        print("GPS Point: {}, {} GPS Prob: {} KinimaticsPoint: {}, {}, kinimatics Prob: {}".format(self.GPSXPos, self.GPSYPos, NGP, self.xPos, self.yPos, NKP))
        self.filterX = self.xPos*NKP + self.GPSXPos*NGP
        self.filterY = self.yPos*NKP + self.GPSYPos*NGP
        self.xPos = self.filterX
        self.yPos = self.filterY
    def GPSConfidence(self):
        temp = 1/((abs(self.averageVel)+1))
        print(self.averageVel)
        return temp
    def kinematicsConfidence(self):
        diffTemp = abs(self.leftVel-self.rightVel)
        diffTemp = 1/(diffTemp+1)
        moveTemp = 1/(0.5*self.averageVel+1)
        temp = (diffTemp + moveTemp)/2
        return temp
if __name__ == "__main__":
    test = Filter()
    test.run()