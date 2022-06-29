#!/usr/bin/env python
import math
import numpy as np
from numpy import array
import time
import rospy
import sys
from std_msgs.msg import Float64MultiArray

class PurePursuit():
    wheelBaseWidth = 0.483
    nextPointDistanceDec = 0
    logData = [0,0]

    maxDriveSpeed = 1.5
    currentMaxDriveSpeed = 1
    startingTheta = 3.14159/2
    lookAheadDist = 1
    allowedError = 0.5
    allowedErrorDist = 0.1
    lookAheadDist = 4
    allowedError = 0.05
    robotCordList = [0,0]
    cordList = []
    currentTruckCord = [0,0,0]
    rosPubMsg = Float64MultiArray()
    pub = rospy.Publisher("VelocityDrive", Float64MultiArray, queue_size = 10)

    def __init__(self):
        rospy.init_node('PurePursuitController', anonymous=True)
    ##updates truck coordinate targets to rover cordites as rover moves
    def UpdateTargetPoints(self):
        newTruckCord = self.getTruckCord()
        theta = (3.1415926*2 - newTruckCord[2] + 3.14159/2)%(2*3.1415926) - self.startingTheta
        xBRef = newTruckCord[0]
        yBRef = newTruckCord[1]
        # print([xBRef, yBRef, theta])
        length = 0
        for i in range(len(self.robotCordList)):

            xTBody = self.cordList[i][0]*math.cos(theta) + self.cordList[i][1]*math.sin(theta) - yBRef*math.sin(theta) - xBRef*math.cos(theta)
            yTBody = -self.cordList[i][0]*math.sin(theta) + self.cordList[i][1]*math.cos(theta) - yBRef*math.cos(theta) + xBRef*math.sin(theta)
            distToPoint = round(math.sqrt(math.pow(xTBody,2) + math.pow(yTBody,2)), 3)
            self.robotCordList[i] = [xTBody, yTBody, distToPoint]
        print("target X,Y: {}, CurrentTheta: {}".format(self.robotCordList[0], theta))
        print("targetTruck X,Y: {}".format(self.cordList[0]))
    ## helper methods for update target points
    def getTruckCord(self):
        return self.currentTruckCord


    def getPath(self):
        pathList = [[0,5]]
        for point in pathList:
            self.cordList.append([point[0],point[1],0])
            self.robotCordList.append([point[0],point[1],0])
        print("uploaded path")
        # self.UpdateTargetPoints()
        self.nextPointDistanceDec = sys.float_info.max
       



    def removeTargetPoint(self):
        if (abs(self.robotCordList[0][2]) < self.nextPointDistanceDec):
            self.nextPointDistanceDec = self.robotCordList[0][2]
        elif (abs(self.robotCordList[0][2]) < (1.5)*self.lookAheadDist and len(self.robotCordList) > 1):
            self.robotCordList.pop(0)
            self.cordList.pop(0)
            if(self.robotCordList):
                self.nextPointDistanceDec = self.robotCordList[0][2]
            else:
                return True
        elif (abs(self.robotCordList[0][2]) < (self.allowedError) and len(self.robotCordList) <= 1):
            self.robotCordList.pop(0)
            self.cordList.pop(0)
            if(self.robotCordList):
                self.nextPointDistanceDec = self.robotCordList[0][2]
            else:
                return True
        return False

    ## calulates wheel velocity's for that given frame
    def calculateSpeedSide(self, maxSpeed, x, y, la):
        leftVel = maxSpeed
        rightVel = maxSpeed
        if(abs(x)>self.allowedErrorDist):
            cRad = self.generateRadius(x, la)
            iCirc = self.generateCircumference((cRad - (self.wheelBaseWidth/2)))
            oCirc = self.generateCircumference((cRad + (self.wheelBaseWidth/2)))
            innerSpeedRatio = (iCirc/oCirc)
            turnRight = True
            if(x>=0):  
                rightVel = rightVel*innerSpeedRatio
                # print("turningRight")
            else:
                leftVel = leftVel*innerSpeedRatio
                # print("turningLeft")
        return ([leftVel, rightVel])

    ##helper methods to calculateSpeedSides
    def generateRadius(self,x,la):
        return(math.pow(la,2)/(abs(2*x)))
    def generateCircumference(self, rad):
        return(2*3.141592*rad)



    ##determines the target cord for each snapshot
    def getTargetCordAndDriveSpeed(self,la, maxSpeed):
        closePoint = self.closestOut(la)
        farPoint = self.furthestIn(la)
        targetPoint = []
        snapshotLa = la
        if (closePoint and farPoint):
            targetPoint = self.doubleVectorIntercept(closePoint, farPoint,la)
        elif farPoint:
            self.maxDriveSpeed = 0.5
            targetPoint = farPoint
            snapshotLa = (targetPoint[0]**2 + targetPoint[1]**2)**0.5
            self.currentMaxDriveSpeed = 0.5
            if (snapshotLa < self.allowedError):
                self.currentMaxDriveSpeed = self.maxDriveSpeed
                return ("atDest")
                
        elif closePoint:
            targetPoint = self.vectorIntercept(closePoint, la)
        
        else:
            print("cannot determine target point")
        sideSpeeds = self.calculateSpeedSide(maxSpeed, targetPoint[0], targetPoint[1], snapshotLa)
        return([sideSpeeds,targetPoint])

    ##helper method for getTargetCord
    def vectorIntercept(self, closePoint, la):
        cPO = array(closePoint)
        normalVect = (cPO)/(((cPO**2).sum()**0.5))
        targetPoint = normalVect*la
        # print("Vector Intercept")
        return targetPoint
    def doubleVectorIntercept(self, closePoint, farPoint, la):
        cPO = array(closePoint)
        fPI = array(farPoint)
        zeroVect= cPO-fPI
        prodOfMag = ((fPI**2).sum()**0.5)*((zeroVect**2).sum()**0.5)
        gama = 3.141592 - math.acos(((zeroVect.dot(fPI))/prodOfMag))
        zeta = math.asin((((fPI**2).sum()**0.5)*math.sin(gama))/(la))
        theta = 3.1415-gama-zeta
        distToEdge = la*(math.sin(theta))/math.sin(gama)
        unitP2Vect = ((cPO-fPI)/(((cPO-fPI)**2).sum()**0.5))
        p2EVect = distToEdge*unitP2Vect
        target = p2EVect+fPI
        # print("double Vector Intercept")
        return target
    def closestOut(self, la):
        closePoint = []
        for point in self.robotCordList:
            if(point[2] > la):
                return [point[0], point[1]]
        return closePoint
    def furthestIn(self, la):
        farPoint = []
        for point in self.robotCordList:
            if(point[2] <= la):
                farPoint = [point[0], point[1]]
            else:
                if(farPoint):
                    return [farPoint[0], farPoint[1]]
        return farPoint

    def callBackOdom(self, data):
        self.currentTruckCord = data.data
        print(data.data)
        if (self.cordList):
            self.UpdateTargetPoints()
            output = self.getTargetCordAndDriveSpeed(self.lookAheadDist, self.currentMaxDriveSpeed)
            isFinished = self.removeTargetPoint()
            if(output != "atDest" and not isFinished):
                # print("{}, {}".format(output[0][0],output[0][1]))
                self.rosPubMsg.data = output[0]
                self.logData = output[0]
                # self.rosPubMsg.data = [2.2, 4]
            else:
                self.logData = [0,0]
                self.rosPubMsg.data = [0,0]


            print(self.rosPubMsg.data)
        
        
        else:
            ## ros.drive.setvel(0,0)
            self.rosPubMsg.data = [0,0]
            print("no target Point")
        
        self.pub.publish(self.rosPubMsg)

    def run(self):

        sub = rospy.Subscriber("GPS/IMUPOS", Float64MultiArray, self.callBackOdom)
        self.getPath()
        rospy.loginfo_throttle(0.5, "Pure pursuit Output: LeftSpeed: {}, RightSpeed: {}".format(self.logData[0], self.logData[1]))
        rospy.spin()

    def __del__(self):
        print("exiting Program")
        self.rosPubMsg.data = [0,0]
        self.pub.publish(self.rosPubMsg)
        time.sleep(3)
        print("exited")



if __name__ == '__main__':
    controller = PurePursuit()
    controller.run()
   
