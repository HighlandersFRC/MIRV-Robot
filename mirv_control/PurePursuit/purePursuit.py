#!/usr/bin/env python
import math
import numpy as np
from numpy import array
import time
import rospy
from std_msgs.msg import Float64MultiArray

class PurePursuit():
    wheelBaseWidth = 0.483
    nextPointDistanceDec = 0
    maxDriveSpeed = 2.5
    currentMaxDriveSpeed = 2
<<<<<<< HEAD
    lookAheadDist = 5
    allowedError = 0.1
=======
    lookAheadDist = 4
    allowedError = 0.05
>>>>>>> d9194df257f13d185327b8f7e3cac6379024cb2a
    robotCordList = []
    cordList = []
    currentTruckCord = [0,0,0]
    rosPubMsg = Float64MultiArray()
    pub = rospy.Publisher("VelocityDrive", Float64MultiArray, queue_size = 10)

    def __init__(self):
        rospy.init_node('PurePursuitController', anonymous=True)

    ##updates truck coordinate targets to rover cordites as rover moves
    def UpdateTargetPoints(self):
        newTruckCord = self.getTruckCord()
        theta = newTruckCord[2] - 3.14159/2
        xBRef = newTruckCord[0]
        yBRef = newTruckCord[1]
        length = 0
        for i in range(len(self.robotCordList)):
            xTBody = self.cordList[i][0]*math.cos(theta) + self.cordList[i][1]*math.sin(theta) - yBRef*math.sin(theta) - xBRef*math.cos(theta)
            yTBody = -self.cordList[i][0]*math.sin(theta) + self.cordList[i][1]*math.cos(theta) - yBRef*math.cos(theta) + xBRef*math.sin(theta)
            distToPoint = round(math.sqrt(math.pow(xTBody,2) + math.pow(yTBody,2)), 3)
            self.robotCordList[i] = [xTBody, yTBody, distToPoint]
    ## helper methods for update target points
    def getTruckCord(self):
        return self.currentTruckCord


    def getPath(self):
        pathList = [[0, 2], [5, 7]]
        for point in pathList:
            self.cordList.append([point[0],point[1],0])
            self.robotCordList.append([point[0],point[1],0])
        self.UpdateTargetPoints()
        self.nextPointDistanceDec = self.robotCordList[0][2]



    def removeTargetPoint(self):
        if (abs(self.robotCordList[0][2]) < self.nextPointDistanceDec):
            self.nextPointDistanceDec = self.robotCordList[0][2]
        elif (abs(self.robotCordList[0][2]) > self.lookAheadDist and abs(self.robotCordList[0][2]) > (self.nextPointDistanceDec+0.5) and abs(self.robotCordList[0][2]) < (self.lookAheadDist+ (1/4)*self.lookAheadDist )):
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
        if(abs(x)>self.allowedError):
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
<<<<<<< HEAD
            self.currentMaxDriveSpeed = 0.6
=======
            self.currentMaxDriveSpeed = 0.5
>>>>>>> d9194df257f13d185327b8f7e3cac6379024cb2a
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

    def callbackOdom(self, data):
        self.currentTruckCord = data.data
        print(data.data)
        if (self.cordList):
            self.UpdateTargetPoints()
            output = self.getTargetCordAndDriveSpeed(self.lookAheadDist, self.currentMaxDriveSpeed)
            isFinished = self.removeTargetPoint()
            if(output != "atDest" and not isFinished):
                # print("{}, {}".format(output[0][0],output[0][1]))
                self.rosPubMsg.data = output[0]
                ## ros.drive.setvel(output[0][0], output[0][1])
            else:
                self.rosPubMsg.data = [0,0]
                ## ros.drive.setvel(0,0)
            print(output[0])
        else:
            ## ros.drive.setvel(0,0)
            self.rosPubMsg.data = [0,0]
            print("no target Point")
        
        self.pub.publish(self.rosPubMsg)


controller = PurePursuit()
def run():
    controller.getPath()
    sub = rospy.Subscriber("odometry/encoder", Float64MultiArray, callBack)
    rospy.spin()

def callBack(data):
    controller.callbackOdom(data)

if __name__ == '__main__':
    run()
   
