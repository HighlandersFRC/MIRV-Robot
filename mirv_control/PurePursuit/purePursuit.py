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
<<<<<<< HEAD
    maxDriveSpeed = 2.5
    currentMaxDriveSpeed = 2
    lookAheadDist = 4
    allowedError = 0.05
=======
    maxDriveSpeed = 4.5
    currentMaxDriveSpeed = 1
    startingTheta = 3.14159/2
    lookAheadDist = 1
    allowedError = 0.5
    allowedErrorDist = 0.1
>>>>>>> 927f74c5bd2796ac2a5bdcc61248ee987243a12b
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
<<<<<<< HEAD
        theta = newTruckCord[2] - 3.14159/2
        xBRef = newTruckCord[0]
        yBRef = newTruckCord[1]
        length = 0
        for i in range(len(self.robotCordList)):
=======
        theta = (3.1415926*2 - newTruckCord[2] + 3.14159/2)%(2*3.1415926) - self.startingTheta
        xBRef = newTruckCord[0]
        yBRef = newTruckCord[1]
        # print([xBRef, yBRef, theta])
        length = 0
        for i in range(len(self.robotCordList)):
            # xTBody = xBRef*math.cos(theta) + yBRef*math.sin(theta) - self.cordList[i][1]*math.sin(theta) - self.cordList[i][0]*math.cos(theta)
            # yTBody = -xBRef*math.sin(theta) + yBRef*math.cos(theta) - self.cordList[i][1]*math.cos(theta) + self.cordList[i][0]*math.sin(theta)
            # distToPoint = round(math.sqrt(math.pow(xTBody,2) + math.pow(yTBody,2)), 3)

>>>>>>> 927f74c5bd2796ac2a5bdcc61248ee987243a12b
            xTBody = self.cordList[i][0]*math.cos(theta) + self.cordList[i][1]*math.sin(theta) - yBRef*math.sin(theta) - xBRef*math.cos(theta)
            yTBody = -self.cordList[i][0]*math.sin(theta) + self.cordList[i][1]*math.cos(theta) - yBRef*math.cos(theta) + xBRef*math.sin(theta)
            distToPoint = round(math.sqrt(math.pow(xTBody,2) + math.pow(yTBody,2)), 3)
            self.robotCordList[i] = [xTBody, yTBody, distToPoint]
<<<<<<< HEAD
=======
        print("target X,Y: {}, CurrentTheta: {}".format(self.robotCordList[0], theta))
        print("targetTruck X,Y: {}".format(self.cordList[0]))
>>>>>>> 927f74c5bd2796ac2a5bdcc61248ee987243a12b
    ## helper methods for update target points
    def getTruckCord(self):
        return self.currentTruckCord


    def getPath(self):
<<<<<<< HEAD
        pathList = [[0, 2], [5, 7]]
=======
        pathList = [[-3.5, 0], [-3,-11], [15, -11]]
>>>>>>> 927f74c5bd2796ac2a5bdcc61248ee987243a12b
        for point in pathList:
            self.cordList.append([point[0],point[1],0])
            self.robotCordList.append([point[0],point[1],0])
        self.UpdateTargetPoints()
        self.nextPointDistanceDec = self.robotCordList[0][2]



    def removeTargetPoint(self):
        if (abs(self.robotCordList[0][2]) < self.nextPointDistanceDec):
            self.nextPointDistanceDec = self.robotCordList[0][2]
<<<<<<< HEAD
        elif (abs(self.robotCordList[0][2]) > self.lookAheadDist and abs(self.robotCordList[0][2]) > (self.nextPointDistanceDec+0.5) and abs(self.robotCordList[0][2]) < (self.lookAheadDist+ (1/4)*self.lookAheadDist )):
=======
        elif (abs(self.robotCordList[0][2]) < (1.5)*self.lookAheadDist and len(self.robotCordList) > 1):
            self.robotCordList.pop(0)
            self.cordList.pop(0)
            if(self.robotCordList):
                self.nextPointDistanceDec = self.robotCordList[0][2]
            else:
                return True
        elif (abs(self.robotCordList[0][2]) < (self.allowedError) and len(self.robotCordList) <= 1):
>>>>>>> 927f74c5bd2796ac2a5bdcc61248ee987243a12b
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
<<<<<<< HEAD
        if(abs(x)>self.allowedError):
=======
        if(abs(x)>self.allowedErrorDist):
>>>>>>> 927f74c5bd2796ac2a5bdcc61248ee987243a12b
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

    def callbackOdom(self, data):
        self.currentTruckCord = data.data
<<<<<<< HEAD
        print(data.data)
=======
        # print(data.data)
>>>>>>> 927f74c5bd2796ac2a5bdcc61248ee987243a12b
        if (self.cordList):
            self.UpdateTargetPoints()
            output = self.getTargetCordAndDriveSpeed(self.lookAheadDist, self.currentMaxDriveSpeed)
            isFinished = self.removeTargetPoint()
            if(output != "atDest" and not isFinished):
                # print("{}, {}".format(output[0][0],output[0][1]))
                self.rosPubMsg.data = output[0]
<<<<<<< HEAD
                ## ros.drive.setvel(output[0][0], output[0][1])
            else:
                self.rosPubMsg.data = [0,0]
                ## ros.drive.setvel(0,0)
            print(output[0])
=======
                # self.rosPubMsg.data = [2.2, 4]
            else:
                self.rosPubMsg.data = [0,0]


            print(self.rosPubMsg.data)
        
        
>>>>>>> 927f74c5bd2796ac2a5bdcc61248ee987243a12b
        else:
            ## ros.drive.setvel(0,0)
            self.rosPubMsg.data = [0,0]
            print("no target Point")
        
        self.pub.publish(self.rosPubMsg)

<<<<<<< HEAD
=======
    def __del__(self):
        print("exiting Program")
        self.rosPubMsg.data = [0,0]
        self.pub.publish(self.rosPubMsg)
        time.sleep(3)
        print("exited")

>>>>>>> 927f74c5bd2796ac2a5bdcc61248ee987243a12b

controller = PurePursuit()
def run():
    controller.getPath()
<<<<<<< HEAD
    sub = rospy.Subscriber("odometry/encoder", Float64MultiArray, callBack)
=======
    sub = rospy.Subscriber("GPS/IMUPOS", Float64MultiArray, callBack)
>>>>>>> 927f74c5bd2796ac2a5bdcc61248ee987243a12b
    rospy.spin()

def callBack(data):
    controller.callbackOdom(data)

if __name__ == '__main__':
    run()
   
