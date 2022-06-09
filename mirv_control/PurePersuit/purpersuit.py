#!/usr/bin/env python
import math
import numpy as np
from numpy import array
from simClass import simulationModel

class PurePursuit():
    wheelBaseWidth = 0.438
    lastTruckCord = [0,0, 3.14159/2]
    nextPointDistanceDec = 0
    lookAheadDist = 5
    cordList = []
    simulation = simulationModel()

    def get
    ##updates truck coordinate targets to rover cordites as rover moves
    def UpdateTargetPoints(self):
        newTruckCord = self.getTruckCord()
        deltaCord = [(newTruckCord[0]-self.lastTruckCord[0]),(newTruckCord[1]-self.lastTruckCord[1]), (self.lastTruckCord[2]-newTruckCord[2]) ]
        for i in self.cordList:
            X1 = round(i[0] - deltaCord[0], 3)
            Y1 = round(i[1] - deltaCord[1], 3)
            i[0] = round((X1*math.cos(deltaCord[2])-Y1*math.sin(deltaCord[2])),4)
            i[1] = round((X1*math.sin(deltaCord[2])+Y1*math.cos(deltaCord[2])),4)
            i[2] = round(math.sqrt(math.pow(i[0],2) + math.pow(i[1],2)), 3)
        self.lastTruckCord = newTruckCord
    ## helper methods for update target points
    def getTruckCord(self):
        return self.simulation.getPos()


    def getPath(self):
        pathList = self.simulation.getPath()
        for point in pathList:
            self.cordList.append([point[0],point[1],0])
        print(self.cordList)
        self.UpdateTargetPoints()
        self.nextPointDistanceDec = self.cordList[0][2]
        print(self.cordList)
        print("")
        print("")



    def removeTargetPoint(self):
        if (abs(self.cordList[0][2]) < self.nextPointDistanceDec):
            self.nextPointDistanceDec = self.cordList[0][2]
        elif (abs(self.cordList[0][2]) > self.lookAheadDist and abs(self.cordList[0][2]) > (self.nextPointDistanceDec+2) and abs(self.cordList[0][2]) < (self.lookAheadDist+ (1/2)*self.lookAheadDist )):
            self.cordList.pop(0)
            if(self.cordList):
                self.nextPointDistanceDec = self.cordList[0][2]
            else:
                return True
        return False

    ## calulates wheel velocity's for that given frame
    def calculateSpeedSide(self, maxSpeed, x, y, la):
        leftVel = maxSpeed
        rightVel = maxSpeed
        if(abs(x)>0.5):
            cRad = self.generateRadius(x, la)
            iCirc = self.generateCircumference((cRad - (self.wheelBaseWidth/2)))
            oCirc = self.generateCircumference((cRad + (self.wheelBaseWidth/2)))
            innerSpeedRatio = (iCirc/oCirc)
            turnRight = True
            if(x>=0):  
                rightVel = rightVel*innerSpeedRatio
                print("turningRight")
            else:
                leftVel = leftVel*innerSpeedRatio
                print("turningLeft")
        return ([leftVel, rightVel])

    ##helper methods to calculateSpeedSides
    def generateRadius(self,x,la):
        return(math.pow(la,2)/(abs(2*x)))
        # return(abs(2*x))/(math.pow(la,2))
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
            targetPoint = farPoint
            snapshotLa = (targetPoint[0]**2 + targetPoint[1]**2)**0.5
            if (snapshotLa < la/10):
                return ("atDest")
        elif closePoint:
            targetPoint = self.vectorIntercept(closePoint, la)
        
        else:
            print("cannot determine target point")
        sideSpeeds = self.calculateSpeedSide(maxSpeed, targetPoint[0], targetPoint[1], snapshotLa)
        return([targetPoint, sideSpeeds])

    ##helper method for getTargetCord
    def vectorIntercept(self, closePoint, la):
        cPO = array(closePoint)
        normalVect = (cPO)/(((cPO**2).sum()**0.5))
        targetPoint = normalVect*la
        print("Vector Intercept")
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
        print("double Vector Intercept")
        return target
    def closestOut(self, la):
        closePoint = []
        for point in self.cordList:
            if(point[2] > la):
                return [point[0], point[1]]
        return closePoint
    def furthestIn(self, la):
        farPoint = []
        for point in self.cordList:
            if(point[2] < la):
                farPoint = point.copy()
            else:
                if(farPoint):
                    return [farPoint[0], farPoint[1]]
        return farPoint
    def testCode(self):
        self.getPath()
        isFinished = False
        for i in range (255):
            if not isFinished:
                self.UpdateTargetPoints()
                output = self.getTargetCordAndDriveSpeed(self.lookAheadDist,2)
                if(output != "atDest"):
                    self.simulation.setDrive(output[1][0],output[1][1],0.25, [output[0][0],output[0][1]])
                    print("targetX: {}, TargetY: {}, LeftVelocity: {}, RightVelocity: {}".format(round(output[0][0], 3),round(output[0][1], 3),round(output[1][0], 3),round(output[1][1], 3)))
                    robotPos = self.simulation.getPos() 
                    print("robot Cordinate: {}".format(robotPos))
                    self.UpdateTargetPoints()
                    isFinished = self.removeTargetPoint()
                    print("itteration: {}".format(i))
                    print(self.cordList)
                    print("")
        self.simulation.LogData()
def main():
    controller = PurePursuit()
    controller.testCode()
   

    # NotAtDest = True
    # while(pathlist and NotAtDest):
    #     UpdateTargetPoints()
    #     targetSpeeds = getTargetCordAndDriveSpeed(lookAheadDist,5)
    #     if(targetSpeeds == "atDest"):
    #         NotAtDest = False
    #     UpdateTargetPoints()
    #     removeTargetPoint()




if __name__ == '__main__':
    main()