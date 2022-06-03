#!/usr/bin/env python
import math
import numpy as np
from numpy import array
wheelBaseWidth = 17.25
lastTruckCord = [0,0]
cordList = []

##updates truck coordinate targets to rover cordites as rover moves
def UpdateTargetPoints():
    newTruckCord = getTruckCord()
    deltaCord = [(lastTruckCord[0]-newTruckCord[0]),(lastTruckCord[1]-newTruckCord[1])]
    for i in cordList:
        i[0] = i[0]+ deltaCord[0]
        i[1] = i[1]+ deltaCord[1]
        i[2] = math.sqrt(math.pow(i[0],2) + math.pow(i[1],2))
    lastTruckCord = newTruckCord
## helper methods for update target points
def getTruckCord():
    return ros.curentpos


def getPath():
    path

## calulates wheel velocity's for that given frame
def calculateSpeedSide(maxSpeed, x, y, la):
    leftVel = maxSpeed
    rightVel = maxSpeed
    if(math.abs(x)>0.5):
        cRad = generateRadius(x, la)
        iCirc = generateCircumference((cRad - (wheelBaseWidth/2)))
        oCirc = generateCircumference((cRad + (wheelBaseWidth/2)))
        innerSpeedRatio = (iCirc/oCirc)
        turnRight = True
        if(x>=0):  
            rightVel = rightVel*innerSpeedRatio
        else:
            leftVel = leftVel*innerSpeedRatio
    return ([rightVel, leftVel])

##helper methods to calculateSpeedSides
def generateRadius(x,la):
    return(math.abs(2*x))/(math.pow(la,2))
def generateCircumference(rad):
    return(2*3.141592*rad)

##determines the target cord for each snapshot
def getTargetCord(la):
    closePoint = closestPoint(la)
    farPoint = furthestPoint(la)
    targetPoint = []
    if not closePoint and not farPoint:
        targetPoint = doubleVectorIntercept(closePoint, farPoint,la)
    elif not farPoint:
        targetPoint = farPoint
        la2 = (targetPoint[0]**2 + targetPoint[1]**2)**0.5
    elif not closePoint:
        targetPoint = vectorIntercept(closePoint, la)
    
    else:
        print("cannot determine target point")


def vectorIntercept(closePoint, la):
    cPO = array(closePoint)
    normalVect = (cPO)/(((cPO**2).sum()**0.5))
    targetPoint = normalVect*la
    return targetPoint

def doubleVectorIntercept(closePoint, farPoint, la):
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
    return target
##helper method for getTargetCord
def closestOut(la):
    closePoint = []
    for point in cordList:
        if(point[2] > la):
            return [point[0], point[1]]
    return closePoint
def furthestIn(la):
    farPoint = []
    for point in cordList:
        if(point[2] < la):
            farPoint = point
        else:
            return [farPoint[0], farPoint[1]]

            

def main():
    cp = [20,20]
    fp = [10,10]
    print(doubleVectorIntercept(cp, fp, 15))




if __name__ == '__main__':
    main()