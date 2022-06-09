import math
import numpy as np
from numpy import array
import csv
from datetime import datetime
wheelBaseWidth = 0.438
class simulationModel():
    pointsPath = [[0,10], [50, 60]]
    # pointsPath = [[-15,-40]]
    truckCord = []
    fallowedPath = []
    def __init__(self):
        self.truckCord = [0,0,3.14159/2]

    def getPath(self):
        return self.pointsPath

    def getPos(self):
        return self.truckCord

    def setDrive(self,leftVel,rightVel, timeDer, targetCord):
        if(leftVel == rightVel):
            xP = self.truckCord[0]+(leftVel*timeDer*np.cos(self.truckCord[2]))
            yP = self.truckCord[1]+(leftVel*timeDer*np.sin(self.truckCord[2]))
            newTheta = self.truckCord[2]
        else:
            omega = (rightVel - leftVel)/wheelBaseWidth
            radius = (wheelBaseWidth/2)*((rightVel+leftVel)/(rightVel-leftVel))
            iCCX = (self.truckCord[0] - radius*np.sin(self.truckCord[2]))
            iCCY = (self.truckCord[1] + radius*np.cos(self.truckCord[2]))
            thetaP = omega*timeDer
            xP = (np.cos(thetaP)*(self.truckCord[0]-iCCX))-(np.sin(thetaP)*(self.truckCord[1]-iCCY))+iCCX
            yP = (np.sin(thetaP)*(self.truckCord[0]-iCCX))+(np.cos(thetaP)*(self.truckCord[1]-iCCY))+iCCY
            newTheta = self.truckCord[2]+thetaP
        self.truckCord = [xP, yP, newTheta]
        self.fallowedPath.append([self.truckCord[0], self.truckCord[1], self.truckCord[2], targetCord[0], targetCord[1]])

    def LogData(self):
        with open(("controllerData{}.csv".format(datetime.now())), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(self.fallowedPath)





def injectTest(left, right,time, testSim):
    trackedtime = 0
    while(trackedtime < time):
        testSim.setDrive(left,right,0.025,[0,1])
        trackedtime = trackedtime + 0.025


if __name__ == '__main__':
    testSim = simulationModel()
    print(testSim.getPath())
    injectTest(1,1,1, testSim)
    injectTest(1,3,1, testSim)
    injectTest(1,1,1, testSim)
    # testSim.setDrive(1,1,1,[0,1])
    # testSim.setDrive(.1,2,1,[0,1])
    # testSim.setDrive(1,1,1,[0,1])
    # testSim.setDrive(1,1.1,0.25,[0,1])
    # testSim.setDrive(1,1.1,0.25,[0,1])
    # testSim.setDrive(1.1,1,.25,[0,1])
    # testSim.setDrive(1.1,1,.25,[0,1])
    # testSim.setDrive(1.1,1,.25,[0,1])
    # testSim.setDrive(1.1,1,.25,[0,1])
    # testSim.setDrive(2,3,.25,[0,1])
    # testSim.setDrive(2,3,.25,[0,1])
    # testSim.setDrive(2,3,.25,[0,1])
    # testSim.setDrive(2,3,.25,[0,1])
    # testSim.setDrive(1.5,1.1,.25,[0,1])
    # testSim.setDrive(1.5,1.1,.25,[0,1])
    # testSim.setDrive(1.5,1.1,.25,[0,1])
    # testSim.setDrive(1.5,1.1,.25,[0,1])
    testSim.LogData()
