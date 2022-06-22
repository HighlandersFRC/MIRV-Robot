#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Float64MultiArray
class GlobalToTruck():
    startingCord = [0, 0]
    newCord = [0, 0]
    prevCord = [0, 0]
    startingHeading = 0
    startCordSet = False
    EarthSMaxis = 6378137
    eccentricity = 0.08181919
    def __init__(self):
        rospy.init_node('TruckCoordinateConversion', anonymous=True)

    def getCordDelta(self, cordDel):
        distance = self.EarthSMaxis/(1-((self.eccentricity**2)*(math.sin(cordDel[0]))**2))
        X = distance*math.cos(cordDel[0])*math.cos(cordDel[1])
        Y = distance*math.cos(cordDel[0])*math.sin(cordDel[1])
        return [X,Y]

    def convertToTruck(self):
        theta = self.startingHeading
        cord = self.newCord
        cord1Cart = self.getCordDelta(self.startingCord) 
        cord2Cart = self.getCordDelta(self.newCord)
        delta = [cord2Cart[0]-cord1Cart[0], cord2Cart[1]-cord1Cart[1]]
        # print(delta)
        XT = delta[0]*math.cos(theta)-delta[1]*math.sin(theta)
        YT = delta[0]*math.sin(theta)+delta[1]*math.cos(theta)

        print(delta)
        print("dist between: {}".format(((XT**2)+(YT**2))**.5))
        return [XT, YT]

    def degToRad(self, angle):
        rad = angle*3.1415926/180
        return (rad)

    def calcPos(self, data):
        temp = data.data
        self.newCord = ([self.degToRad(temp[0]), self.degToRad(temp[1])])
        
        if(not self.startCordSet):
            self.setStartingPoint([self.degToRad(temp[0]), self.degToRad(temp[1])])
        output = self.convertToTruck()
        # print("deltaLat: {}, deltaLong: {}, lat: {}, Long: {}".format(round(self.newCord[0]-self.startingCord[0],6), round(self.newCord[1]-self.startingCord[1], 6), self.newCord[0], self.newCord[1]))
        self.prevCord = self.newCord 
        print("output: {}".format(output))

    def setStartingPoint(self, data):
        self.startingHeading = self.degToRad(-90-125)
        self.startingCord = data
        self.startCordSet = True


calculator = GlobalToTruck()
def run():
    sub = rospy.Subscriber("GPSCoordinates", Float64MultiArray, callBack)
    rospy.spin()

def callBack(data):
    calculator.calcPos(data)

if __name__ == '__main__':
    run()
