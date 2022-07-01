#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Float64MultiArray
import pymap3d as pm
class GlobalToTruck():
    startingCord = [0, 0, 0]
    newCord = [0, 0, 0]
    startingHeading = 0
    startCordSet = False
    # EarthSMaxis = 6378137
    # eccentricity = 0.08181919
    pub = rospy.Publisher("TruckCoordinates", Float64MultiArray, queue_size = 2)
    rosPubMsg = Float64MultiArray()
    def __init__(self):
        rospy.init_node('TruckCoordinateConversion', anonymous=True)

    def getCordDelta(self, cordDel):
        distance = self.EarthSMaxis/(1-((self.eccentricity**2)*(math.sin(cordDel[0]))**2))
        X = distance*math.cos(cordDel[0])*math.cos(cordDel[1])
        Y = distance*math.cos(cordDel[0])*math.sin(cordDel[1])
        return [X,Y]

    def convertToTruck(self):
        theta = self.startingHeading
        cord1 = self.newCord
        cord2 = self.startingCord
        # cord1Cart = self.getCordDelta(self.startingCord) 
        # cord2Cart = self.getCordDelta(self.newCord)
        # delta = [cord2Cart[0]-cord1Cart[0], cord2Cart[1]-cord1Cart[1]]
        # print(delta)
        delta = pm.geodetic2enu(cord1[0], cord1[1], cord1[2], cord2[0], cord2[1],cord2[2])
        XT = delta[0]*math.cos(theta)-delta[1]*math.sin(theta)
        YT = delta[0]*math.sin(theta)+delta[1]*math.cos(theta)

        print("dist between: {}".format(((XT**2)+(YT**2))**.5))
        return [XT, YT]

    def degToRad(self, angle):
        rad = angle*3.1415926/180
        return (rad)

    def calcPos(self, data):
        temp = data.data
        self.newCord = [temp[0], temp[1], temp[2]]
        
        if(not self.startCordSet):
            self.setStartingPoint([temp[0], temp[1], temp[2]])
        output = self.convertToTruck()

        self.rosPubMsg.data = [output[0],output[1]]
        self.pub.publish(self.rosPubMsg)

        print("output: {}".format(output))

    def setStartingPoint(self, data):
        self.startingHeading = self.degToRad(-90-135)
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
