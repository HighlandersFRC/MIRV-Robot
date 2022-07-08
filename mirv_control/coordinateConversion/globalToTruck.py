#!/usr/bin/env python3
import math

import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import pymap3d as pm


class GlobalToTruck():
    startingCord = [0, 0, 0]
    newCord = [0, 0, 0]
    startingHeading = 0
    startCordSet = False
    # EarthSMaxis = 6378137
    # eccentricity = 0.08181919
    gps_m_pub = rospy.Publisher("gps/odom", Odometry, queue_size=2)

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

        # Use library to convert to NorthEastDown, we want it to be in NorthWestUp, so we will negate [1] and [2] when we use them
        # Global is normal GPS coords
        # Truck is the frame of mirv when it has been dropped off by the truck.
        # 40.479462, -104.973091 is 0, 0
        # 40.479429, -104.971272 should be 150m east
        # pm.geodetic2ned(40.479429, -104.971272, 0, 40.479462, -104.973091, 0) = (-3.6628579618074184, 154.2396502043585, 0.001863376252056348)
        # pm.geodetic2ned(current point, zero point)
        delta = pm.geodetic2ned(cord1[0], cord1[1], cord1[2], cord2[0], cord2[1], cord2[2])
        XT = delta[0] * math.cos(theta) - (-delta[1]) * math.sin(theta)
        YT = delta[0] * math.sin(theta) + (-delta[1]) * math.cos(theta)

        print("dist between: {}".format(((XT**2)+(YT**2))**.5))
        return [XT, YT]

    def degToRad(self, angle):
        rad = angle * np.pi/180
        return rad

    def calcPos(self, data):
        self.newCord = [data.latitude, data.longitude, data.altitude]
        
        if not self.startCordSet:
            self.setStartingPoint([data.latitude, data.longitude, data.altitude])
        output = self.convertToTruck()

        gps_m_odom = Odometry()
        gps_m_odom.header.stamp = data.header.stamp
        gps_m_odom.header.frame_id = 'odom'
        gps_m_odom.child_frame_id = 'odom'
        gps_m_odom.pose.pose.position.x = output[0]
        gps_m_odom.pose.pose.position.y = output[1]
        gps_m_odom.pose.pose.position.z = 0
        gps_m_odom.pose.pose.orientation.w = 1
        gps_m_odom.pose.covariance[0:2] = data.position_covariance[0:2]
        gps_m_odom.pose.covariance[6:8] = data.position_covariance[2:4]

        self.gps_m_pub.publish(gps_m_odom)

        print("output: {}".format(output))

    def setStartingPoint(self, data):
        self.startingHeading = self.degToRad(90-220)
        self.startingCord = data
        self.startCordSet = True


calculator = GlobalToTruck()
def run():
    sub = rospy.Subscriber("gps/fix", NavSatFix, callBack)
    rospy.spin()

def callBack(data):
    calculator.calcPos(data)

if __name__ == '__main__':
    run()
