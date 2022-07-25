#!/usr/bin/env python3
import math

import numpy as np
import rospy
import actionlib
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import pymap3d as pm
import mirv_control.msg as ASmsg


class GlobalToTruck():
    startingCord = [0, 0, 0]
    newCord = [0, 0, 0]
    startingHeading = 0
    result = ASmsg.NavSatToTruckResult()
    startCordSet = False
    # EarthSMaxis = 6378137
    # eccentricity = 0.08181919
    gps_m_pub = rospy.Publisher("gps/odom", Odometry, queue_size=2)
    acceptStartingPoint = False
    def __init__(self):
        rospy.init_node('TruckCoordinateConversion', anonymous=True)
        sub = rospy.Subscriber("gps/fix", NavSatFix, self.callBack)
        self._action_name = "NavSatToTruckAS"
        self._as = actionlib.SimpleActionServer(self._action_name, ASmsg.mirv_control.msg.NavSatToTruckAction, auto_start = False)
        self._as.register_goal_callback(self.execute_cb)
        self._as.start()
    def convertToTruck(self, newCord):
        theta = self.startingHeading
        cord1 = newCord
        print(cord1)
        cord2 = self.startingCord
        delta = pm.geodetic2ned(cord1[0], cord1[1], cord1[2], cord2[0], cord2[1], cord2[2])
        XT = delta[0] * math.cos(theta) - (-delta[1]) * math.sin(theta)
        YT = delta[0] * math.sin(theta) + (-delta[1]) * math.cos(theta)

        print("dist between: {}".format(((XT**2)+(YT**2))**.5))
        return [XT, YT]

    def degToRad(self, angle):
        rad = angle * np.pi/180
        return rad

    def execute_cb(self):
        goal = self._as.accept_new_goal()
        if(self.acceptStartingPoint == True):
            point = self.convertToTruck([goal.latitude, goal.longitude, goal.altitude])
            self.result.truckCoordX = point[0]
            self.result.truckCoordY = point[1]
            self.result.distanceToOrigin = ((point[0]**2)+(point[1]**2))**.5
            self._as.set_succeeded(self.result)
        else:
            self._as.set_aborted()
            raise Exception("no starting point has been sent, Need Data to e sent on /gps/fix ros topic")
    def calcPos(self, data):
        newCord = [data.latitude, data.longitude, data.altitude]
        if(self.acceptStartingPoint == True):
            if (not self.startCordSet):
                self.setStartingPoint([data.latitude, data.longitude, data.altitude])
            output = self.convertToTruck(newCord)
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
            return gps_m_odom
        else:
            raise Exception("no starting point has been sent, Need Data to e sent on /gps/fix ros topic")

    def setStartingPoint(self, data):
        self.startingHeading = self.degToRad(230)
        self.startingCord = data
        self.startCordSet = True

    def callBack(self, data):
        self.acceptStartingPoint = True
        msg = self.calcPos(data)
        self.gps_m_pub.publish(msg)
        rospy.loginfo(msg)
    def run(self):
        sub = rospy.Subscriber("gps/fix", NavSatFix, self.callBack)
        rospy.spin()

calculator = GlobalToTruck()
if __name__ == '__main__':
    calculator.run()
