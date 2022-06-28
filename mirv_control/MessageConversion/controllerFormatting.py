#!/usr/bin/env python3
import math
import time
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64


class DataMerge():
    point = []
    startingTheta = 0.0
    setStartingTheta = False
    theta = 0.0
    pub = rospy.Publisher("GPS/IMUPOS", Float64MultiArray, queue_size = 5)
    rosPubMsg = Float64MultiArray()
    def __init__(self):
        rospy.init_node('OdometyDataUnfiltered', anonymous=True)
    def setCoordinate(self, data):
        temp = data.data
        self.point = [temp[0], temp[1]]
        self.rosPubMsg.data = [self.point[0],self.point[1],(math.radians(self.theta)-self.startingTheta),self.startingTheta, self.theta]
        print(self.rosPubMsg.data)
        self.pub.publish(self.rosPubMsg)
    def setHeading(self, data):
        self.theta = data.data
        if (not self.setStartingTheta):
            self.startingTheta = math.radians(self.theta)
            self.setStartingTheta = True


fuse = DataMerge()
def run():
    subCord = rospy.Subscriber("TruckCoordinates", Float64MultiArray, callBackPoint)
    subHeading = rospy.Subscriber("CameraIMU", Float64, callBackHeading)
    print("looping")
    rospy.spin()
def callBackPoint(data):
    fuse.setCoordinate(data)
def callBackHeading(data):
    fuse.setHeading(data)
if __name__ == '__main__':
    run()