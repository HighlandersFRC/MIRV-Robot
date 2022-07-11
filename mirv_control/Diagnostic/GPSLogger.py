#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import NavSatFix
import math
import time
import numpy as np
from numpy import array
import csv
from datetime import datetime


class Logger():
    points = []
    iterations = 0

    def __init__(self):
        rospy.init_node('TruckCoordinateLogger', anonymous=True)
        self.points.append(["Iteration", "lat", "lon"])

    def getPos(self, data):
        if(self.iterations < 1000):
            print(self.iterations)
            self.points.append([self.iterations, data.latitude, data.longitude])
            self.iterations += 1
        else:
            self.logData()
            time.sleep(5)
            rospy.signal_shutdown("finished")

    def logData(self):
            print("logging data")
            # print(self.points)
            with open(("Data{}.csv".format(datetime.now())), 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(self.points)


calculator = Logger()


def run():
    sub = rospy.Subscriber("/gps/fix", NavSatFix, callBack)
    rospy.spin()
    calculator.logData()


def callBack(data):
    calculator.getPos(data)


if __name__ == '__main__':
    run()
