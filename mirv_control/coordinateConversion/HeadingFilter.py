#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import time
from ublox_msgs.msg import NavPVT

class ComputeHeading():
    relativeHeadingChange = 0
    calibrationLength = 100
    currentHeading = 0
    sampleCountGPS = 0
    sampleCountIMU = 0
    startingAngle = 0
    startSet = False

    def __init__(self):
        rospy.init_node("ComputeStartHeading")
        self.imu_dumb_sub = rospy.Subscriber("/CameraIMU", Float64, self.callBackIMU)
        self.GPS_sub = rospy.Subscriber("ublox/navpvt", NavPVT , self.callBackGPS)
        self.pub = rospy.Publisher("/Heading/Start", Float64, queue_size=5)

    def callBackGPS(self, data):
        self.currentHeading = data.heading/100000
        self.sampleCountGPS+=1
        print("got GPS heading")

    def callBackIMU(self, data):
        print("got imu data")
        if(self.startSet == False):
            self.startingAngle = data.data
            self.startSet = True
        else:
            self.sampleCountIMU+=1
            self.relativeHeadingChange = data.data - self.startingAngle
    def run(self):
        while not rospy.is_shutdown():
            if( self.sampleCountGPS >= self.calibrationLength and self.sampleCountIMU >= self.calibrationLength ):
                startingHeading = self.currentHeading - self.relativeHeadingChange
                self.pub.publish(self.startingHeading)
                print(self.startingHeading)
                time.sleep(5)
                break

if __name__ == '__main__':
    compute = ComputeHeading()
    compute.run()
    print("Heading calibration complete")