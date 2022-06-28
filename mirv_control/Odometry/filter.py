#!/usr/bin/env python
from signal import sigpending
import time
import rospy

encoder/velocity

class KalmanFilter:
    heading = 0
    xPos = 0
    yPos = 0
    timeSinceUpdate = 0
    WBWidth = 0.483
    loopingRate = rospy.Rate(25)

    def __init__(self):
        encoderSub = rospy.Subscriber("encoder/velocity", Float64MultiArray, self.encoderCallBack)
        IMUSub = rospy.Subscriber("encoder/velocity", Float64, self.IMUcallback)
    def run(self):
        print("one")
        rospy.spin()
        print("two")



    def Forward_Kinimatics(self, data):
        pass
    def IMUcallback(self, data):
        pass
    def encoderCallback(self):
        pass

test = KalmanFilter()
if __name__ == "__main__":
    rospy.init_node('Filter', anonymous=True)
    test.run()