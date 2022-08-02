#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String
import time
reversePin = "DO0"
inhibitPin = "DO1"
resetPin = "DO2"
patternPin = "DO3"
connectionPin = "DI1"

class PiLitControl():
    DI1 = False

    #Constructor
    def __init__(self):
        self.a_class_constant
        self.pub = rospy.Publisher("DOControl", String, queue_size=10)
        self.sub = rospy.Subscriber("/DIO/DI1", bool, self.callback)
        rospy.init_node("Pi-LitController", anonymous=True)
        
    def reversePattern(self, isReversed):
        self.pub.publish("{},{}".format(patternPin, isReversed))

    def callback(self, data):
        self.DI1 = data

    def isConnected(self):
        return self.DI1


    def inhibit(self, isInhibit):
        self.pub.publish("{},{}".format(inhibitPin, isInhibit))


    def patternType(self, isWave):
        self.pub.publish("{},{}".format(patternPin, isWave))


    def resetcontroller(self):
        self.pub.publish("{},1".format())
        time.sleep(0.5)
        self.pub.publish("{},0".format())
        time.sleep(0.5)
        self.pub.publish("{},1".format())

    def testFunction(self):
        pass



if __name__ == '__main__':
    piLitCtrl = PiLitControl()
    piLitCtrl.inhibit(True)