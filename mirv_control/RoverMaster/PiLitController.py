#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String
import time
import threading
reversePin = "DO0"
inhibitPin = "DO1"
resetPin = "DO2"
patternPin = "DO3"
connectionPin = "DI1"

class PiLitControl():
    DI1 = False
    piLitRestartTimeout = 5
    networkFailCount = 0
    reveresState = 0
    inhibitState = 0
    patternState = 0
    def __init__(self):
        self.pub = rospy.Publisher("DOControl", String, queue_size=10)
        self.sub = rospy.Subscriber("/DIO/DI1", Bool, self.callback)
        rospy.init_node("PiLitController", anonymous=True)
        self.rate = rospy.Rate(1)
        WatchdogThread = threading.Thread(target = self.watchDog, name = "thread2")
        print("Starting PiLit watchdog")
        WatchdogThread.start()
        
    def reversePattern(self, isReversed):
        self.pub.publish("{},{}".format(patternPin, int(isReversed)))
        self.reversState = int(isReversed)
    def callback(self, data):
        self.DI1 = data

    def isConnected(self):
        return self.DI1

    def inhibit(self, isInhibit):
        self.pub.publish("{},{}".format(inhibitPin, int(isInhibit)))
        self.inhibitState = int(isInhibit)

    def patternType(self, isWave):
        self.pub.publish("{},{}".format(patternPin, int(isWave)))
        self.patternState = int(isWave)

    def reset(self):
        self.pub.publish("{},1".format())
        time.sleep(0.5)
        self.pub.publish("{},0".format())
        time.sleep(0.5)
        self.pub.publish("{},1".format())

    def watchDog(self):
        while not rospy.is_shutdown():
            if not self.isConnected():
                self.networkFailCount += 1
            if self.networkFailCount >= self.piLitRestartTimeout:
                rospy.logwarn("PiLit failed to connect for {} Cycles, Resetting Module".format(self.piLitRestartTimeout))
                self.reset()
                self.networkFailCount = 0
            self.pub.publish("{},{}".format(patternPin, int(self.patternState)))
            self.pub.publish("{},{}".format(inhibitPin, int(self.inhibitState)))
            self.pub.publish("{},{}".format(patternPin, int(self.reversState)))
            self.rate.sleep()



if __name__ == '__main__':
    piLitCtrl = PiLitControl()
