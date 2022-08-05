#!/usr/bin/env python3
import rospy
# Brings in the SimpleActionClient
import actionlib
import time
from std_msgs.msg import Float64, Float64MultiArray, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import threading


class RoverStates():
    def __init__(self, Interface):
        self.interface = Interface
        self.calibrationClient, self.PPclient, self.pickupClient, self.cloudControllerClient = self.interface.getDriveClients()
        self.roverMonitor = threading.Thread(target=actionServerMonitor, args=(self))
        self.roverMonitor.start()
        self.calStat = self.calibrationClient.get_state()
        self.calTime = 0
        self.PPStat = self.PPclient.get_state()
        self.PPTime = 0
        self.CCStat = self.CloudControlClient.get_state()
        self.CCTime = 0
        self.PUStat = self.pickupClient.get_state()
        self.PUTime = 0

    def actionServerMonitor(self):
        lastTime = rospy.get_time()
        loopTime = 0
        while not rospy.is_shutdown():
            if (self.calStat != self.calibrationClient.get_state()):
                self.calStat = self.calibrationClient.get_state()
                self.calTime = 0
            if (self.PPStat != self.PPclient.get_state()):
                self.PPStat = self.PPclient.get_state()
                self.PPTime = 0
            if (self.CCStat != self.CloudControlClient.get_state())
                self.CCStat = self.CloudControlClient.get_state()
                self.CCTime = 0
            if (self.PUStat != self.pickupClient.get_state())
                self.PUStat = self.pickupClient.get_state()
                self.PUTime = 0
            loopTime = rospy.get_time()-lastTime
            lastTime = rospy.get_time()
            self.calTime += loopTime
            self.PPTime += loopTime
            self.CCTime += loopTime
            self.PUTime += loopTime
            time.sleep(1)
    def getPurePursuitStatus(self):
        return self.PPStat, self.PPTime
    def getCloudStatus(self):
        return self.CCStat, self.CCTime     
    def getPickupStatus(self):
        return self.PUStat, self.PUTime
    def getCalibrationStatus(self):
        return self.calStat, self.calTime
    