#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import time
from ublox_msgs.msg import NavPVT
import mirv_control.msg as ASmsg
from geometry_msgs.msg import Twist
import actionlib

class ComputeHeading():
    relativeHeadingChange = 0
    calibrationLength = 100
    failCount = 150
    currentHeading = 0
    sampleCountGPS = 0
    sampleCountIMU = 0
    startingAngle = 0
    succeeded = False
    startSet = False
    result = ASmsg.IMUCalibrationResult()
    rosPubMsg = Twist()
    rosPubMsgHeading = Float64()
    headingPub = rospy.Publisher("Start/Heading", Float64, queue_size=2)
    drivePub = rospy.Publisher("cmd_vel", Twist, queue_size=2)
    def __init__(self):
        rospy.init_node("ComputeStartHeading")
        self.imu_dumb_sub = rospy.Subscriber("/CameraIMU", Float64, self.callBackIMU)
        self.GPS_sub = rospy.Subscriber("ublox/navpvt", NavPVT , self.callBackGPS)
        self._action_name = "StartingHeading"
        self._as = actionlib.SimpleActionServer(self._action_name, ASmsg.mirv_control.msg.IMUCalibrationAction, auto_start = False)
        self._as.register_goal_callback(self.execute_cb)
        self._as.start()


    def execute_cb(self):
        goal = self._as.accept_new_goal()
        self.sampleCountGPS = 0
        self.sampleCountIMU = 0
        self.rosPubMsg.linear.x = 0.2
        self.rosPubMsg.angular.z = 0
        self.drivePub.publish(self.rosPubMsg)
        startHead = self.run()
        self.rosPubMsg.linear.x = 0 
        self.rosPubMsg.angular.z = 0
        self.drivePub.publish(self.rosPubMsg)
        self.result.succeeded = self.succeeded
      
        self.rosPubMsgHeading.data = startHead
        self.headingPub.publish(self.rosPubMsgHeading)
        time.sleep(5)
        self._as.set_succeeded(self.result)


    def callBackGPS(self, data):
        self.currentHeading = data.heading/100000
        self.sampleCountGPS+=1
        # print("got GPS heading")

    def callBackIMU(self, data):
        # print("got imu data")
        if(self.startSet == False):
            self.startingAngle = data.data
            self.startSet = True
        else:
            self.sampleCountIMU+=1
            self.relativeHeadingChange = data.data - self.startingAngle
    def run(self):
        while not rospy.is_shutdown():
            print("gps {}, IMU, {}".format(self.sampleCountGPS, self.sampleCountIMU))
            if( self.sampleCountGPS >= self.calibrationLength and self.sampleCountIMU >= self.calibrationLength ):
                startingHeading = self.currentHeading - self.relativeHeadingChange
                print(startingHeading)
                self.succeeded = True
                return startingHeading
            if (abs(self.sampleCountGPS - self.sampleCountIMU) > self.failCount):
                self.succeeded = False
                if( self.sampleCountIMU == 0):
                    return self.currentHeading
                else:
                    return self.startingAngle
            time.sleep(0.5)
if __name__ == '__main__':
    compute = ComputeHeading()
    rospy.spin()
    print("Heading calibration complete")