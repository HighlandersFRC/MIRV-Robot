#!/usr/bin/env python3
from http.client import IM_USED
import math
import time

from numpy import True_
import rospy
from std_msgs.msg import Float64MultiArray, String, Float64
from PID import PID
from geometry_msgs.msg import Twist
import actionlib
import mirv_control.msg as msg
import asyncio



class driveDistance:
    def __init__(self):
        rospy.init_node("driveDistance")
        self._feedback = msg.DriveDistanceResult()
        self._result = msg.DriveDistanceResult()

        self._action_name = "DriveDistanceAS"
        self._as = actionlib.SimpleActionServer(
            self._action_name, msg.DriveDistanceAction, auto_start=False)
        self._as.register_goal_callback(self.driveDistance)
        self._as.register_preempt_callback(self.preemptedDriveDistance)
        self._as.start()

        self.powerdrive_pub = rospy.Publisher(
            "PowerDrive", Float64MultiArray, queue_size=10)
        self.velocitydrive_pub = rospy.Publisher(
            "cmd_vel", Twist, queue_size=5)

        self.velocityMsg = Twist()

        self.imu = 0

        self.kP = 0.0175
        self.kI = 0
        self.kD = 2
        self.setPoint = 0

        self.setAllZeros()

        self.pid = PID(self.kP, self.kI, self.kD, self.setPoint)
        self.pid.setMaxMinOutput(0.5)
        self.pid.setContinuous(360, 0)
        self.pid.setIZone(8)

    def setAllZeros(self):
        self.imu = 0

        self.driveToPiLit = False
        self.prevPiLitAngle = 0

        self.runPID = False

        self.moveToPiLitRunning = False
        self.movementInitTime = 0

        self.finished = False

        self.reachedEstimate = False
        self.allowSearch = False

    def getDistanceError(self, timeStart, velocity, targetDistance):
        # timeDelta: seconds, velocity: meters per second, targeDistance: meters
        timeDelta = time.time() - timeStart
        return targetDistance - timeDelta * velocity

    def driveDistance(self):
        print("GOT DRIVE DISTANCE CALLBACK")
        goal = self._as.accept_new_goal()
        print("ACCEPTED GOAL TO DRIVE DISTANCE!")

        timeStart = time.time()
        targetDistanceMeters = goal.targetDistanceMeters
        velocityMPS = goal.velocityMPS
        successThreshold = goal.successThreshold

        distanceError = self.getDistanceError(
            timeStart, velocityMPS, targetDistanceMeters)
        reachedTarget = False

        prevTime = time.time()

        while(reachedTarget == False):
            distanceError = self.getDistanceError(
                timeStart, velocityMPS, targetDistanceMeters)
            direction = 1 if distanceError > 0 else -1

            self.velocityMsg.linear.x = velocityMPS * direction
            self.velocityMsg.angular.z = 0

            self.velocitydrive_pub.publish(self.velocityMsg)


            if (time.time() - prevTime > 0.1):
                prevTime = time.time()
                print(f"Current Set Point: {targetDistanceMeters}, Current Position: {(time.time()-timeStart)*velocityMPS}")
                print(f"Setting Velocity to {velocityMPS * direction}")
                print(f"ERROR: {distanceError}")

            if abs(distanceError) < successThreshold:
                reachedTarget = True
                self.velocityMsg.linear.x = 0
                self.velocityMsg.angular.z = 0
                self.velocitydrive_pub.publish(self.velocityMsg)
        self.setAllZeros()
        self._result.finished = reachedTarget
        self._result.distanceError = distanceError
        self._as.set_succeeded(self._result)

    def preemptedDriveDistance(self):
        if(self._as.is_new_goal_available()):
            print("point turn preempt received")
            self._as.set_preempted()
        else:
            print("aborting point turn")
            self._as.set_aborted()

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    print("RUNNING")
    drive = driveDistance()
    drive.run()