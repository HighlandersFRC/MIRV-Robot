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



class pointTurnRelative():
    def __init__(self):
        rospy.init_node("pointTurnRelative", anonymous=True)
        self._feedback = msg.PointTurnFeedback()
        self._result = msg.PointTurnResult()

        self._action_name = "PointTurnRelativeAS"
        self._as = actionlib.SimpleActionServer(
            self._action_name, msg.PointTurnAction, auto_start=False)
        self._as.register_goal_callback(self.turnToTarget)
        self._as.register_preempt_callback(self.preemptedPointTurn)
        self._as.start()

        self.powerdrive_pub = rospy.Publisher(
            "PowerDrive", Float64MultiArray, queue_size=10)
        self.imu_sub = rospy.Subscriber('CameraIMU', Float64, self.updateIMU)
        self.velocitydrive_pub = rospy.Publisher(
            "cmd_vel", Twist, queue_size=5)

        self.velocityMsg = Twist()

        self.prevTriggerVal = 1

        self.imu = 0

        self.kP = 0.02
        self.kI = 0.00000
        self.kD = 2
        self.setPoint = 0
        self.PID_SUCCESS_THRESHOLD = 0.2

        self.setAllZeros()

        self.pid = PID(self.kP, self.kI, self.kD, self.setPoint)
        self.pid.setMaxMinOutput(0.5)
        self.pid.setContinuous(360, 0)
        self.pid.setIZone(8)

        self.lastImuTime = time.time()

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

    def getAngleError(self, imu, target):
        # initialImu: 12, target: 6, imu: 10, (10 - 12 + 6) = 4
        # angle error: +4 (turn 4 degreez in positive Z direction to reach target)
        error = (imu - target) % 360
        if error > 180:
                error -= 360
        return error

    def updateIMU(self, data):
        self.imu = data.data
        if (time.time() - self.lastImuTime > .1):
            self.lastImuTime = time.time()
            print(self.imu)
        # print("UPDATED IMU TO: ", self.imu, " at Time: ", time.time())

    def turnToTarget(self):
        print("GOT PICKUP CALLBACK")
        goal = self._as.accept_new_goal()
        print("ACCEPTED GOAL TO PICKUP PI LIT!")
        initialImu = self.imu
        targetAngle = (initialImu + goal.targetAngle)%360
        successThreshold = goal.successThreshold
        print(f"Target angle: {targetAngle}, Success Threshold: {successThreshold}")
        # angleError = self.getAngleError(initialImu, targetAngle, self.imu)
        self.pid.setSetPoint(targetAngle)
        reachedTarget = False

        prevTime = time.time()

        while(reachedTarget == False):
            angularVelPID = self.pid.updatePID(self.imu)  # this returns in radians/sec
            error = self.getAngleError(self.imu, targetAngle)

            self.velocityMsg.linear.x = 0
            self.velocityMsg.angular.z = -angularVelPID

            self.velocitydrive_pub.publish(self.velocityMsg)

            

            # if (time.time() - prevTime > 0.1):
            #     prevTime = time.time()
            #     print(f"Current Set Point: {self.pid.setPoint}, Current Angle: {self.imu}")
            #     print(f"Setting Angular Velocity to {angularVelPID}")
            #     print(f"ERROR: {error}")

            if(abs(error) < successThreshold): # and abs(angularVelPID) < self.PID_SUCCESS_THRESHOLD
                print("SUCCESSFULLY POINT TURNED")
                print(f"ERROR: {self.getAngleError(self.imu, targetAngle)}")
                reachedTarget = True
                startTime = time.time()
                self.velocityMsg.linear.x = 0
                self.velocityMsg.angular.z = 0
                self.velocitydrive_pub.publish(self.velocityMsg)
                print("Set motion to zero")
                while (time.time() - startTime < 0.2):
                    self.velocityMsg.linear.x = 0
                    self.velocityMsg.angular.z = -angularVelPID
                    self.velocitydrive_pub.publish(self.velocityMsg)
                self.velocityMsg.linear.x = 0
                self.velocityMsg.angular.z = 0
                self.velocitydrive_pub.publish(self.velocityMsg)
        print(f"ERROR: {self.getAngleError(self.imu, targetAngle)}")
        self._result.finished = reachedTarget
        self._result.angleError = self.getAngleError(self.imu, targetAngle)
        self._as.set_succeeded(self._result)
        self.setAllZeros()

    def preemptedPointTurn(self):
        if(self._as.is_new_goal_available()):
            print("point turn preempt received")
            self._as.set_preempted()
        else:
            print("aborting point turn")
            self._as.set_aborted()

    def run(self):
        rospy.spin()


turn = pointTurnRelative()
if __name__ == '__main__':
    print("RUNNING")
    turn.run()
