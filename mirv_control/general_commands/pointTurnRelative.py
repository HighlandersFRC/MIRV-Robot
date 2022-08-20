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

rospy.init_node("pointTurnRelative")


class pointTurnRelative:
    def __init__(self):
        self._feedback = msg.PointTurnFeedback()
        self._result = msg.PointTurnResult()

        self._action_name = "PointTurnRelativeAS"
        self._as = actionlib.SimpleActionServer(
            self._action_name, msg.MovementToPiLitAction, auto_start=False)
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

        self.kP = 0.0175
        self.kI = 0
        self.kD = 2
        self.setPoint = 0

        self.setAllZeros()

        self.pid = PID(self.kP, self.kI, self.kD, self.setPoint)
        self.pid.setMaxMinOutput(0.5)

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

    def getAngleError(initialImu, target, imu):
        # initialImu: 12, target: 6, imu: 10, (10 - 12 + 6) = 4
        # angle error: +4 (turn 4 degreez in positive Z direction to reach target)
        return (imu - initialImu) - target

    def turnToTarget(self):
        print("GOT PICKUP CALLBACK")
        goal = self._as.accept_new_goal()
        print("ACCEPTED GOAL TO PICKUP PI LIT!")
        initialImu = self.imu
        targetAngle = goal.targetAngle
        successThreshold = goal.successThreshold
        angleError = self.getAngleError(initialImu, targetAngle, self.imu)
        self.estimatePID.setSetPoint(targetAngle)
        self.reachedTarget = False

        while(reachedTarget == False):
            angleError = self.getAngleError(initialImu, targetAngle, self.imu)

            angularVelPID = self.estimatePID.updatePID(
                self.angleError)  # this returns in radians/sec

            self.velocityMsg.linear.x = 0
            self.velocityMsg.angular.z = angularVelPID

            self.velocitydrive_pub.publish(self.velocityMsg)

            if(abs(angleError) < successThreshold):
                reachedTarget = True
                self.velocityMsg.linear.x = 0
                self.velocityMsg.angular.z = 0
                self.velocitydrive_pub.publish(self.velocityMsg)
        self.setAllZeros()
        self._result.finished = reachedTarget
        self._result.angleError = angleError
        self._as.set_succeeded(self._result)

    def preemptedPointTurn(self):
        if(self._as.is_new_goal_available()):
            print("point turn preempt received")
            self._as.set_preempted()
        else:
            print("aborting point turn")
            self.cancelCallback()

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.velocityMsg.linear.x = 0
            self.velocityMsg.angular.z = 0

            print(self.velocityMsg)
            self.velocitydrive_pub.publish(self.velocityMsg)
        except:
            print("an error occurred in purePursuit.py")


if __name__ == '__main__':
    print("RUNNING")
    turn = pointTurn()
    turn.run()
