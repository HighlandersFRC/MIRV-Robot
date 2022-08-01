#!/usr/bin/env python3
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

rospy.init_node("piLitPickup")

class piLitPickup:
    def __init__(self):
        self._feedback = msg.MovementToPiLitFeedback()
        self._result = msg.MovementToPiLitResult()

        self._action_name = "PickupAS"
        self._as = actionlib.SimpleActionServer(self._action_name, msg.MovementToPiLitAction, auto_start = False)
        self._as.register_goal_callback(self.turnToPiLit)
        self._as.register_preempt_callback(self.preemptedPickup)
        self._as.start()

        self.powerdrive_pub = rospy.Publisher("PowerDrive", Float64MultiArray, queue_size = 10)
        self.intake_command_pub = rospy.Publisher("intake/command", String, queue_size = 10)
        self.pilit_location_sub = rospy.Subscriber("piLitLocation", Float64MultiArray, self.updatePiLitLocation)
        self.imu_sub = rospy.Subscriber('CameraIMU', Float64, self.updateIMU)
        self.velocitydrive_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 5)
        self.intake_limit_switch_sub = rospy.Subscriber("intake/limitswitches", Float64MultiArray, self.limit_switch_callback)

        self.velocityMsg = Twist()

        self.piLitDepth = 0
        self.piLitAngle = 0

        self.updatedLocation = False
        self.running = False

        self.prevTriggerVal = 1

        self.imu = 0

        self.kP = 0.017
        self.kI = 0
        self.kD = 2
        self.setPoint = 0

        self.estimatekP = 0.01
        self.estimatekI = 0
        self.estimatekD = 0.03
        self.estimateSetPoint = 0

        self.driveToPiLit = False

        self.runPID = False

        self.moveToPiLitRunning = False
        self.movementInitTime = 0
        
        self.finished = False

        self.reachedEstimate = False

        self.setAllZeros()

        self.piLitPID = PID(self.kP, self.kI, self.kD, self.setPoint)
        self.estimatePID = PID(self.estimatekP, self.estimatekI, self.estimatekD, self.estimateSetPoint)
        self.piLitPID.setMaxMinOutput(0.5)
        self.estimatePID.setMaxMinOutput(0.3)
        self.allowSearch = False

        # in order: Left button, Right Button, Bottom Switch, Top Switch
        self.limit_switches = [0, 0, 1, 1]

    def limit_switch_callback(self, switches):
        self.limit_switches = switches.data
        print(self.limit_switches)

    def setAllZeros(self):
        self.piLitDepth = 0
        self.piLitAngle = 0

        self.updatedLocation = False
        self.running = False

        self.prevTriggerVal = 1

        self.imu = 0


        self.driveToPiLit = False
        self.prevPiLitAngle = 0

        self.runPID = False

        self.moveToPiLitRunning = False
        self.movementInitTime = 0
        
        self.finished = False

        self.reachedEstimate = False
        self.allowSearch = False

    def set_intake_state(self, state: str):
        self.intake_command_pub.publish(state)

    def updatePiLitLocation(self, location):
        piLitLocation = location.data
        # if(self.runPID == False and self.driveToPiLit == False):
        if(self.allowSearch == True):
            self.piLitDepth = piLitLocation[0]
            self.piLitAngle = piLitLocation[1]
            self.setPoint = self.imu + self.piLitAngle
            self.setPoint = self.setPoint + 360
            self.setPoint = self.setPoint%360
            self.piLitPID.setSetPoint(self.setPoint)
            self.updatedLocation = True
            self.prevPiLitAngle = self.piLitAngle
            print("SETPOINT: ", self.setPoint)

    def updateIMU(self, data):
        self.imu = data.data
        # print("UPDATED IMU TO: ", self.imu, " at Time: ", time.time())

    def cancelCallback(self):
        self.velocityMsg.linear.x = 0
        self.velocityMsg.angular.z = 0
        self.velocitydrive_pub.publish(self.velocityMsg)
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_aborted()

    def preemptedPickup(self):
        if(self._as.is_new_goal_available()):
            print("pickup preempt received")
            self._as.set_preempted()
        else:
            print("aborting pickup")
            self.cancelCallback()
         
    def moveToPiLit(self):
        # print("SETPOINT: ", self.setPoint, " CURRENT: ", self.imu)
        result = self.piLitPID.updatePID(self.imu) # this returns in radians/sec
        result = -result
        self.velocityMsg.linear.x = 0.25 # m/s
        self.velocityMsg.angular.z = result
        self.velocitydrive_pub.publish(self.velocityMsg)
        if(time.time() - self.movementInitTime > 2/0.25 or self.limit_switches[2] == 1 or self.limit_switches[3] == 1):
            # print("WANTED ANGLE: ", self.setPoint)
            # print("CURRENT ANGLE: ", self.imu)
            self.driveToPiLit = False
            self.moveToPiLitRunning = False
            self.velocityMsg.linear.x = 0
            self.velocityMsg.angular.z = 0
            self.velocitydrive_pub.publish(self.velocityMsg)
            self._result.finished = True
            # self._as.set_succeeded(self._result)
            self.movementInitTime = 0
            self.finished = True
            self.allowSearch = False

    def turnToPiLit(self):
        self.allowSearch = True
        print("GOT PICKUP CALLBACK")
        goal = self._as.accept_new_goal()
        print("ACCEPTED GOAL TO PICKUP PI LIT!")
        # running = goal.runPID
        intakeSide = goal.intakeSide
        estimatedPiLitAngle = goal.estimatedPiLitAngle
        estimatedPiLitAngle = estimatedPiLitAngle + 360
        estimatedPiLitAngle = estimatedPiLitAngle%360
        estimatedPiLitAngle = estimatedPiLitAngle + self.imu
        estimatedPiLitAngle = estimatedPiLitAngle%360
        self.estimatePID.setSetPoint(estimatedPiLitAngle)
        self._result.finished = False
     
        # while(self.reachedEstimate == False and self.piLitAngle == 0):
        #     print("TRYING TO REACH ESTIMATE")
        #     result = self.estimatePID.updatePID(self.imu) # this returns in radians/sec
        #     print("SETPOINT: ", estimatedPiLitAngle, " CURRENT ANGLE: ", self.imu)

        #     self.velocityMsg.linear.x = 0
        #     self.velocityMsg.angular.z = result

        #     self.velocitydrive_pub.publish(self.velocityMsg)

        #     if(abs(self.imu - estimatedPiLitAngle) < 5):
        #         print("GOT TO ESTIMATED TARGET!!!!")
        #         self.reachedEstimate = True
        #         self.velocityMsg.linear.x = 0
        #         self.velocityMsg.angular.z = 0
        #         self.velocitydrive_pub.publish(self.velocityMsg)

        intakeInitTime = time.time()
        self.allowSearch = False
        while(time.time() - intakeInitTime < 3):
            # print(time.time() - intakeInitTime)
            self.velocityMsg.linear.x = 0
            self.velocityMsg.angular.z = 0
            self.velocitydrive_pub.publish(self.velocityMsg)
            self.set_intake_state("intake")
            self.set_intake_state(intakeSide)

        self.allowSearch = True

        searchStartTime = time.time()
        
        while(abs(searchStartTime - time.time()) < 12):
            self.velocityMsg.linear.x = 0
            self.velocityMsg.angular.z = 0
            self.velocitydrive_pub.publish(self.velocityMsg)
            self.set_intake_state("intake")
            self.set_intake_state(intakeSide)
        while(time.time() - searchStartTime < 15 and self.piLitAngle == 0):
            # print("HAVEN'T FOUND A PI LIT YET")
            self.velocityMsg.linear.x = 0
            self.velocityMsg.angular.z = 0
            self.velocitydrive_pub.publish(self.velocityMsg)
            self.runPID = False
            self.moveToPiLit = False
        if(self.piLitAngle != 0):
            self.runPID = True
            self.allowSearch = False
        else:
            print("TIMED OUT")
            self.set_intake_state("reset")
            self.allowSearch = False
            self._as.set_succeeded(self._result)
            self.finished = True

        while(self._result.finished == False):
            # print("RUNNING CALLBACK")
            # print("RUN PID:, ", self.runPID)
            if(self.runPID):
                result = self.piLitPID.updatePID(self.imu) # this returns in radians/sec
                result = -result
                # print("SETPOINT: ", self.setPoint, " CURRENT: ", self.imu)

                self.velocityMsg.linear.x = 0
                self.velocityMsg.angular.z = result

                self.velocitydrive_pub.publish(self.velocityMsg)

                self._feedback.result = result
                self._as.publish_feedback(self._feedback)

                if(abs(self.imu - self.setPoint) < 6.5 and abs(result) < 0.05):
                    print("GOT TO TARGET!!!!")
                    self.driveToPiLit = True
                    self.runPID = False
                    self.velocityMsg.linear.x = 0
                    self.velocityMsg.angular.z = 0
                    self.velocitydrive_pub.publish(self.velocityMsg)
            if(self.driveToPiLit):
                if(self.moveToPiLitRunning == False):
                    self.movementInitTime = time.time()
                    self.moveToPiLitRunning = True
                self.moveToPiLit()
            else:
                storeInitTime = time.time()
                while(time.time() < 3):
                    self.set_intake_state("store")
                    self.velocityMsg.linear.x = 0
                    self.velocityMsg.angular.z = 0
                    self.velocitydrive_pub.publish(self.velocityMsg)
                    self._result.finished = True
        self.setAllZeros()
        self.set_intake_state("store")
        self._as.set_succeeded(self._result)

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.velocityMsg.linear.x = 0
            self.velocityMsg.angular.z = 0
            self.velocitydrive_pub.publish(self.velocityMsg)
        except:
            print("an error occurred in purePursuit.py")

if __name__ == '__main__':
    print("RUNNING")
    pickup = piLitPickup()
    pickup.run()