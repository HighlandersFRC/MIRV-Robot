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

rospy.init_node("garageDocking")

class moveToGarage:
    def __init__(self):
        self._feedback = msg.GarageFeedback()
        self._result = msg.GarageResult()
        self.allowSearch = True

        self._action_name = "Docking"

        self.velocityMsg = Twist()

        self.GarageDepth = 0
        self.GarageAngle = 0

        self.updatedLocation = False
        self.running = False

        self.prevTriggerVal = 1

        self.imu = 0

        self.kP = 0.02
        self.kI = 0
        self.kD = 2
        self.setPoint = 0

        self.estimatekP = 0.02
        self.estimatekI = 0
        self.estimatekD = 2
        self.estimateSetPoint = 0

        self.driveToGarage = False

        self.runPID = False

        self.moveToGarageRunning = False
        self.movementInitTime = 0
        
        self.finished = False

        self.reachedEstimate = False

        self.setAllZeros()

        self.GaragePID = PID(self.kP, self.kI, self.kD, self.setPoint)
        self.estimatePID = PID(self.estimatekP, self.estimatekI, self.estimatekD, self.estimateSetPoint)
        self.GaragePID.setMaxMinOutput(0.5)
        self.estimatePID.setMaxMinOutput(0.3)
        

        self.touchSensorVals = [0, 0]
        
        self._as = actionlib.SimpleActionServer(self._action_name, msg.GarageAction, auto_start = False)
        self._as.register_goal_callback(self.turnToGarage)
        self._as.register_preempt_callback(self.preemptedPickup)
        self._as.start()

        self.powerdrive_pub = rospy.Publisher("PowerDrive", Float64MultiArray, queue_size = 10)
        self.intake_command_pub = rospy.Publisher("intake/command", String, queue_size = 10)
        self.Garage_location_sub = rospy.Subscriber("garageLocation", Float64MultiArray, self.updateGarageLocation)
        self.imu_sub = rospy.Subscriber('CameraIMU', Float64, self.updateIMU)
        self.velocitydrive_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 5)
        self.intake_limit_switch_sub = rospy.Subscriber("intake/limitswitches", Float64MultiArray, self.limit_switch_callback)
        self.touch_sensor_sub = rospy.Subscriber("TouchSensors", Float64MultiArray, self.updateTouchSensorVals)

    def setAllZeros(self):
        self.GarageDepth = 0
        self.GarageAngle = 0

        self.updatedLocation = False
        self.running = False

        self.prevTriggerVal = 1

        self.imu = 0


        self.driveToGarage = False
        self.prevGarageAngle = 0

        self.runPID = False

        self.moveToGarageRunning = False
        self.movementInitTime = 0
        
        self.finished = False

        self.reachedEstimate = False

    def set_intake_state(self, state: str):
        self.intake_command_pub.publish(state)

    def updateTouchSensorVals(self, touchSensors):
        self.touchSensorVals = touchSensors.data

    def updateGarageLocation(self, location):
        GarageLocation = location.data
        print(GarageLocation)
        # if(self.runPID == False and self.driveToGarage == False):
        if(self.allowSearch == True):
            self.GarageDepth = GarageLocation[0]
            self.GarageAngle = GarageLocation[1]
            self.setPoint = self.imu + self.GarageAngle
            self.setPoint = self.setPoint + 360
            self.setPoint = self.setPoint%360
            self.GaragePID.setSetPoint(self.setPoint)
            self.updatedLocation = True
            self.prevGarageAngle = self.GarageAngle
        # print("SETPOINT: ", self.setPoint)

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
         
    def moveToGarage(self):
        # print("SETPOINT: ", self.setPoint, " CURRENT: ", self.imu)
        result = self.GaragePID.updatePID(self.imu) # this returns in radians/sec
        result = -result
        self.velocityMsg.linear.x = 0.25 # m/s
        self.velocityMsg.angular.z = result
        self.velocitydrive_pub.publish(self.velocityMsg)
        if(time.time() - self.movementInitTime > self.GarageDepth/0.25 or self.touchSensorVals[0] != 0 or self.touchSensorVals[1] != 0):
            # print("WANTED ANGLE: ", self.setPoint)
            # print("CURRENT ANGLE: ", self.imu)
            self.driveToGarage = False
            self.moveToGarageRunning = False
            self.velocityMsg.linear.x = 0
            self.velocityMsg.angular.z = 0
            self.velocitydrive_pub.publish(self.velocityMsg)
            self._result.finished = True
            self.movementInitTime = 0
            self.finished = True
            self.allowSearch = False
    def limit_switch_callback(self, data):
        pass
    def turnToGarage(self):
        print("GOT GARAGE CALLBACK")
        goal = self._as.accept_new_goal()
        print("ACCEPTED GOAL TO DOCK")
        estimatedGarageAngle = goal.estimatedGarageAngle
        estimatedGarageAngle = estimatedGarageAngle + self.imu
        estimatedGarageAngle = estimatedGarageAngle%360
        self.estimatePID.setSetPoint(estimatedGarageAngle)
        self._result.finished = False

        searchInitTime = time.time()
     
        # while(self.reachedEstimate == False and self.GarageAngle == 0):
        #     # print("TRYING TO REACH ESTIMATE")
        #     result = self.estimatePID.updatePID(self.imu) # this returns in radians/sec
        #     print("SETPOINT: ", estimatedGarageAngle, " CURRENT ANGLE: ", self.imu)

        #     self.velocityMsg.linear.x = 0
        #     self.velocityMsg.angular.z = result

        #     self.velocitydrive_pub.publish(self.velocityMsg)

        #     if(abs(self.imu - estimatedGarageAngle) < 5):
        #         print("GOT TO ESTIMATED TARGET!!!!")
        #         self.reachedEstimate = True
        #         self.velocityMsg.linear.x = 0
        #         self.velocityMsg.angular.z = 0
        #         self.velocitydrive_pub.publish(self.velocityMsg)

        while(time.time() - searchInitTime < 4):
            print(time.time() - searchInitTime)

        if(self.GarageAngle != 0):
            self.runPID = True
        else:
            self._result.finished = False
            self.setAllZeros()
            self._as.set_succeeded(self._result)

        while(self._result.finished == False):
            if(self.runPID):
                self.allowSearch = False
                result = self.GaragePID.updatePID(self.imu) # pid returns in radians/sec
                result = -result
                print("Turning - " , " SETPOINT: ", self.setPoint, " CURRENT: ", self.imu)

                self.velocityMsg.linear.x = 0
                self.velocityMsg.angular.z = result

                self.velocitydrive_pub.publish(self.velocityMsg)

                self._feedback.result = result
                self._as.publish_feedback(self._feedback)

                if(abs(self.imu - self.setPoint) < 7):
                    print("GOT TO TARGET!!!!")
                    self.driveToGarage = True
                    self.runPID = False
                    self.velocityMsg.linear.x = 0
                    self.velocityMsg.angular.z = 0
                    self.velocitydrive_pub.publish(self.velocityMsg)
            elif(self.driveToGarage):
                if(self.moveToGarageRunning == False):
                    self.movementInitTime = time.time()
                    self.moveToGarageRunning = True
                self.moveToGarage()
        self.setAllZeros()
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
    pickup = moveToGarage()
    pickup.run()