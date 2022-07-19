#!usr/bin/env python3
import math
import time

from numpy import True_
import rospy
from std_msgs.msg import Float64MultiArray, String, Float64
from PID import PID
from geometry_msgs.msg import Twist

class RobotController:
    def __init__(self):
        self.powerdrive_pub = rospy.Publisher("PowerDrive", Float64MultiArray, queue_size = 10)
        self.intake_command_pub = rospy.Publisher("intake/command", String, queue_size = 10)
        self.pilit_location_sub = rospy.Subscriber("piLitLocation", Float64MultiArray, self.updatePiLitLocation)
        self.imu_sub = rospy.Subscriber('CameraIMU', Float64, self.updateIMU)
        self.velocitydrive_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 5)

        self.velocityMsg = Twist()

        self.piLitDepth = 0
        self.piLitAngle = 0

        self.updatedLocation = False
        self.running = False

        self.prevTriggerVal = 1

        self.imu = 0

        self.kP = 0.025
        self.kI = 0
        self.kD = 0.03
        self.setPoint = 0
        self.driveToPiLit = False
        self.prevPiLitAngle = 0

        self.runPID = False

        self.moveToPiLitRunning = False
        self.movementInitTime = 0
        
        self.imuList = []

        self.piLitPID = PID(self.kP, self.kI, self.kD, self.setPoint)
        self.piLitPID.setMaxMinOutput(0.4)

    def set_intake_state(self, state: str):
        self.intake_command_pub.publish(state)

    def power_drive(self, left, right):
        powers = Float64MultiArray()
        powers.data = [left, right]
        self.powerdrive_pub.publish(powers)

    def updatePiLitLocation(self, location):
        piLitLocation = location.data
        if(self.runPID == False and self.driveToPiLit == False):
            self.piLitDepth = piLitLocation[0]
            self.piLitAngle = piLitLocation[1]
            self.setPoint = self.imu + self.piLitAngle
            self.piLitPID.setSetPoint(self.setPoint)
            self.updatedLocation = True
            self.prevPiLitAngle = self.piLitAngle

    def updateIMU(self, data):
        self.imu = data.data
        if(self.runPID):
            result = self.piLitPID.updatePID(self.imu) # this returns in radians/sec
            result = -result
            print("-----------------------------------------")

            print("WANTED ANGLE: ", self.setPoint)
            print("CURRENT ANGLE: ", self.imu)
            print("ADJUSTMENT: ", self.piLitAngle)

            self.velocityMsg.linear.x = 0
            self.velocityMsg.angular.z = result

            self.velocitydrive_pub.publish(self.velocityMsg)

            if(abs(self.imu - self.setPoint) < 3):
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

        print("UPDATED IMU TO: ", self.imu, " at Time: ", time.time())

    def moveToPiLit(self):
        result = self.piLitPID.updatePID(self.imu) # this returns in radians/sec
        result = -result
        self.velocityMsg.linear.x = 0.25
        self.velocityMsg.angular.z = result
        self.velocitydrive_pub.publish(self.velocityMsg)
        if(time.time() - self.movementInitTime > self.piLitDepth/0.25):
            print("WANTED ANGLE: ", self.setPoint)
            print("CURRENT ANGLE: ", self.imu)
            self.driveToPiLit = False
            self.moveToPiLitRunning = False
            self.velocityMsg.linear.x = 0
            self.velocityMsg.angular.z = 0
            self.velocitydrive_pub.publish(self.velocityMsg)
            self.set_intake_state("store")

    def turnToPiLit(self, currentTriggerVal, intakeSide):
        # self.updatedLocation = False
        if(self.running == False and self.prevTriggerVal > 0):
            self.running = True
            self.prevTriggerVal = currentTriggerVal
            intakeInitTime = time.time()
            while(time.time() - intakeInitTime < 3):
                print(time.time() - intakeInitTime)
                self.set_intake_state("intake")
                # if(intakeSide == "RIGHT"):
                if(self.piLitAngle > 0):
                    self.set_intake_state("switch_right")
                else:
                    self.set_intake_state("switch_left")
            self.runPID = True