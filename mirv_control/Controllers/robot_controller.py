#!usr/bin/env python3
import math
import rospy
from std_msgs.msg import Float64MultiArray, String, Float64
import PID

class RobotController:
    def __init__(self):
        self.powerdrive_pub = rospy.Publisher("PowerDrive", Float64MultiArray, queue_size = 10)
        self.intake_command_pub = rospy.Publisher("intake/command", String, queue_size = 10)
        self.pilit_location_sub = rospy.Subscriber("piLitLocation", Float64MultiArray, self.updatePiLitLocation)
        self.imu_sub = rospy.Subscriber('CameraIMU', Float64, self.updateIMU)

        self.piLitDepth = 0
        self.piLitAngle = 0

        self.updatedLocation = False

        self.imu = 0

        self.kP = 0
        self.kI = 0
        self.kD = 0
        self.setPoint = 0

        self.piLitPID = PID(self.kP, self.kI, self.kD, self.setPoint)

    def set_intake_state(self, state: str):
        self.intake_command_pub.publish(state)

    def power_drive(self, left, right):
        powers = Float64MultiArray()
        powers.data = [left, right]
        self.powerdrive_pub.publish(powers)

    def updatePiLitLocation(self, location):
        piLitLocation = location.data
        self.piLitDepth = piLitLocation[0]
        self.piLitAngle = piLitLocation[1]
        self.setPoint = self.imu + self.piLitAngle
        self.piLitPID.setSetPoint(self.setPoint)

    def updateIMU(self, data):
        self.imu = data.data

    def turnToPiLit(self):
        while True:
            result = self.piLitPID.updatePID(self.imuAngle)

            self.power_drive(-result, result)

            if(math.abs(self.imuAngle - self.setPoint) < 0.1):
                break

            rospy.sleep(0.5)



