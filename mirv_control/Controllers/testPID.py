#!usr/bin/env python3
import math
import rospy
from std_msgs.msg import Float64MultiArray, String, Float64
from PID import PID

class testPID:
    def __init__(self):
        self.powerdrive_pub = rospy.Publisher("PowerDrive", Float64MultiArray, queue_size = 10)
        self.intake_command_pub = rospy.Publisher("intake/command", String, queue_size = 10)
        self.pilit_location_sub = rospy.Subscriber("piLitLocation", Float64MultiArray, self.updatePiLitLocation)
        self.imu_sub = rospy.Subscriber('CameraIMU', Float64, self.updateIMU)

        self.piLitDepth = 0
        self.piLitAngle = 0

        self.updatedLocation = False

        self.imu = 0

        self.kP = 0.25
        self.kI = 0
        self.kD = 0
        self.setPoint = 0

        self.piLitPID.setMaxMinOutput(0.2)

        self.piLitPID = PID(self.kP, self.kI, self.kD, self.setPoint)

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
            print("ASDJFKLAJSDKLFJASLKDFJ")
            result = self.piLitPID.updatePID(self.imu)

            print(result)

            if(abs(self.imu - self.setPoint) < 0.05):
                break

            if(self.piLitAngle == 0):
                break

            rospy.sleep(0.5)
