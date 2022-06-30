#!usr/bin/env python3
import math
import rospy
from std_msgs.msg import Float64MultiArray, String

import controlLoops.PID as PID

class RobotController:
    def __init__(self):
        self.powerdrive_pub = rospy.Publisher("PowerDrive", Float64MultiArray, queue_size = 10)
        self.intake_command_pub = rospy.Publisher("intake/command", String, queue_size = 10)

        self.kP = 0
        self.kI = 0
        self.kD = 0
        self.setPoint = math.pi/2

        self.piLitPID = PID(self.kP, self.kI, self.kD, self.setPoint)

    def set_intake_state(self, state: str):
        self.intake_command_pub.publish(state)

    def power_drive(self, left, right):
        powers = Float64MultiArray()
        powers.data = [left, right]
        self.powerdrive_pub.publish(powers)

    def turnToPiLit(self):

        while not rospy.is_shutdown:
            piLitLocation = (rospy.wait_for_message("piLitLocation", Float64MultiArray, timeout = 1)).data

            depth = piLitLocation[0]
            angle = piLitLocation[1]

            result = self.piLitPID.updatePID(angle)

            self.power_drive(-result, result)

            if(math.abs(self.setPoint - angle) < 0.1):
                break

            rospy.sleep(0.5)



