
#!/usr/bin/env python3
import math
import time

from numpy import True_
import rospy
from std_msgs.msg import Float64MultiArray, String, Float64
from mirv_control.Controllers.PID import PID
from geometry_msgs.msg import Twist
import actionlib
import mirv_control.msg as msg
import asyncio

rospy.init_node("piLitPickup")

class piLitPickup:
    def __init__(self):
        self._feedback = msg.MovementToPiLitFeedback()
        self._result = msg.MovementToPiLitResult()

        self._action_name = "RobotController"
        self._as = actionlib.SimpleActionServer(self._action_name, msg.MovementToPiLitAction, auto_start = False)
        self._as.register_goal_callback(self.turnToPiLit)
        self._as.register_preempt_callback(self.preemptedPickup)
        self._as.start()

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

        self.kP = 0.03
        self.kI = 0
        self.kD = 0.03
        self.setPoint = 0

        self.driveToPiLit = False

        self.runPID = False

        self.moveToPiLitRunning = False
        self.movementInitTime = 0
        
        self.finished = False

        self.setAllZeros()

        self.piLitPID = PID(self.kP, self.kI, self.kD, self.setPoint)
        self.piLitPID.setMaxMinOutput(0.4)

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

    def set_intake_state(self, state: str):
        self.intake_command_pub.publish(state)

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
        print("UPDATED IMU TO: ", self.imu, " at Time: ", time.time())

    def preemptedPickup(self):
        self.velocityMsg.linear.x = 0
        self.velocityMsg.angular.z = 0
        self.velocitydrive_pub.publish(self.velocityMsg)
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_aborted()
         
    def moveToPiLit(self):
        result = self.piLitPID.updatePID(self.imu) # this returns in radians/sec
        result = -result
        self.velocityMsg.linear.x = 0.5 # m/s
        self.velocityMsg.angular.z = result
        self.velocitydrive_pub.publish(self.velocityMsg)
        if(time.time() - self.movementInitTime > self.piLitDepth/0.5):
            print("WANTED ANGLE: ", self.setPoint)
            print("CURRENT ANGLE: ", self.imu)
            self.driveToPiLit = False
            self.moveToPiLitRunning = False
            self.velocityMsg.linear.x = 0
            self.velocityMsg.angular.z = 0
            self.velocitydrive_pub.publish(self.velocityMsg)
            self._result.finished = True
            # self._as.set_succeeded(self._result)
            self.movementInitTime = 0
            self.finished = True

    def turnToPiLit(self):
        goal = self._as.accept_new_goal()
        intakeInitTime = time.time()
        running = goal.runPID
        intakeSide = goal.intakeSide
        self._result.finished = False

        while(time.time() - intakeInitTime < 3):
            print(time.time() - intakeInitTime)
            self.set_intake_state("intake")
            self.set_intake_state(intakeSide)
        while(time.time() - intakeInitTime < 10 and self.piLitAngle == 0):
            print("HAVEN'T FOUND A PI LIT YET")
            self.velocityMsg.linear.x = 0
            self.velocityMsg.angular.z = 0
            self.velocitydrive_pub.publish(self.velocityMsg)
            self.runPID = False
        if(self.piLitAngle != 0):
            self.runPID = True
        else:
            print("TIMED OUT")
            self._as.set_succeeded(self._result)
            self.finished = True

        while(self.finished == False):
            print("RUNNING CALLBACK")
            print("RUN PID:, ", self.runPID)
            if(self.runPID):
                result = self.piLitPID.updatePID(self.imu) # this returns in radians/sec
                result = -result
                print("-----------------------------------------")

                self.velocityMsg.linear.x = 0
                self.velocityMsg.angular.z = result

                self.velocitydrive_pub.publish(self.velocityMsg)

                self._feedback.result = result
                self._as.publish_feedback(self._feedback)

                if(abs(self.imu - self.setPoint) < 5):
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
        self.setAllZeros()
        self.set_intake_state("disable")
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