#!/usr/bin/env python3
import math
import time
import rospy
import actionlib
import mirv_control.msg as msg
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String, Float64
from PID import PID


class PickupPilit():
    def __init__(self):
        rospy.init_node("PickupPilit", anonymous=True)
        self._feedback = msg.PickupPilitFeedback()
        self._result = msg.PickupPilitResult()

        self._action_name = "PickupAS"
        self._as = actionlib.SimpleActionServer(
            self._action_name, msg.PickupPilitAction, auto_start=False)
        self._as.register_goal_callback(self.pickupPilit)
        self._as.register_preempt_callback(self.preemptedPickupPilit)
        self._as.start()

        self.velocityMsg = Twist()
        self.imu = 0
        self.limit_switches = [0, 0, 1, 1]
        self.piLitDepth = 0
        self.piLitAngle = 0
        self.setPoint = 0
        self.foundPiLit = False
        self.allowSearch = False
        self.intakeSide = "switch_left"

        self.piLitPID = PID(0.02, 0.000001, 2, 0)
        self.piLitPID.setMaxMinOutput(0.5)
        self.piLitPID.setContinuous(360,0)
        self.piLitPID.setIZone(8)

        self.movementPID = PID(0.0175, 0, 2, 0)
        self.piLitPID.setMaxMinOutput(0.5)
        self.piLitPID.setContinuous(360,0)

        self.intake_command_pub = rospy.Publisher("intake/command", String, queue_size = 10)
        self.pilit_location_sub = rospy.Subscriber("piLitLocation", Float64MultiArray, self.updatePiLitLocation)
        self.imu_sub = rospy.Subscriber('CameraIMU', Float64, self.updateIMU) 
        self.velocitydrive_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 5)
        self.intake_limit_switch_sub = rospy.Subscriber("intake/limitswitches", Float64MultiArray, self.limit_switch_callback)
    
    def setDefault(self):
        self.velocityMsg = Twist()
        self.piLitDepth = 0
        self.piLitAngle = 0
        self.setPoint = 0
        self.foundPiLit = False
        self.allowSearch = False
        self.intakeSide = "switch_left"
    
    def limit_switch_callback(self, switches):
        self.limit_switches = switches.data

    def updatePiLitLocation(self, location):
        piLitLocation = location.data
        print("Got a PI Lit")
        if(self.allowSearch == True):
            self.piLitDepth = piLitLocation[0]
            self.piLitAngle = piLitLocation[1]
            self.setPoint = self.imu + self.piLitAngle
            print("Pi Lit Angle: ", self.piLitAngle, "Pi Lit Distance", self.piLitDepth)
            if self.intakeSide == "switch_left":
                self.setPoint = self.setPoint + 360
            else:
                self.setPoint = self.setPoint + 360
            self.setPoint = self.setPoint%360
            self.piLitPID.setSetPoint(self.setPoint)
            self.updatedLocation = True
            self.prevPiLitAngle = self.piLitAngle
            print("SETPOINT: ", self.setPoint)
            self.foundPiLit = True
            #self.allowSearch = False


    def set_intake_state(self, state: str):
        self.intake_command_pub.publish(state)


    def updateIMU(self, data):
        self.imu = data.data

    def drive(self, forward, turn):
        self.velocityMsg.linear.x = forward
        self.velocityMsg.angular.z = turn
        self.velocitydrive_pub.publish(self.velocityMsg)

    def searchForPilits(self):
        self.allowSearch = True
        print("Searching For Pilits")
        searchStartTime = time.time()
        while abs(searchStartTime - time.time()) < 10: #and not self.foundPiLit:
            self.drive(0,0)

            

        self.allowSearch = False

    def turn(self, angle):
        print(f"Turning Towards {angle} Starting Angle: {self.imu}")
        alignStartTime = time.time()
        error = angle - self.imu
        result = 0
        if abs(error) > 180:
            if error > 0:
                error = error -360
            else:
                error = error + 360
        
        self.piLitPID.setSetPoint(angle)
        while abs(error) > 5 or result > 0.05:
            error = angle - self.imu
            if abs(error) > 180:
                if error > 0:
                    error = error -360
                else:
                    error = error + 360

            if time.time() - alignStartTime > 5:
                break

            result = self.piLitPID.updatePID(self.imu) # this returns in radians/sec
            result = -result
            self.drive(0,result)
        self.drive(0,0)
        print(f"Turn Complete. Target Angle {angle}, Current Angle: {self.imu}")

    def pickupPilit(self):
        goal = self._as.accept_new_goal()
        self.setDefault()
        self.intakeSide = goal.intakeSide
        self.set_intake_state(self.intakeSide)
        
        
        self.foundPiLit = False

        self.searchForPilits()

        if not self.foundPiLit:
            self.turn((self.imu - 30)%360)
            self.searchForPilits()

            if not self.foundPiLit:
                self.turn((self.imu + 60)%360)
                self.searchForPilits()

        print("Search Complete Found Pi-Lit:", self.foundPiLit)

        success = False
        # Exit if no pilit was found
        if not self.foundPiLit:
            print("Failed to Find Pi-lit")
            self.set_intake_state("reset")
            self._result.finished = False
            success = False
            #self._as.set_succeeded(self._result)
        else:
            print("Aligning", self.imu, self.setPoint)

            #Align Based on Camera Feed
            self.turn(self.setPoint)

            # Send Intake Down
            intakeInitTime = time.time()
            while(time.time() - intakeInitTime < 3):
                self.drive(0,0)
                self.set_intake_state("intake")
                self.set_intake_state(self.intakeSide)
            
            print("Picking Up", self.imu, self.setPoint)
            movementInitTime = time.time()
            self.movementPID.setSetPoint(self.setPoint)
            while time.time() - movementInitTime < (self.piLitDepth*1.5)/0.25  and self.limit_switches[2] != 1 and self.limit_switches[3] != 1:
                result = self.movementPID.updatePID(self.imu) # this returns in radians/sec
                result = -result # This should be negative
                self.drive(0.25,result)
            
            
            if(self.limit_switches[2] == 1 or self.limit_switches[3] == 1):
                print("Setting Finished True")
                success = True
            else:
                print("Setting Finished False")
                success = False

            storeInitTime = time.time()
            while(time.time() - storeInitTime < 3):
                    self.set_intake_state("store")
                    self.drive(0,0)
            

        self.drive(0,0)
        
        if success:
            self._result.finished = True
        else:
            self._result.finished = False

        print("Succeeded", self._result)
        self._as.set_succeeded(self._result)



    def preemptedPickupPilit(self):
        if(self._as.is_new_goal_available()):
            print("Pickup Pilit preempt received")
            self._as.set_preempted()
        else:
            print("aborting point turn")
            self.velocityMsg.linear.x = 0
            self.velocityMsg.angular.z = 0
            self.velocitydrive_pub.publish(self.velocityMsg)
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_aborted()

    def run(self):
        rospy.spin()


pickup = PickupPilit()
if __name__ == '__main__':
    print("RUNNING")
    pickup.run()
