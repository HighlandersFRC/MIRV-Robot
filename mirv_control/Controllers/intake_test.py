#!/usr/bin/env python3
import rospy
#from robot_controller import RobotController
import time
from std_msgs.msg import String, Float64MultiArray

intake_command_pub = rospy.Publisher("/intake/command", String, queue_size = 10)
limit_switches = [1, 1, 0, 0]
intake_is_canceled = False

def limit_switch_callback(switches):
    limit_switches = switches.data
    print(limit_switches)

def intake_and_store(timeout: float):
    start_time = time.time()
    print("Intaking")
    intake_command_pub.publish(String("intake"))
    while not limit_switches[3] and not limit_switches[2]:
        if time.time() - start_time > timeout or intake_is_canceled:
            print("Timed out or canceled - Intake and Store")
            return False
    intake_command_pub.publish(String("store"))
    while limit_switches[0]:
        if intake_is_canceled:
            return False
    return True
    
def deposit(timeout: float):
    start_time = time.time()
    intake_command_pub.publish(String("deposit"))
    while not limit_switches[3] and not limit_switches[2]:
        if time.time() - start_time > timeout or intake_is_canceled:
            print("Timed out or canceled - Deposit")
            return False
    while limit_switches[1]:
        if intake_is_canceled:
            return False
    time.sleep(1)
    return True

intake_limit_switch_sub = rospy.Subscriber("intake/limitswitches", Float64MultiArray, limit_switch_callback)

def run():
    rospy.init_node("IntakeTest")

    print("Intaking and storing")
    intake_and_store(5)
    print("Done intaking and storing")

    time.sleep(10)

    print("Depositing")
    deposit(5)
    print("Done depositing")
    rospy.spin()

if __name__ == "__main__":
    run()