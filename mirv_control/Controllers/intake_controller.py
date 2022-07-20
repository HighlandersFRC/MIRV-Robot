from tracemalloc import start
import rospy
from std_msgs.msg import String, Float64MultiArray
import time

class IntakeController:
    def __init__(self):
        self.intake_command_pub = rospy.Publisher("intake/command", String, queue_size = 10)
        self.intake_limit_switch_sub = rospy.Subscriber("intake/limitswitches", Float64MultiArray, self.limit_switch_callback)
        self.limit_switches = [0, 0, 1, 1]

    def limit_switch_callback(self, switches):
        self.limit_switchs = switches.data

    def intake_and_store(self, timeout: float):
        start_time = time.time()
        self.intake_command_pub.publish(String("intake"))
        had_pilit = False
        is_canceled = False
        while self.limit_switches[3] and self.limit_switches[2]:
            if not had_pilit and time.time() - start_time > timeout or not is_canceled:
                return False
        self.intake_command_pub.publish(String("store"))
        while not self.limit_switches[0]:
            pass
        return True
        
    def deposit(self, timeout: float):
        start_time = time.time()
        self.intake_command_pub.publish(String("deposit"))
        while self.limit_switches[3] and self.limit_switches[2]:
            if time.time() - start_time > timeout:
                return False
        while not self.limit_switches[1]:
            pass
        time.sleep(1)
        return True