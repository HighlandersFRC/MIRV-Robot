#!/usr/bin/env python3

import subprocess
import rospy
from std_msgs.msg import Bool
from mirv_control.msg import pilit_state
from time import sleep, time


# Pin Definitions
# Pins have negative Logic
# A value of 0 in the file, is output HIGH


# A value of 1 in the file is outpput LOW
#                   Pi lit Board Pin | AGX PIN | function       |      HI (0)               |    LO (1)
REVERSE_PIN = 320   ## tp4           | DO0     | reverse        | Rev enable                | Rev Disable
INHIBIT_PIN = 321   ## header 6      | DO1     | inhibit        | Flash disable             | Flash enable 
RESET_PIN = 322     ## header 7      | DO2     | reset          | Board resets when the reset line is released to HI
PATTERN_PIN=323             ## tp5           | DO3     | pattern        | Simultaneous              | Wave
DI1=325             ## Unused        | DI1     |
DI2=331             ## Unused        | DI2     |
CONN_PIN=326             ## tp3           | DI3     | conn status    | Connected                 | No Pi-lits connected

HIGH = 0
LOW =1





class PiLitModuleController():
    
    def __init__(self):
        self.reverse = False
        self.inhibit = True
        self.wave = False
        self.new_message = False
        

    def digitalRead(self, pin):
        return(subprocess.check_output(["cat /sys/class/gpio/gpio{}/value".format(pin)], shell=True))


    def digitalWrite(self, pin, state):
        print("turning pin {} to {}".format(pin, state))
        subprocess.call(["echo {} >/sys/class/gpio/gpio{}/value".format(state, pin)], shell=True)
        
    def reset(self):
        self.digitalWrite(RESET_PIN, LOW)
        sleep(0.1)
        self.digitalWrite(RESET_PIN, HIGH)
        
    def is_connected(self):
        return int(self.digitalRead(CONN_PIN)) == 1

    def block_until_connected(self, timeout = 10):
        startTime = time()
        while not self.is_connected():
            sleep(0.1)
            if time() - startTime > timeout:
                return False
        return True

    def set_state(func):
        def inner(self):
            self.reset()
            reconnect = self.block_until_connected()
            if reconnect:
                sleep(1)
                func(self)
                return True
            return False
        return inner
        
    @set_state
    def set_wave_formation(self):
        self.digitalWrite(PATTERN_PIN, LOW)
        
    @set_state
    def set_simultaneous_formation(self):
        self.digitalWrite(PATTERN_PIN, HIGH)
        
    @set_state        
    def stop_flash(self):
        self.digitalWrite(INHIBIT_PIN, HIGH)
        self.reset()
        
    @set_state        
    def start_flash(self):
        self.digitalWrite(INHIBIT_PIN, LOW)
        self.reset()
        
    @set_state
    def set_reverse(self):
        self.digitalWrite(REVERSE_PIN, HIGH)
        
    @set_state
    def set_forward(self):
        self.digitalWrite(REVERSE_PIN, LOW)
        
    def set_state(self, inhibit, wave, reverse):
        self.inhibit = inhibit
        self.wave = wave
        self.reverse = reverse
        
    def send_state(self):
        if self.inhibit:
            self.stop_flash()
        else:
            self.start_flash()
        
        if self.wave:
            self.set_wave_formation()
        else:
            self.set_simultaneous_formation()
        
        if self.reverse:
            self.set_reverse()
        else:
            self.set_forward()

def callback(data):
    print("Setting Pilit Module State", data)
    reverse = data.reverse
    inhibit = data.inhibit
    wave = data.wave
    
    if controller.is_connected():
        controller.set_state(inhibit, wave, reverse)
        controller.send_state()
    else:
        controller.new_message = True
    
        
    
def main():
    rospy.init_node("PiLitModuleController", anonymous=True)
    rate = rospy.Rate(2)
    rospy.Subscriber("PilitControl", pilit_state, callback)
    last_connection_time = 0
    while not rospy.is_shutdown():
        connected = controller.is_connected()
        if connected:
            last_connection_time = time()
            if controller.new_message:
                controller.set_state()
                controller.new_message = False
                last_connection_time = time()
            else:
                pass
        
        elif time() - last_connection_time > 60:
            rospy.logwarn("Pi-lit Module Not connected")
            controller.reset() 
        rospy.spin()
        
        
if __name__ == '__main__':
    controller = PiLitModuleController()
    main()


