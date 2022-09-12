#!/usr/bin/env python
import time
import os
import subprocess
import rospy
from std_msgs.msg import String, Bool

# Pin Definitions
# Pins have negative Logic
# A value of 0 in the file, is output HIGH
# A value of 1 in the file is outpput LOW



#                    function       |      HI (0)               |    LO (1)
DO0=320 ## tp4       reverse        | Rev enable                | Rev Disable
DO1=321 ## header 6  inhibit        | Flash disable             | Flash enable 
DO2=322 ## header 7  reset          | active low reset with low pulse
DO3=323 ## tp5       pattern        | Simultaneous              | Wave
DI1=325 
DI2=331
DI3=326 ##           Connection     | 
## note: Control board is wired so binary LOW is HIGH, and binary HIGH is LOW 
outputsDict = {"DO0": DO0, "DO1": DO1, "DO2": DO2, "DO3": DO3}
def digitalRead(pin):
    return(subprocess.check_output(["cat /sys/class/gpio/gpio{}/value".format(pin)], shell=True))
def digitalWrite(pin, state):
    print("tunring pin {} to {}".format(pin, state))
    subprocess.call(["echo {} >/sys/class/gpio/gpio{}/value".format(state, pin)], shell=True)
def callback(data):
    print(str(data.data))
    inputData = str(data.data).split(",", 1)
    digitalWrite(outputsDict[inputData[0]], inputData[1])

def main():
    rospy.init_node("DIOController", anonymous=True)
    rate = rospy.Rate(2)
    pub1 = rospy.Publisher("/DIO/DI1", Bool, queue_size = 5)
    pub2 = rospy.Publisher("/DIO/DI2", Bool, queue_size = 5)
    pub3 = rospy.Publisher("/DIO/DI3", Bool, queue_size = 5)
    rospy.Subscriber("DOControl", String, callback)
    while not rospy.is_shutdown():
        pub1.publish(digitalRead(DI1)=="1")
        pub2.publish(digitalRead(DI2)=="1")
        pub3.publish(digitalRead(DI3)=="1")
        rate.sleep()
        print(digitalRead(DI1)=="1\n")
        print(digitalRead(DI2)=="1\n")
        print(digitalRead(DI3)=="1\n")
        print("\n")
if __name__ == '__main__':
    main()
