#!/usr/bin/env python
import time
import os
import subprocess
import rospy
from std_msgs.msg import Bool
DI1=325
DI2=331
DI3=326
def digitalRead(pin):
    return(subprocess.check_output(["cat /sys/class/gpio/gpio{}/value".format(pin)], shell=True))
def main():
    pub1 = rospy.Publisher("/DIO/DI1", Bool, queue_size = 10)
    pub2 = rospy.Publisher("/DIO/DI2", Bool, queue_size = 10)
    pub3 = rospy.Publisher("/DOI/DI3", Bool, queue_size = 10)
    rospy.init_node("DIPublisher")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub1.publish(digitalRead(DI1))
        pub2.publish(digitalRead(DI2))
        pub3.publish(digitalRead(DI3))
        rate.sleep()


if __name__ == '__main__':
    main()