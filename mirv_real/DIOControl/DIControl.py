#!/usr/bin/env python

# Deprecated class, use DIOControl.py
import time
import os
import subprocess
import rospy
from std_msgs.msg import Bool
# DI1=325
# DI2=331
# DI3=326
# def digitalRead(pin):
#     return(subprocess.check_output(["cat /sys/class/gpio/gpio{}/value".format(pin)], shell=True))
def main():
    pass
    # pub1 = rospy.Publisher("/DIO/DI1", Bool, queue_size = 10)
    # pub2 = rospy.Publisher("/DIO/DI2", Bool, queue_size = 10)
    # pub3 = rospy.Publisher("/DIO/DI3", Bool, queue_size = 10)
    # rospy.init_node("DIPublisher")
    # rate = rospy.Rate(2)
    # while not rospy.is_shutdown():
    #     pub1.publish(digitalRead(DI1)=="1")
    #     pub2.publish(digitalRead(DI2)=="1")
    #     pub3.publish(digitalRead(DI3)=="1")
    #     rate.sleep()
    #     print(digitalRead(DI1)=="1\n")
    #     print(digitalRead(DI2)=="1\n")
    #     print(digitalRead(DI3)=="1\n")
    #     print("\n")


if __name__ == '__main__':
    main()