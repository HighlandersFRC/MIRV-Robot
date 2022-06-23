#!/usr/bin/env python3


# Simple Ros Module to read in Joystick values and publish them to the mirv_drive topic



import rospy
from std_msgs.msg import String
import sys

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('JoystickDrive', anonymous = True)
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        data = str(sys.version)
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        print ('Interrupted, Exiting')
        exit()
