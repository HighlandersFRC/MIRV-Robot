#!/usr/bin/env python3
import rospy

class ComputeHeading():
    def __init__(self):
        rospy.init_node("ComputeStartHeading")
        


if __name__ == '__main__':
    compute = ComputeHeading()
    compute.run()