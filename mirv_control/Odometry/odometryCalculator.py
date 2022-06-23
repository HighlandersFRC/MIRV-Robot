import rospy
from filter import KalmanFilter
import math

#gps
pos = [5, 6, 7, 9, 10]
pos_sig = 2

#encoder v
mot = [1, 1, 2, 1, 1]
mot_sig = 1

x_filter = KalmanFilter(0, 1)
y_filter = KalmanFilter(0, 1)
theta_filter = KalmanFilter(math.pi / 2, 2)

def run():
    rospy.init_node("odometryCalculator.py")
    for i in range(len(pos)):
        x_filter.update(pos[i], pos_sig, mot[i], mot_sig)
    rospy.spin()

if __name__ == "__main__":
    run()