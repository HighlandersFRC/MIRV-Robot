#!/usr/bin/env python3
import math
import csv
import os
from datetime import datetime
import time


class State:
    def __init__(self):
        self.gps_lat = 0
        self.gps_long = 0
        self.gps_heading = 0
        self.gps_vel = 0
        self.rtk_lat = 0
        self.rtk_long = 0
        self.rtk_heading = 0
        self.rtk_vel = 0
        self.encoder_vel_left = 0
        self.encoder_vel_right = 0
        self.target_vel = 0
        self.target_angular_vel = 0
        self.imu_angle = math.pi / 2.0
        self.encoder_vel_x = 0
        self.encoder_vel_y = 0
        # os.chdir(f"/home/{os.getlogin()}/mirv_ws/")
        self.f = open(f"/home/{os.getlogin()}/mirv_ws/src/MIRV-Robot/mirv_control/Odometry/output/recording-{datetime.now()}.csv", "w", newline = "")
        self.writer = csv.writer(self.f)
        self.writer.writerow(["time", "gps_lat", "gps_long", "rtk_lat", "rtk_long"])

    def record(self):
        self.writer.writerow([time.time(), self.gps_lat, self.gps_long, self.rtk_lat, self.rtk_long])

    def close_file(self):
        self.f.close()
