#!/usr/bin/env python3
import math
import csv
import os
from datetime import datetime

class State:
    def __init__(self):
        self.gps_x = 0
        self.gps_y = 0
        self.encoder_vel_left = 0
        self.encoder_vel_right = 0
        self.target_vel_left = 0
        self.target_vel_right = 0
        self.imu_angle = math.pi / 2.0
        self.encoder_vel_x = 0
        self.encoder_vel_y = 0
        os.chdir(f"/home/{os.getlogin()}/mirv_ws")
        self.f = open(f"src/MIRV-Robot/mirv_control/Odometry/output/recording-{datetime.now()}.csv", "w", newline = "")
        self.writer = csv.writer(self.f)

    def update_xy_vel(self):
        print(f"Left: {self.target_vel_left - self.encoder_vel_left} Right: {self.target_vel_right - self.encoder_vel_right}")

    def record(self):
        self.writer.writerow([self.gps_x, self.gps_y, self.imu_angle, self.encoder_vel_left, self.encoder_vel_right, self.target_vel_left, self.target_vel_right])

    def close_file(self):
        self.f.close()