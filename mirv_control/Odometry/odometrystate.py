#!/usr/bin/env python3
import math
import csv
import os

class State:
    def __init__(self):
        self.gps_x = 0
        self.gps_y = 0
        self.encoder_vel = 0
        self.imu_angle = math.pi / 2.0
        self.encoder_vel_x = 0
        self.encoder_vel_y = 0
        os.chdir(f"/home/{os.getlogin()}/mirv_ws")
        try:
            os.remove("src/MIRV-Robot/mirv_control/Odometry/output/recording.csv")
        except:
            pass
        self.f = open("src/MIRV-Robot/mirv_control/Odometry/output/recording.csv", "w", newline = "")
        self.writer = csv.writer(self.f)

    def update_xy_vel(self):
        self.encoder_vel_x = self.encoder_vel * math.cos(self.imu_angle)
        self.encoder_vel_y = self.encoder_vel * math.sin(self.imu_angle)

    def record(self):
        self.writer.writerow([self.gps_x, self.gps_y, self.encoder_vel, self.imu_angle])

    def close_file(self):
        self.f.close()