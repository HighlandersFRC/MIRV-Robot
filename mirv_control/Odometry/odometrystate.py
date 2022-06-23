import math
import csv
import os

class State:
    def __init__(self):
        self.gps_x = 0
        self.gps_y = 0
        self.encoder_vel = 0
        self.imu_angle = math.pi / 2.0
        self.f = None

    def open_file(self):
        os.remove("output/recording.csv")
        self.f = open(f"output/recording.csv", "w")

    def close_file(self):
        self.f.close()