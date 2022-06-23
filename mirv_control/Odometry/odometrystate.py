import math
import csv

class State:
    def __init__(self):
        self.gps_x = 0
        self.gps_y = 0
        self.encoder_vel = 0
        self.imu_angle = math.pi / 2.0
        self.writer = csv.writer(open("output/recording.csv", "w+"))

    def record(self):
        self.writer.writerow([self.gps_x, self.gps_y, self.encoder_vel, self.imu_angle])

    def close_file(self):
        self.f.close()