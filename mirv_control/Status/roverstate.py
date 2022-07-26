#!/bin/usr/env python3
import time

HEALTH_STATES = ["unhealthy", "degraded", "healthy", "unavailable"]
ROVER_STATES = ["docked", "remoteOperation", "disabled", "eStop"]
ROVER_STATUSES = ["available", "unavailable"]
ROVER_LOCATION = [-104.969523, 40.474083]

class Timer:
    def __init__(self, timeout):
        self.timeout = timeout
        self.start = time.time()
        self.period = 0

    #Update period (time since data recieved)
    def update(self):
        self.period = time.time() - self.start
        return self.period < self.timeout

    #Reset period
    def reset(self):
        self.start = time.time()


class RoverState:
    def __init__(self):
        self.rover_state = {
            "rover_id": "rover_1",
            "state": ROVER_STATES[0],
            "status": ROVER_STATUSES[0],
            "battery_percent": 100,
            "battery_voltage": 12.6,
            "health": {
                "electronics": HEALTH_STATES[2],
                "drivetrain": HEALTH_STATES[2],
                "intake": HEALTH_STATES[2],
                "sensors": HEALTH_STATES[2],
                "garage": HEALTH_STATES[2],
                "power": HEALTH_STATES[2],
                "general": HEALTH_STATES[2]
            },
            "telemetry": {
                "location": {
                    "lat": ROVER_LOCATION[0],
                    "long": ROVER_LOCATION[1]
                },
                "heading": 90,
                "speed": 0
            },
            "pi_lits": {
                "state": "sequential_1",
                "pi_lits_stowed_left": 4,
                "pi_lits_stowed_right": 4,
                "deployed_pi_lits": []
            }
        }
        self.timers = {
            "battery_voltage": Timer(5),
            "gps": Timer(1),
            "encoders": Timer(1),
            "camera_frames": Timer(1),
            "camera_imu": Timer(1),
            "pilit_table": Timer(5)
        }