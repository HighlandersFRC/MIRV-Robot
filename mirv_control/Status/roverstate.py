#!/bin/usr/env python3
from datetime import datetime
from email.header import Header

HEALTH_STATES = ["unhealthy", "degraded", "healthy", "unavailable"]
ROVER_STATES = ["docked", "remoteOperation", "disabled", "eStop"]
ROVER_STATUSES = ["available", "unavailable"]
ROVER_LOCATION = [-104.969523, 40.474083]

class Timer:
    def __init__(self, timeout):
        self.timeout = timeout
        self.start = datetime.now().timestamp()
        self.period = 0

    #Update period (time since data recieved)
    def update(self):
        self.period = datetime.now().timestamp() - self.start
        return self.period < self.timeout

    #Reset period
    def reset(self):
        self.start = datetime.now().timestamp()


class RoverState:
    def __init__(self):
        self.rover_state = {
            "state": ROVER_STATES[0],
            "status": ROVER_STATUSES[0],
            "battery-percent": 100,
            "battery-voltage": 14,
            "health": {
                "battery-voltage": HEALTH_STATES[2],
                "gps": HEALTH_STATES[2],
                "encoders": HEALTH_STATES[2],
                "camera-frames": HEALTH_STATES[2],
                "camera-imu": HEALTH_STATES[2]
            },
            "telemetry": {
                "lat": ROVER_LOCATION[0],
                "long": ROVER_LOCATION[1],
                "heading": 90,
                "speed": 0
            }
        }
        self.timers = {
            "battery-voltage": Timer(5),
            "gps": Timer(1),
            "encoders": Timer(1),
            "camera-frames": Timer(1),
            "camera-imu": Timer(1)
        }