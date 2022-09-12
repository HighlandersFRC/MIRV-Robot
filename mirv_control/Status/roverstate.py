#!/bin/usr/env python3
import time

HEALTH_STATES = ["unhealthy", "degraded", "healthy", "unavailable"]
ROVER_STATES = ["disconnected","disconnected_fault", "e_stop", "connected_disabled", "connected_idle_roaming", "connected_idle_docked","connected_fault","autonomous","remote_operation"]
ROVER_STATUSES = ["available", "unavailable"]

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
            "timestamp": int(time.time() * 1000),
            "rover_id": "rover_1",
            "state": ROVER_STATES[8],
            "docked": False,
            "status": ROVER_STATUSES[0],
            "docked": False,
            "active_command": "",
            "battery_percent": 0,
            "battery_voltage": 0,
            "subsystems": {
                "electronics": {"health": HEALTH_STATES[2]},
                "drivetrain": {"health": HEALTH_STATES[2]},
                "intake": {"health": HEALTH_STATES[2]},
                "sensors": {"health": HEALTH_STATES[2]},
                "garage": {"health": HEALTH_STATES[0]},
                "power": {"health": HEALTH_STATES[2]},
                "general": {"health": HEALTH_STATES[2]}
            },
            "telemetry": {
                "location": {
                    "lat": 0,
                    "long": 0
                },
                "heading": 0,
                "speed": 0
            },
            "pi_lits": {
                "state": "wave",
                "pi_lits_stowed_left": 4,
                "pi_lits_stowed_right": 4,
                "deployed_pi_lits": []
            },
            "garage": {
                "garage_id": "garage_1",
                "location": {
                    "lat": 0,
                    "long": 0
                }
            }
        }
        self.timers = {
            "battery_voltage": Timer(5),
            "gps": Timer(1),
            "encoders": Timer(1),
            "camera_frames": Timer(1),
            "heading": Timer(1),
            "pilit_table": Timer(5),
            "garage":Timer(10),
        }

    def update_timestamp(self):
        self.rover_state["timestamp"] = int(time.time() * 1000)
