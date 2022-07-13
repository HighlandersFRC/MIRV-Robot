#!/usr/bin/env python3
import json
import rospy
import time
from std_msgs.msg import String

rospy.init_node('RobotStatusPublisher', anonymous=True)

statusPub = rospy.Publisher('RobotStatus', String, queue_size=1)


while True:
    statusPub.publish(json.dumps({
        "roverId": "rover_6",
        "state": "docked",
        "status": "available",
        "battery-percent": 12,
        "battery-voltage": 18,
        "health": {
            "electronics": "healthy",
            "drivetrain": "healthy",
            "intake": "healthy",
            "sensors": "healthy",
            "garage": "healthy",
            "power": "healthy",
            "general": "healthy"
        },
        "telemetry": {
            "lat": 39,
            "long": -105,
            "heading": 90,
            "speed": 0
        }
    }))

    time.sleep(1)

