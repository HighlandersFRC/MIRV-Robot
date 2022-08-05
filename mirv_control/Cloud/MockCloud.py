
import signal
import sys
import json
import time
import rospy
from std_msgs.msg import String

rospy.init_node("CloudConnection")

command_pub = rospy.Publisher('CloudCommands', String, queue_size=1)
availability_pub = rospy.Publisher('RoverAvailable', String, queue_size=1)


pi_lit_msg = json.dumps({"subsystem": "pi_lit", "command": "sequential"})

deploy_msg = json.dumps({"subsystem": "general", "command": "deploy_pi_lits", "commandParameters": {"formation": "taper-5", "direction": "right", "start_location": {"lat": 39, "long": -103.5}}})

joystick_msg = json.dumps({"subsystem": "general", "command": "deploy_pi_lits", "commandParameters": {"x": 0, "y":0.1}})


running = True

while not rospy.is_shutdown():
    command_pub.publish(pi_lit_msg)
    print("Sending Message:", pi_lit_msg)
    time.sleep(5)
