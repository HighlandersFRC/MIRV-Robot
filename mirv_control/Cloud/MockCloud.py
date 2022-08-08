
import signal
import sys
import json
import time
import rospy
from std_msgs.msg import String

rospy.init_node("CloudConnection")

command_pub = rospy.Publisher('CloudCommands', String, queue_size=1)
availability_pub = rospy.Publisher('RoverAvailable', String, queue_size=1)


#pi_lit_msg = json.dumps({"subsystem": "pi_lit", "command": "sequential"})

#deploy_msg = json.dumps({"subsystem": "general", "command": "deploy_pi_lits", "commandParameters": {"formation": "taper-5", "direction": "right", "start_location": {"lat": 39, "long": -103.5}}})

#joystick_msg = json.dumps({"subsystem": "general", "command": "deploy_pi_lits", "commandParameters": {"x": 0, "y":0.1}})

cancel = json.dumps({"command":"cancel","subsystem":"general","runtimeType":"generalCommand"})

deploy_pi_lits = json.dumps({"command":"deploy_pi_lits","subsystem":"general","runtimeType":"generalCommand"})

retrieve_pi_lits = json.dumps({"command":"retrieve_pi_lits","subsystem":"general","runtimeType":"generalCommand"})


deploy = json.dumps({"command":"deploy","subsystem":"general","runtimeType":"generalCommand"})

enable = json.dumps({"command":"enable","subsystem":"general","runtimeType":"generalCommand"})


enable_remote = json.dumps({"command":"enable_remote_operation","subsystem":"general","runtimeType":"generalCommand"})

pickup_one = json.dumps({"command":"pickup_1_pi_lit","subsystem":"intake","runtimeType":"intakeCommand"})
place_one = json.dumps(	{"command":"place_1_pi_lit","subsystem":"intake","runtimeType":"intakeCommand"})
drive_rover = json.dumps({"command":"arcade","commandParameters":{"x":0.0,"y":0.0,"runtimeType":"drivetrain"},"subsystem":"drivetrain","runtimeType":"drivetrainCommand"})


disable_remote = json.dumps({"command":"disable_remote_operation","subsystem":"general","runtimeType":"generalCommand"})

goto_point = json.dumps({"command":"to_location","commandParameters":{"lat":39.0,"long":-105.0,"runtimeType":"movement"},"subsystem":"movement","runtimeType":"movementCommand"})


disable = json.dumps({"command":"disable","subsystem":"general","runtimeType":"generalCommand"})

e_stop = json.dumps({"command":"e_stop","subsystem":"general","runtimeType":"generalCommand"})

stop = json.dumps({"command":"stow","subsystem":"general","runtimeType":"generalCommand"})

heartbeat = json.dumps({"command":"heartbeat","subsystem":"heartbeat","runtimeType":"heartbeatCommand"})

running = True

sequence = [
    cancel,
    deploy_pi_lits,
    retrieve_pi_lits,
    deploy,
    enable,
    heartbeat,
    enable_remote,
    pickup_one,
    place_one,
    drive_rover,
    disable_remote,
    goto_point,
    disable,
    e_stop
]




index = 0
while not rospy.is_shutdown():


    print("Press Enter to send an input, type q to quit")
    step = input()
    if step == 'q':
        exit()
    command_pub.publish(sequence[index])
    print("Sending Message:", sequence[index])


    index +=1
    index = index % len(sequence)
