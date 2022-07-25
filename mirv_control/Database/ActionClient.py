#!/usr/bin/env python3
import rospy, actionlib
import mirv_control.msg as msg

class DatabaseClient:
    def __init__(self):
        rospy.init_node("DatabaseActionClient")
        self.client = actionlib.SimpleActionClient("Database", msg.DatabaseAction)
        self.client.wait_for_server()
        msg.DatabaseGoal.SendLatest = True
        self.goal = msg.DatabaseGoal
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        print(self.client.get_result())

dbc = DatabaseClient()