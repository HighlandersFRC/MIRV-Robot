#!/usr/bin/env python3
from tablemanager import TableManager
import rospy
from mirv_description.msg import pilit_db_msg
import time
import actionlib
import mirv_control.msg as msg

pilit_table_cmd = """ CREATE TABLE IF NOT EXISTS pilits (
    timestamp double PRIMARY KEY,
    right_count integer NOT NULL,
    left_count integer NOT NULL,
    p1_timestamp double NOT NULL,
    p1_state text NOT NULL,
    p1_lat double NOT NULL,
    p1_long double NOT NULL,
    p1_elev double NOT NULL,
    p2_timestamp double NOT NULL,
    p2_state text NOT NULL,
    p2_lat double NOT NULL,
    p2_long double NOT NULL,
    p2_elev double NOT NULL,
    p3_timestamp double NOT NULL,
    p3_state text NOT NULL,
    p3_lat double NOT NULL,
    p3_long double NOT NULL,
    p3_elev double NOT NULL,
    p4_timestamp double NOT NULL,
    p4_state text NOT NULL,
    p4_lat double NOT NULL,
    p4_long double NOT NULL,
    p4_elev double NOT NULL,
    p5_timestamp double NOT NULL,
    p5_state text NOT NULL,
    p5_lat double NOT NULL,
    p5_long double NOT NULL,
    p5_elev double NOT NULL,
    p6_timestamp double NOT NULL,
    p6_state text NOT NULL,
    p6_lat double NOT NULL,
    p6_long double NOT NULL,
    p6_elev double NOT NULL,
    p7_timestamp double NOT NULL,
    p7_state text NOT NULL,
    p7_lat double NOT NULL,
    p7_long double NOT NULL,
    p7_elev double NOT NULL,
    p8_timestamp double NOT NULL,
    p8_state text NOT NULL,
    p8_lat double NOT NULL,
    p8_long double NOT NULL,
    p8_elev double NOT NULL
); """
pilit_table_columns = (
    "timestamp",
    "right_count",
    "left_count",
    "p1_timestamp",
    "p1_state",
    "p1_lat",
    "p1_long",
    "p1_elev",
    "p2_timestamp",
    "p2_state",
    "p2_lat",
    "p2_long",
    "p2_elev",
    "p3_timestamp",
    "p3_state",
    "p3_lat",
    "p3_long",
    "p3_elev",
    "p4_timestamp",
    "p4_state",
    "p4_lat",
    "p4_long",
    "p4_elev",
    "p5_timestamp",
    "p5_state",
    "p5_lat",
    "p5_long",
    "p5_elev",
    "p6_timestamp",
    "p6_state",
    "p6_lat",
    "p6_long",
    "p6_elev",
    "p7_timestamp",
    "p7_state",
    "p7_lat",
    "p7_long",
    "p7_elev",
    "p8_timestamp",
    "p8_state",
    "p8_lat",
    "p8_long",
    "p8_elev"
)

def pilit_callback(pilit_event):
    if pilit_event.deploy_or_retrieve.data == "deploy":
        tm.deployed_pilit(pilit_event.gps_pos, pilit_event.side.data, pilit_table_columns)
    elif pilit_event.deploy_or_retrieve.data == "retrieve":
        tm.retrieved_pilit(pilit_event.gps_pos, pilit_event.side.data, pilit_table_columns)

def query_callback():
    goal = action_server.accept_new_goal()
    feedback = msg.DatabaseFeedback()
    result = msg.DatabaseResult()
    t = TableManager()
    t.connect(r"mirv.db")
    data = t.get_last_row("pilits")
    t.close()
    result.latitude = data[5::5]
    result.longitude = data[6::5]
    result.altitude = data[7::5]
    action_server.set_succeeded(result)

if __name__ == "__main__":
    rospy.init_node("DatabaseController")
    pilit_sub = rospy.Subscriber("pilit/events", pilit_db_msg, pilit_callback)
    action_server = actionlib.SimpleActionServer("Database", msg.DatabaseAction, auto_start = False)
    action_server.register_goal_callback(query_callback)
    action_server.start()

    tm = TableManager()
    tm.connect(r"mirv.db")
    tm.create_table(pilit_table_cmd)
    tm.append_row("pilits", pilit_table_columns, (time.time(), 4, 4, 0, "stored", 0, 0, 0, 0, "stored", 0, 0, 0, 0, "stored", 0, 0, 0, 0, "stored", 0, 0, 0, 0, "stored", 0, 0, 0, 0, "stored", 0, 0, 0, 0, "stored", 0, 0, 0, 0, "stored", 0, 0, 0))

    while not rospy.is_shutdown():
        pass