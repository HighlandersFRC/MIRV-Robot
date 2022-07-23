#!/usr/bin/env python3
from tablemanager import TableManager
import rospy
from mirv_description.msg import pilit_db_msg
import time

pilit_table_cmd = """ CREATE TABLE IF NOT EXISTS pilits (
    timestamp double PRIMARY KEY,
    right_count integer NOT NULL,
    left_count integer NOT NULL,
    p1_lat double NOT NULL,
    p1_long double NOT NULL,
    p1_elev double NOT NULL,
    p2_lat double NOT NULL,
    p2_long double NOT NULL,
    p2_elev double NOT NULL,
    p3_lat double NOT NULL,
    p3_long double NOT NULL,
    p3_elev double NOT NULL,
    p4_lat double NOT NULL,
    p4_long double NOT NULL,
    p4_elev double NOT NULL,
    p5_lat double NOT NULL,
    p5_long double NOT NULL,
    p5_elev double NOT NULL,
    p6_lat double NOT NULL,
    p6_long double NOT NULL,
    p6_elev double NOT NULL,
    p7_lat double NOT NULL,
    p7_long double NOT NULL,
    p7_elev double NOT NULL,
    p8_lat double NOT NULL,
    p8_long double NOT NULL,
    p8_elev double NOT NULL
); """
pilit_table_columns = (
    "timestamp",
    "right_count",
    "left_count",
    "p1_lat",
    "p1_long",
    "p1_elev",
    "p2_lat",
    "p2_long",
    "p2_elev",
    "p3_lat",
    "p3_long",
    "p3_elev",
    "p4_lat",
    "p4_long",
    "p4_elev",
    "p5_lat",
    "p5_long",
    "p5_elev",
    "p6_lat",
    "p6_long",
    "p6_elev",
    "p7_lat",
    "p7_long",
    "p7_elev",
    "p8_lat",
    "p8_long",
    "p8_elev"
)

def pilit_callback(pilit_event):
    if pilit_event.deploy_or_retrieve.data == "deploy":
        tm.deployed_pilit(pilit_event.gps_pos, pilit_event.side.data, pilit_table_columns)
    elif pilit_event.deploy_or_retrieve.data == "retrieve":
        tm.retrieved_pilit(pilit_event.gps_pos, pilit_event.side.data, pilit_table_columns)

if __name__ == "__main__":
    rospy.init_node("DatabaseController")
    pilit_sub = rospy.Subscriber("pilit/events", pilit_db_msg, pilit_callback)

    tm = TableManager()
    tm.connect(r"mirv.db")
    tm.create_table(pilit_table_cmd)
    tm.append_row("pilits", pilit_table_columns, (time.time(), 4, 4, 0, "stored", 0, 0, 0, 0, "stored", 0, 0, 0, 0, "stored", 0, 0, 0, 0, "stored", 0, 0, 0, 0, "stored", 0, 0, 0, 0, "stored", 0, 0, 0, 0, "stored", 0, 0, 0, 0, "stored", 0, 0, 0))

    while not rospy.is_shutdown():
        pass