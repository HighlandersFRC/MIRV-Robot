#!/usr/bin/env python3
from tablemanager import TableManager
import rospy
from mirv_control.msg import pilit_db_msg, pilit_status_msg
from sensor_msgs.msg import NavSatFix
import time
import actionlib
import mirv_control.msg as msg

garage_table_cmd = """ CREATE TABLE IF NOT EXISTS garage (
    timestamp double PRIMARY KEY,
    latitude double NOT NULL,
    longitude double NOT NULL,
    altitude double NOT NULL
); """
garage_table_columns = (
    "timestamp",
    "latitude",
    "longitude",
    "altitude"
)
pilit_table_cmd = """ CREATE TABLE IF NOT EXISTS pilits (
    timestamp double PRIMARY KEY,
    right_count integer NOT NULL,
    left_count integer NOT NULL,
    p1_timestamp double NOT NULL,
    p1_state text NOT NULL,
    p1_lat double,
    p1_long double,
    p1_elev double,
    p2_timestamp double,
    p2_state text NOT NULL,
    p2_lat double,
    p2_long double,
    p2_elev double,
    p3_timestamp double,
    p3_state text NOT NULL,
    p3_lat double,
    p3_long double,
    p3_elev double,
    p4_timestamp double,
    p4_state text NOT NULL,
    p4_lat double,
    p4_long double,
    p4_elev double,
    p5_timestamp double,
    p5_state text NOT NULL,
    p5_lat double,
    p5_long double,
    p5_elev double,
    p6_timestamp double,
    p6_state text NOT NULL,
    p6_lat double,
    p6_long double,
    p6_elev double,
    p7_timestamp double,
    p7_state text NOT NULL,
    p7_lat double,
    p7_long double,
    p7_elev double,
    p8_timestamp double,
    p8_state text NOT NULL,
    p8_lat double,
    p8_long double,
    p8_elev double
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

def pilit_callback(pilit_event: pilit_db_msg):
    t = TableManager()
    t.connect(r"/mnt/SSD/mirv.db")
    if pilit_event.deploy_or_retrieve.data == "deploy":
        rospy.loginfo(f"Deployed pili at lat: {pilit_event.gps_pos.latitude} long: {pilit_event.gps_pos.longitude} alt: {pilit_event.gps_pos.altitude}t")
        t.deployed_pilit(pilit_event.gps_pos, pilit_event.side.data, pilit_table_columns)
    elif pilit_event.deploy_or_retrieve.data == "retrieve":
        rospy.loginfo(f"Retrieved pilit at lat: {pilit_event.gps_pos.latitude} long: {pilit_event.gps_pos.longitude} alt: {pilit_event.gps_pos.altitude}")
        t.retrieved_pilit(pilit_event.gps_pos, pilit_event.side.data, pilit_table_columns)
    t.close()

def garage_callback(gps_pos: NavSatFix):
    t = TableManager()
    t.connect(r"/mnt/SSD/mirv.db")
    rospy.loginfo(f"Recorded garage position at lat: {gps_pos.latitude} long: {gps_pos.longitude} alt: {gps_pos.altitude}")
    t.append_row("garage", garage_table_columns, (time.time(), gps_pos.latitude, gps_pos.longitude, gps_pos.altitude))
    t.close()

def query_callback():
    goal = action_server.accept_new_goal()
    feedback = msg.DatabaseFeedback()
    result = msg.DatabaseResult()
    t = TableManager()
    t.connect(r"/mnt/SSD/mirv.db")
    if goal == "pilits":
        data = t.get_last_row("pilits")
        result.latitude = data[5::5]
        result.latitude = [lat for lat in result.latitude if lat != None]
        result.longitude = data[6::5]
        result.longitude = [long for long in result.longitude if long != None]
        result.altitude = data[7::5]
        result.altitude = [alt for alt in result.altitude if alt != None]
    elif goal == "garage":
        data = t.get_last_row("garage")
        data = [0 for value in data if not value]
        result.latitude = data[1]
        result.longitude = data[2]
        result.altitude = data[3]
    t.close()
    action_server.set_succeeded(result)

def publish_pilit_info(time):
    t = TableManager()
    t.connect(r"/mnt/SSD/mirv.db")
    data = t.get_last_row("pilits")
    t.close()
    status = pilit_status_msg()
    if not data:
        return
    status.timestamp.data = data[0]
    status.right_count.data = data[1]
    status.left_count.data = data[2]
    status.latitudes.data = data[5::5]
    status.latitudes.data = [lat for lat in status.latitudes.data if lat != None]
    status.longitudes.data = data[6::5]
    status.longitudes.data = [long for long in status.longitudes.data if long != None]
    status.altitudes.data = data[7::5]
    status.altitudes.data = [alt for alt in status.altitudes.data if alt != None]
    pilit_pub.publish(status)

rospy.init_node("DatabaseController")
pilit_sub = rospy.Subscriber("pilit/events", pilit_db_msg, pilit_callback)
pilit_pub = rospy.Publisher("pilit/status", pilit_status_msg, queue_size = 10)
pilit_pub_timer = rospy.Timer(rospy.Duration(1), publish_pilit_info)

action_server = actionlib.SimpleActionServer("Database", msg.DatabaseAction, auto_start = False)
action_server.register_goal_callback(query_callback)
action_server.start()

tm = TableManager()
tm.connect(r"/mnt/SSD/mirv.db")
tm.create_table(pilit_table_cmd)
tm.create_table(garage_table_cmd)
tm.append_row("pilits", pilit_table_columns, (time.time(), 4, 4, 0, "stored", None, None, None, 0, "stored", None, None, None, 0, "stored", None, None, None, 0, "stored", None, None, None, 0, "stored", None, None, None, 0, "stored", None, None, None, 0, "stored", None, None, None, 0, "stored", None, None, None))
tm.close()

while not rospy.is_shutdown():
    time.sleep(0.1)