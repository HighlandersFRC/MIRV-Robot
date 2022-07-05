#!/usr/bin/env python3
import serial
from GGAData import GGAData
from VTGData import VTGData
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64MultiArray
import rospy
import time

pub = rospy.Publisher("gps/fix", NavSatFix, queue_size = 4)
rosPubMsg = NavSatFix()
rospy.init_node('RTKModule', anonymous=True)
currentGGA = GGAData()
currentVTG = VTGData()
print("reading in data")
with serial.Serial('/dev/ttyUSB0', 115200, timeout=1) as ser:
    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode('utf-8')  # read a '\n' terminated line
            line = line.split(",")
        except:
            pass
        try:
            if (line[0] == "$GPVTG"):
                currentVTG.loadMessage(line)
            if (line[0] == "$GPGGA"):
                currentGGA.loadMessage(line)
                rosPubMsg.header.stamp = rospy.Time.now()
                rosPubMsg.header.frame_id = "global"
                rosPubMsg.latitude = currentGGA.getLatitude()
                rosPubMsg.longitude = currentGGA.getLongitude()
                rosPubMsg.altitude = currentGGA.getAltitude()
                rosPubMsg.status.status = currentGGA.getQualityIndicator()
                rosPubMsg.position_covariance = currentGGA.getCovariance()
                pub.publish(rosPubMsg)
                print(line)
        except KeyboardInterrupt:
            quit()
            break
        except:
            print("Failed to receive message, trying again in 1 second")
