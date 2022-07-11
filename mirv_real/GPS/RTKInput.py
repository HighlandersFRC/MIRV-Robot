#!/usr/bin/env python3
import serial
from GGAData import GGAData
from VTGData import VTGData
from std_msgs.msg import Float64MultiArray
import rospy
import time

pub = rospy.Publisher("GPSCoordinates", Float64MultiArray, queue_size=4)
rosPubMsg = Float64MultiArray()
rospy.init_node('RTKModule', anonymous=True)
currentGGA = GGAData()
currentVTG = VTGData()

with serial.Serial('/dev/ttyACM9', 115200, timeout=1) as ser:
    while True:
        try:
            line = ser.readline().decode('utf-8')  # read a '\n' terminated line
            line = line.split(",")
        except:
            pass
        try:
            if (line[0] == "$GNVTG"):
                currentVTG.loadMessage(line)
            if (line[0] == "$GNGGA"):
                currentGGA.loadMessage(line)
                rosPubMsg.data = [
                    currentGGA.getLatitude(), currentGGA.getLongitude()]
                pub.publish(rosPubMsg)
                print(line)
        except KeyboardInterrupt:
            break
        except:
            print("Failed to receive message, trying again in 1 second")
