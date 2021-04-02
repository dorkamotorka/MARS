#!/usr/bin/python

import sys
import serial
import pynmea2

import rospy
from nav_msgs.msg import Odometry

# Initialize
try:
    ser = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=0.1)
except serial.serialutil.SerialException as e:
    print(e)
    sys.exit(0)

rospy.init_node('gps_m7', anonymous=False)
gps = rospy.Publisher("/nav_odom", Odometry, queue_size=1)

try:
    while not rospy.is_shutdown():
        try:
            data = ser.readline()
            dataout = pynmea2.NMEAStreamReader()
            # print(data)

            if data[0:6] == "$GPRMC":
                msg = pynmea2.parse(data)
                print(dir(msg))
                lat = msg.latitude
                # print("Latitude: %s" % str(lat))
                lng = msg.longitude
                # print("Longitude: %s" % str(lng))

                msg = Odometry()
                msg.header.stamp = rospy.Time.now()
                # TODO: Check if in dec. degrees!
                # TODO: Need orientation to know in which direction to go!
                msg.pose.pose.position.x = lng  # dec. degrees!
                msg.pose.pose.position.y = lat  # dec. degrees!
                msg.pose.pose.position.z = 35.0  # meters!
                msg.pose.pose.orientation.x = 0
                msg.pose.pose.orientation.y = 0
                msg.pose.pose.orientation.z = 0
                msg.pose.pose.orientation.w = 1

                gps.publish(msg)

        except serial.serialutil.SerialException:
            print('No data')
except KeyboardInterrupt:
    sys.exit(0)
