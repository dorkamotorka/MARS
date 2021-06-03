#!/usr/bin/python

import sys
import serial
import pynmea2

import rospy
from gps_common.msg import GPSFix

# Initialize
try:
	ser = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=0.1)
except serial.serialutil.SerialException as e:
	print(e)
	sys.exit(0)

rospy.init_node('gps_m7', anonymous=False)
gps = rospy.Publisher("/gps_data", GPSFix, queue_size=1)

try:
	while not rospy.is_shutdown(): 
		try:
			data = ser.readline()
			dataout = pynmea2.NMEAStreamReader()
			# print(data)

			if data[0:6] == "$GPRMC":
				msg = pynmea2.parse(data)
				# print(dir(msg))
				lat = msg.latitude
				# print("Latitude: %s" % str(lat))
				lng = msg.longitude
				# print("Longitude: %s" % str(lng))

				msg = GPSFix()
				msg.header.stamp = rospy.Time.now()
				msg.header.frame_id = "map" # Global frame

				msg.latitude = lat
				msg.longitude = lng

				gps.publish(msg)
		
		except serial.serialutil.SerialException:
			print('No data')
except KeyboardInterrupt:
	sys.exit(0)
