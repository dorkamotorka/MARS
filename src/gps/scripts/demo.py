import sys
import serial
import pynmea2

ser = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=0.1)

try:
	while True: 
		try:
			data = ser.readline()
			dataout = pynmea2.NMEAStreamReader()
			# print(data)

			if data[0:6] == "$GPRMC":
				msg = pynmea2.parse(data)
				# print(dir(msg))
				lat = msg.latitude
				print("Latitude: %s" % str(lat))
				lng = msg.longitude
				print("Longitude: %s" % str(lng))
		
		except serial.serialutil.SerialException:
			print('No data')
except KeyboardInterrupt:
	sys.exit(0)
