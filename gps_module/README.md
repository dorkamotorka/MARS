# GPS Neo M6 Module

# WARNING: Wiring for Neo M6 is different from Neo M7, double-check if the connections are correct (You should check on the actual GPS board, not from the marks on the wires!)

## Dependencies

Before running GPS we should install dependencies:

	sudo apt-get install gpsd gpsd-clients 
	pip install pynmea2

and some ROS dependencies:
	
	sudo apt install ros-kinetic-gps-common

## Run GPS Software

In order to retreive GPS data, run:

	rosrun gps_module m7_ros.py

and in the other terminal, type:

	rostopic echo /gps_data

You should see stream of GPS data.

## Verification of GPS Boards

### Neo-6M

In order to verify operation of the GPS Module:

1.) Wire the GPS <br>
2.) Wait until Green light start blinking (takes about 5 min and you need to be near the window or outside) <br>
3.) Type in the terminal: 

	stty -F /dev/ttyAMA0 115200 (If you will be getting trash data later this means baudrate is wrong)
	sudo gpsd /dev/ttyAMA0 -F /var/run/gpsd.sock
4.) In the same terminal display data with:

	cgps -s

### Neo-7M

In order to verify operation of the GPS Module:

1.) Wire the GPS
2.) Wait for a 1-2 minute (You need to be near the window or outside) 
3.) Type in the terminal:

	stty -F /dev/ttyAMA0 9600 (If you will be getting trash data later this means baudrate is wrong)
4.) In the same terminal display data with:

	sudo scree /dev/ttyAMA0 9600

NOTE: Lots of tutorials are mentioning to output data with:

	sudo cat /dev/ttyAMA0

Which was empty for both of my modules in my case. 

## Debug information

If you get error:

	[Errno 16] could not open port /dev/ttyAMA0: [Errno 16] Device or resource busy: '/dev/ttyAMA0'

Try calling:
	
	sudo lsof /dev/ttyAMA0

Which should tell which Process and its PID is using the port. You should kill it with:

	kill -9 <PID>

