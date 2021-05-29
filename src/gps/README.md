# GPS Neo M6 Module

# WARNING: Wiring for Neo M6 is different from Neo M7, double-check if the connections are correct (You should check on the actual GPS board, not from the marks on the wires!)

## Dependencies

Before running GPS you type in the terminal:

	sudo apt-get install gpsd gpsd-clients 
	pip install pynmea2

## Verification of Neo-6M

In order to verify operation of the GPS Module:

1.) Wire the GPS <br>
2.) Wait until Green light start blinking (takes about 5 min and you need to be near the window or outside) <br>
3.) Type in the terminal: 

	stty -F /dev/ttyAMA0 115200 (If you will be getting trash data later this means baudrate is wrong)
	sudo gpsd /dev/ttyAMA0 -F /var/run/gpsd.sock
4.) In the same terminal display data with:

	cgps -s

## Verification of Neo-7M

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
