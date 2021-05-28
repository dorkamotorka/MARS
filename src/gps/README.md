# GPS Neo M6 Module

# WARNING: Wiring for Neo M6 is different from Neo M7, double-check if the connections are correct (You should check on the actual GPS board, not from the marks on the wires!)

## Dependencies

Before running GPS you type in the terminal:

	 sudo apt-get install gpsd gpsd-clients

## Verification

In order to verify operation of the GPS Module:

1.) Wire the GPS
2.) Wait until Green light start blinking (takes about 5 min and you need to be near the window or outside) 
3.) Type in the terminal:

	stty -F /dev/ttyAMA0 115200 (If you will be getting trash data later this means baudrate is wrong)
	sudo gpsd /dev/ttyAMA0 -F /var/run/gpsd.sock
4.) In the same terminal display data with:

	cgps -s
