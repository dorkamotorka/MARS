Steps to get it running:

1. Compile code with:

        catkin_make
        
2. Upload code to the connected Arduino board with:

        catkin_make imega_arduino_firmware_hello-upload

3. Run program with:

        roscore

    and in the other terminal:

        rosrun rosserial_python serial_node.py /dev/ttyACM0

4. Visualize data in other terminal with:

        rostopic echo /chatter
        
        
        
Compile time issue, solve with:

        sudo find . -exec touch {} \;
