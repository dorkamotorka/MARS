cmake_minimum_required(VERSION 3.0.2)
project(imega_arduino)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rosserial_arduino
  rosserial_client
  sensor_msgs
  std_msgs
)

catkin_package()

# Arduino specific configuration
rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware hello ALL)
rosserial_add_client_target(firmware hello-upload)
