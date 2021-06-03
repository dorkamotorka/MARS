cmake_minimum_required(VERSION 3.0.2)
project(gps_module)

find_package(catkin REQUIRED COMPONENTS
  gps_common
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gps_module
#  CATKIN_DEPENDS gps_common rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
