cmake_minimum_required(VERSION 3.0.2)
project(mars_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  topic_tools
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mars_nav
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
