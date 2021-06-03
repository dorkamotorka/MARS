cmake_minimum_required(VERSION 3.0.2)
project(mars_msg)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
   FILES
   custom.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

