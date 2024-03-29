cmake_minimum_required(VERSION 2.8.3)
project(mars_detect)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  geometry_msgs
  tf
  roscpp
  tf2_geometry_msgs
  tf2_ros
  tf2
  visualization_msgs
  vision_msgs
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
  fiducial_msgs
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

generate_dynamic_reconfigure_options(cfg/DetectorParams.cfg)

catkin_package(
  INCLUDE_DIRS include src/stag
  LIBRARIES stag_ros
  CATKIN_DEPENDS camera_info_manager cv_bridge geometry_msgs image_transport roscpp sensor_msgs std_msgs
)

###########
## Build ##
###########

add_definitions(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Runnables
add_library(stag_core
  src/stag/Decoder.cpp
  src/stag/Drawer.cpp
  src/stag/EDInterface.cpp
  src/stag/Ellipse.cpp
  src/stag/Marker.cpp
  src/stag/PoseRefiner.cpp
  src/stag/Quad.cpp
  src/stag/QuadDetector.cpp
  src/stag/Stag.cpp
  src/stag/utility.cpp
  src/stag/ED/ED.cpp
  src/stag/ED/EDInternals.cpp
  src/stag/ED/EDLines.cpp
  src/stag/ED/GradientOperators.cpp
  src/stag/ED/ImageSmooth.cpp
  src/stag/ED/LineSegment.cpp
  src/stag/ED/MyMath.cpp
  src/stag/ED/NFA.cpp
# src/stag/ED/Utilities.cpp
  src/stag/ED/ValidateEdgeSegments.cpp
  include/stag_ros/load_yaml_tags.h
)

add_executable(mars_detect src/stag_ros/stag_detect.cpp)
add_dependencies(mars_detect ${catkin_EXPORTED_TARGETS})
target_link_libraries(stag_core ${OpenCV_LIBS})
target_link_libraries(mars_detect ${catkin_LIBRARIES} ${OpenCV_LIBS} stag_core)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS mars_detect
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY cfg/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
)

install(TARGETS
  mars_detect
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
