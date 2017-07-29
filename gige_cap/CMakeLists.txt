cmake_minimum_required(VERSION 2.8.3)
project(gige_cap CXX)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++11 -Wno-unused-parameter")

find_package(
  catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  SetParam.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs message_runtime
  DEPENDS OpenCV
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(gige_cap
  src/gige_main.cpp
  src/gige_connect.cpp
  src/gige_capture.cpp
  src/gige_setparam.cpp
)
target_link_libraries(gige_cap
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  libflycapture.so
)

add_dependencies(gige_cap gige_cap_gencpp)

#############
## Install ##
#############


#############
## Testing ##
#############


