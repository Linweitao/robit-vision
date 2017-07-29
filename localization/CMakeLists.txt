cmake_minimum_required(VERSION 2.8.3)
project(localization)

find_package(
  catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  gige_cap
  message_generation
)

find_package(OpenCV)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  Location.msg
)
add_message_files(
  FILES
  Samples.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
##################################
catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs gige_cap message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(localizer 
  src/main.cpp  
  src/localizer.cpp
)
add_executable(scanline
  src/scanlinemain.cpp
  src/sign.cpp
)
target_link_libraries(localizer 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
)
target_link_libraries(scanline
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(localizer gige_cap_gencpp localization_generate_messages_cpp)
add_dependencies(scanline gige_cap_gencpp localization_generate_messages_cpp)


#############
## Install ##
#############


#############
## Testing ##
#############

