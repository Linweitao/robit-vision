cmake_minimum_required(VERSION 2.8.3)
project(image_processor)

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
  Ball_Location.msg
)
add_message_files(
  FILES
  Obstacle_Location.msg
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

add_executable(image_processor
  src/main.cpp
  src/msg_transport.cpp
  src/Thresholder.cpp
)

add_executable(Thresholder
  src/msg_transport.cpp
  src/Thresholder_Main.cpp
  src/Thresholder.cpp
)

add_executable(threshold
  src/msg_transport.cpp
  src/image_process_main.cpp
  src/Thresholder.cpp
)

add_executable(listenner
  src/listenner.cpp
  src/msg_transport.cpp
  src/Thresholder.cpp
)
add_executable(talker
  src/talker.cpp
  src/msg_transport.cpp
  src/Thresholder.cpp
)
target_link_libraries(image_processor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(Thresholder
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(threshold
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(listenner
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(talker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(image_processor gige_cap_gencpp image_processor_gencpp)
add_dependencies(Thresholder gige_cap_gencpp image_processor_gencpp)
add_dependencies(threshold gige_cap_gencpp image_processor_gencpp)
add_dependencies(listenner gige_cap_gencpp image_processor_gencpp)
add_dependencies(talker gige_cap_gencpp image_processor_gencpp)
#############
## Install ##
#############


#############
## Testing ##
#############

