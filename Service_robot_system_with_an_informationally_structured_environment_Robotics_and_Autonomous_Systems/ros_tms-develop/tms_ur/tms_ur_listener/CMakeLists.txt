cmake_minimum_required(VERSION 2.8.3)
project(tms_ur_listener)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  julius_msg.msg
)

add_service_files(
  FILES
  gSpeech_msg.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES tms_ur_listener
   CATKIN_DEPENDS roscpp rospy std_msgs message_generation
#  DEPENDS system_lib
)
