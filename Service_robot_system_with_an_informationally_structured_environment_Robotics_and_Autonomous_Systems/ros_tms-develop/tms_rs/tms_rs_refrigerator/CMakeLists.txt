cmake_minimum_required(VERSION 2.8.3)
project(tms_rs_refrigerator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  tms_msg_db
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES tms_rs_refrigerator
  CATKIN_DEPENDS rospy std_msgs tms_msg_db
  #DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)
