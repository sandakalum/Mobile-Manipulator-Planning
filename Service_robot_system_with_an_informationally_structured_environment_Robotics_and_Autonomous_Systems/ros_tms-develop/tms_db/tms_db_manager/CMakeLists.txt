cmake_minimum_required(VERSION 2.8.3)
project(tms_db_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tms_msg_db
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES tms_db_manager
  #DEPENDS system_lib
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs tms_msg_db
)

catkin_python_setup()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)
