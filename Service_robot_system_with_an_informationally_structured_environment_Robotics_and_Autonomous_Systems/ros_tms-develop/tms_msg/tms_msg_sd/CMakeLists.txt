###############################
## cmake and project setting ##
###############################
cmake_minimum_required(VERSION 2.8.3)
project(tms_msg_sd)

find_package(
  catkin REQUIRED 
  COMPONENTS std_msgs sensor_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################
# none

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs
)

###########
## Build ##
###########
# none
