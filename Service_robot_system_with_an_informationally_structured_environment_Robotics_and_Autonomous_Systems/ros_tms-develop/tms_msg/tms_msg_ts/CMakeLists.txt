###############################
## cmake and project setting ##
###############################
cmake_minimum_required(VERSION 2.8.3)
project(tms_msg_ts)

find_package(
  catkin REQUIRED 
  COMPONENTS std_msgs sensor_msgs message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  arg.msg
)

add_service_files(
  FILES
  tms_sa_find_objects.srv
  ts_req.srv
  ts_state_control.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  tms_msg_db
)

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

