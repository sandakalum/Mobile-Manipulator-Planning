###############################
## cmake and project setting ##
###############################
cmake_minimum_required(VERSION 2.8.3)
project(tms_msg_ur)

find_package(
  catkin REQUIRED
  COMPONENTS std_msgs message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
)

add_service_files(
  FILES
  tms_rs_get_around_object.srv
  tms_rs_get_object_info_rt.srv
  tms_rs_renew.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS std_msgs
)



###########
## Build ##
###########
# none
