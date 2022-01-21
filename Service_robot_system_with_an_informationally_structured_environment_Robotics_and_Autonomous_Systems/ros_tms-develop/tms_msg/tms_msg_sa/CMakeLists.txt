###############################
## cmake and project setting ##
###############################
cmake_minimum_required(VERSION 2.8.3)
project(tms_msg_sa)

find_package(
  catkin REQUIRED 
  COMPONENTS std_msgs message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  bas_behavior_data.msg
)

add_service_files(
  FILES
  tms_utility_activate_service.srv
  tms_to_activate_service.srv
  tms_to_modify_missing_objects.srv
  rps_arm_drive.srv
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


