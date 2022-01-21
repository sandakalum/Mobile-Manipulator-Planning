###############################
## cmake and project setting ##
###############################
cmake_minimum_required(VERSION 2.8.3)
project(tms_msg_rs)

find_package(
  catkin REQUIRED COMPONENTS
  genmsg
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  robot_task.msg
)

add_service_files(
  FILES
  rs_task.srv
  rs_home_appliances.srv
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
