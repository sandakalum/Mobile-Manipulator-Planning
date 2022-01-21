###############################
## cmake and project setting ##
###############################
cmake_minimum_required(VERSION 2.8.3)
project(tms_msg_rc)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  robot_current_data.msg
  tag_data.msg
  odom_rad.msg
  katana_pos.msg
  arduino_serial.msg 
  nucleo_serial.msg 
)

add_service_files(
  FILES
  rc_robot_control.srv
  move_kobuki.srv
  kobuki_control_1.srv
  smartpal_control.srv
  smartpal_speak.srv
  robot_tts.srv
  tms_rc_ppose.srv
  tms_rc_pmove.srv
  tms_rc_pparam.srv
  katana_pos_array.srv
  katana_pos_single.srv
  gripper_action.srv
  robot_control.srv
  nav_command.srv
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
