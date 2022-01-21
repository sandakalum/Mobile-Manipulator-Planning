###############################
## cmake and project setting ##
###############################
cmake_minimum_required(VERSION 2.8.3)
project(tms_msg_db)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  Tmsdb.msg
  TmsdbStamped.msg
  TmsData.msg
  TmsDataStamped.msg
)

add_service_files(
  FILES
  TmsdbGetData.srv
  tmsdb_get_furnitures_info.srv
  tmsdb_get_movable_furnitures_info.srv
  tmsdb_get_objects_info.srv
  tmsdb_get_person_info.srv
  tmsdb_get_robots_info.srv
  tmsdb_get_robot_info.srv
  tmsdb_get_person_behavior_info.srv
  tmsdb_get_pcd_info.srv
  tmsdb_get_task_list.srv
  tmsdb_missing_objects_checker.srv
  tmsdb_obj_dt.srv
  tmsdb_planning.srv
  tmsdb_file_conservation.srv
  tmsdb_objects_data.srv
  tmsdb_get_task_list.srv
  tmsdb_get_unknown_object.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES tms_msg_db
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs
  #  DEPENDS system_lib
)

###########
## Build ##
###########

# none
