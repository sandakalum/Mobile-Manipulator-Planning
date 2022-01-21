###############################
## cmake and project setting ##
###############################
cmake_minimum_required(VERSION 2.8.3)
project(tms_msg_rp)

find_package(
  catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  pos.msg
  rps_cnoid_wagon_motion_state.msg
  rps_command.msg
  rps_command_array.msg
  rps_joint_angle.msg
  rps_joint_angle_route.msg
  rps_map_data.msg
  rps_map_full.msg
  rps_map_y.msg
  rps_path_alarm.msg
  rps_pos_param.msg
  rps_position.msg
  rps_pushing_wagon_route.msg
  rps_route.msg
)

add_service_files(
  FILES
  action.srv
  rp_cmd.srv
  rp_action.srv
  rp_place.srv
  rp_pick.srv
  rp_release.srv
  rp_move.srv
  rp_take.srv
  rp_arrow.srv
  rps_IK_arm_planning.srv
  rps_cnoid_PRM_planning.srv
  rps_cnoid_grasp_obj_planning.srv
  rps_cnoid_grasp_wagon_planning.srv
  rps_arm_drive.srv
  rps_command_to_move_smartpal.srv
  rps_robot_drive.srv
  rps_give_obj_command.srv
  rps_goal_planning.srv
  rps_path_checking.srv
  rps_path_planning.srv
  rps_push_wagon_path_planning.srv
  rps_robot_command.srv
  rps_select_path.srv
  rps_set_human_arm_angle.srv
  rps_voronoi_path_planning.srv
  #temp tms_msg_rp
  tms_ts_find_objects.srv
  tms_rs_get_object_info_ex.srv
  get_object.srv
  #temp tms_msg_rp
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
