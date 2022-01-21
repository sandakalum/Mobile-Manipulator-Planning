###############################
## cmake and project setting ##
###############################
cmake_minimum_required(VERSION 2.8.3)
project(tms_msg_ss)

find_package(
  catkin REQUIRED
  COMPONENTS std_msgs sensor_msgs geometry_msgs message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  vicon_data.msg
  accumulate_data.msg
  # fss_class_data.msg
  # fss_cluster_data.msg
  # fss_detected_cluster_data.msg
  # fss_object_data.msg
  # fss_observed_datas.msg
  # fss_person_trajectory_data.msg
  # fss_pre_data.msg
  # fss_tf_data.msg
  # fss_tf_datas.msg
  bas_behavior_data.msg
  # ics_object_data.msg
  # iss_object_data.msg
  lrf_analyze_data.msg
  lrf_cluster.msg
  lrf_clusters_data.msg
  lrf_datas.msg
  lrf_human_data.msg
  lrf_trajectory_data.msg
  ods_person_dt.msg
  Robot.msg
  Furniture.msg
  tracking_points.msg
  tracking_grid.msg
  pot_tracking_points.msg
  pot_tracking_grid.msg
  pot_tracking_psen.msg
  pot_tracking_psens.msg
  pot_manager.msg
  pot_clustering.msg
  tracking_cluster.msg
  tracking_clusters_data.msg

  Sensor.msg

  Skeleton.msg
  SkeletonArray.msg
  CameraPosture.msg
  SkeletonStreamWrapper.msg
  Beacon.msg
)

add_service_files(
  FILES
  ods_cylinder.srv
  ods_face_detection.srv
  ods_change_detection.srv
  ods_furniture.srv
  ods_pcd.srv
  ods_get_robots_pos.srv
  ods_skincolor_extraction.srv
  ods_wagon.srv
  ods_wagon_object.srv
  ods_person_detection.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  tms_msg_db
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs
)

###########
## Build ##
###########
# none
