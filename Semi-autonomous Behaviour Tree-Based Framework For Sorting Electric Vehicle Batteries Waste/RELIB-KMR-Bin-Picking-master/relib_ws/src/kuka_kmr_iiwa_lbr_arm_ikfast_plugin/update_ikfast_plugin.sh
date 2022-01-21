search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=kuka_kmr_iiwa.srdf
robot_name_in_srdf=kuka_kmr_iiwa
moveit_config_pkg=kuka_kmr_iiwa_moveit_config
robot_name=kuka_kmr_iiwa
planning_group_name=lbr_arm
ikfast_plugin_pkg=kuka_kmr_iiwa_lbr_arm_ikfast_plugin
base_link_name=base_footprint
eef_link_name=link_7
ikfast_output_path=/home/hector/RELIB-KMR-Bin-Picking/relib_ws/src/kuka_kmr_iiwa_description/urdf/kuka_kmr_iiwa_lbr_arm_ikfast_plugin/src/kuka_kmr_iiwa_lbr_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
