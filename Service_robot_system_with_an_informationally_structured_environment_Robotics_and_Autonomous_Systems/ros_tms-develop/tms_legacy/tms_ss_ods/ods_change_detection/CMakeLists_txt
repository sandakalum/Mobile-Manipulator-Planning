cmake_minimum_required(VERSION 2.8.3)
project(tms_ss_ods_change_detection)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
 add_executable(ods_change_detection src/ods_change_detection.cpp)
 add_executable(ods_changedt_table src/ods_changedt_table.cpp)
 add_executable(ods_changedt_shelf src/ods_changedt_shelf.cpp)
 add_executable(ods_changedt_chair src/ods_changedt_chair.cpp)
 add_executable(ods_changedt_bed src/ods_changedt_bed.cpp)

 target_link_libraries(ods_change_detection
   ${catkin_LIBRARIES}
 )
 target_link_libraries(ods_changedt_table
   ${catkin_LIBRARIES}
 )
 target_link_libraries(ods_changedt_shelf
   ${catkin_LIBRARIES}
 )
 target_link_libraries(ods_changedt_chair
   ${catkin_LIBRARIES}
 )
 target_link_libraries(ods_changedt_bed
   ${catkin_LIBRARIES}
 )

add_dependencies(ods_change_detection DEPENDS tms_msg_ss_gencpp tms_msg_rc_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
add_dependencies(ods_changedt_table DEPENDS tms_msg_ss_gencpp tms_msg_rc_gencpp tms_msg_db_gencpp)
add_dependencies(ods_changedt_shelf DEPENDS tms_msg_ss_gencpp)
add_dependencies(ods_changedt_chair DEPENDS tms_msg_ss_gencpp)
add_dependencies(ods_changedt_bed DEPENDS tms_msg_ss_gencpp)
