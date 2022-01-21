cmake_minimum_required(VERSION 2.8.3)
project(tms_rp_rps_commander)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#   INCLUDE_DIRS include
   LIBRARIES tms_rp_rps_commander
   CATKIN_DEPENDS roscpp std_msgs
   DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(rps_map_checker
#   src/${PROJECT_NAME}/rps_map_checker.cpp
# )

#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../src/rps/rps_map_checker)

## Declare a cpp executable
 #~ add_executable(rps_test_planner src/rps_test_planner.cpp)
 add_executable(rps_test_commander src/rps_test_commander.cpp)
 add_executable(rps_give_obj_planner src/rps_give_obj_planner.cpp)
 add_executable(rps_give_obj_commander src/rps_give_obj_commander.cpp)
 add_executable(rps_robot_commander src/rps_robot_commander.cpp)
 #~ add_executable(rps_path_converter src/rps_path_converter.cpp)
 #~ add_executable(rps_path_converter_straight_turn src/rps_path_converter_straight_turn.cpp)
 #~ add_executable(rps_path_relay src/rps_path_relay.cpp)

add_dependencies(rps_test_commander DEPENDS tms_msg_rp_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp)
add_dependencies(rps_give_obj_planner DEPENDS tms_msg_rp_gencpp tms_msg_db_gencpp)
add_dependencies(rps_give_obj_commander DEPENDS tms_msg_rp_gencpp tms_msg_ss_gencpp)
add_dependencies(rps_robot_commander DEPENDS tms_msg_rp_gencpp tms_msg_rc_gencpp tms_msg_db_gencpp)
 #~ target_link_libraries(
  #~ rps_test_planner
  #~ ${catkin_LIBRARIES}
 #~ )
 target_link_libraries(
  rps_test_commander
  ${catkin_LIBRARIES}
 )
 target_link_libraries(
  rps_give_obj_planner
  ${catkin_LIBRARIES}
 )
 target_link_libraries(
  rps_give_obj_commander
  ${catkin_LIBRARIES}
 )
 target_link_libraries(
  rps_robot_commander
  ${catkin_LIBRARIES}
 )
 #~ target_link_libraries(
  #~ rps_path_converter
  #~ ${catkin_LIBRARIES}
 #~ )
 #~ target_link_libraries(
  #~ rps_path_converter_straight_turn
  #~ ${catkin_LIBRARIES}
 #~ )
 #~ target_link_libraries(
  #~ rps_path_relay
  #~ ${catkin_LIBRARIES}
 #~ )
