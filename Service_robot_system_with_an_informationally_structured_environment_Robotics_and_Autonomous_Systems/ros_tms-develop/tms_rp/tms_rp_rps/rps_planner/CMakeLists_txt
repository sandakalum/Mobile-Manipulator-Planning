cmake_minimum_required(VERSION 2.8.3)
project(tms_rp_rps_planner)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#   INCLUDE_DIRS include
   LIBRARIES rps_planner
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
 add_executable(rps_goal_planner src/rps_goal_planner.cpp)
 add_executable(rps_path_planner src/rps_path_planner.cpp)
 add_executable(rps_voronoi_path_planner src/rps_voronoi_path_planner.cpp)
 add_executable(rps_push_wagon_path_planner src/rps_push_wagon_path_planner.cpp)

## Test for subtask give ################################################
 add_executable(rps_give src/rps_give.cpp)
#########################################################################

add_dependencies(rps_goal_planner DEPENDS tms_msg_rp_gencpp tms_msg_db_gencpp)
add_dependencies(rps_path_planner DEPENDS tms_msg_rp_gencpp tms_msg_db_gencpp)
add_dependencies(rps_voronoi_path_planner DEPENDS tms_msg_rp_gencpp tms_msg_db_gencpp)
add_dependencies(rps_push_wagon_path_planner DEPENDS tms_msg_rp_gencpp tms_msg_db_gencpp)

## Test for subtask give ################################################
 add_dependencies(rps_give DEPENDS tms_msg_rp_gencpp tms_msg_db_gencpp)
######################################################################### 


## Specify libraries to link a library or executable target against
 target_link_libraries(rps_goal_planner
   ${catkin_LIBRARIES}
 )
  target_link_libraries(rps_path_planner
   ${catkin_LIBRARIES}
 )
 target_link_libraries(rps_voronoi_path_planner
   ${catkin_LIBRARIES}
 )
 target_link_libraries(rps_push_wagon_path_planner
   ${catkin_LIBRARIES}
 )

## Test for subtask give ################################################
 target_link_libraries(rps_give
   ${catkin_LIBRARIES}
 )
#########################################################################