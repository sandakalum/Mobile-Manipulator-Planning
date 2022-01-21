cmake_minimum_required(VERSION 2.8.3)
project(tms_rp_voronoi_map)

find_package(catkin REQUIRED
  roscpp
  roslib
  std_msgs
  visualization_msgs
  tms_msg_db
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################
# We use the tms_msg_xxx packages for messages, services

###################################
## catkin specific configuration ##
###################################
catkin_package(
# INCLUDE_DIRS include
  LIBRARIES tms_rp_voronoi_map
  CATKIN_DEPENDS roscpp std_msgs visualization_msgs tms_msg_db tms_msg_rp
  #DEPENDS system_lib eigen
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)

add_executable(tms_rp_voronoi_map src/voronoi_map.cpp)
add_dependencies(tms_rp_voronoi_map tms_msg_db_generate_messages_cpp tms_msg_rp_generate_messages_cpp)
target_link_libraries(tms_rp_voronoi_map ${catkin_LIBRARIES})
