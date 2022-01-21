cmake_minimum_required(VERSION 2.8.3)
project(tms_ss_ods_color_segmentation)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
 add_executable(ods_color_segmentation src/ods_color_segmentation.cpp)

 target_link_libraries(ods_color_segmentation
   ${catkin_LIBRARIES}
 )

 add_dependencies(ods_color_segmentation DEPENDS tms_msg_ss_gencpp)
