cmake_minimum_required(VERSION 2.8.3)
project(tms_sa_missing_objects)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

###########
## Build ##
###########
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tms_sa_missing_objects src/tms_sa_missing_objects.cpp)
target_link_libraries(tms_sa_missing_objects ${catkin_LIBRARIES})
add_dependencies(tms_sa_missing_objects DEPENDS tms_msg_ts_gencpp)

add_executable(tms_sa_missing_objects_rev2 src/tms_sa_missing_objects_rev2.cpp)
target_link_libraries(tms_sa_missing_objects_rev2 ${catkin_LIBRARIES})
add_dependencies(tms_sa_missing_objects_rev2 DEPENDS tms_msg_ts_gencpp)
