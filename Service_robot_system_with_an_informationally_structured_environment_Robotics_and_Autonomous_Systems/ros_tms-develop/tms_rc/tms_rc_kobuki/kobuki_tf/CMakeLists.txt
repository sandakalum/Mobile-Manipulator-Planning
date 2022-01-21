cmake_minimum_required(VERSION 2.8.3)
project(kobuki_tf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kobuki_lrf_tf
#  CATKIN_DEPENDS geometry_msgs roscpp tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(kobuki_tf src/tf_broadcaster.cpp)
target_link_libraries(kobuki_tf ${catkin_LIBRARIES})
