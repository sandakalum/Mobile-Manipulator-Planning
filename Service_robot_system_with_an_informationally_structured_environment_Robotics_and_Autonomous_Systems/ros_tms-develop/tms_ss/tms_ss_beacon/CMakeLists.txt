cmake_minimum_required(VERSION 2.8.3)
project(tms_ss_beacon)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tms_msg_ss
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES tms_ss_beacon
  CATKIN_DEPENDS rospy roscpp
  DEPENDS system_lib eigen
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)

add_executable(tms_ss_beacon src/main.cpp)

add_dependencies(tms_ss_beacon DEPENDS tms_msg_ss_gencpp)

target_link_libraries(tms_ss_beacon
   ${catkin_LIBRARIES}
)
