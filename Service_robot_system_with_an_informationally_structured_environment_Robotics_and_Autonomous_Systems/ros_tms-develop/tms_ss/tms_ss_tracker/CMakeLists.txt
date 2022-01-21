cmake_minimum_required(VERSION 2.8.3)
project(tms_ss_tracker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  tms_msg_ss
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tms_ss_tracker
 # DEPENDS system_lib eigen opencv
  CATKIN_DEPENDS roscpp message_runtime tms_msg_ss
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
)


add_executable(tracker src/tracker.cpp src/target.cpp src/laser.cpp src/multiple_particle_filter.cpp src/particle_filter.cpp include/condens/condens.cpp)
add_dependencies(tracker tms_msg_ss_gencpp)
target_link_libraries(tracker ${catkin_LIBRARIES})

add_executable(umo_tracker src/umo_tracker.cpp)
add_dependencies(umo_tracker tms_msg_ss_gencpp)
target_link_libraries(umo_tracker ${catkin_LIBRARIES})
