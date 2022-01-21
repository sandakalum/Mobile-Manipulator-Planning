cmake_minimum_required(VERSION 2.8.3)
project(tms_db_state_publisher)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  std_msgs
  sensor_msgs
  tms_msg_db
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tms_db_state_publisher
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs tms_msg_db
  #DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tms_db_state_publisher src/tms_db_state_publisher.cpp)
add_dependencies(tms_db_state_publisher tms_msg_db_generate_messages_cpp)
target_link_libraries(tms_db_state_publisher ${catkin_LIBRARIES})
