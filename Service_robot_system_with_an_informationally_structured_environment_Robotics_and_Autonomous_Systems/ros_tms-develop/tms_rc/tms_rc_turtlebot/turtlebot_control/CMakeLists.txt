cmake_minimum_required(VERSION 2.8.3)
project(tms_rc_turtlebot_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tms_msg_rc
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES kobuki_control
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(turtlebot_control src/turtlebot_control.cpp)
add_dependencies(turtlebot_control tms_msg_rc_generate_messages_cpp tms_msg_ss_generate_messages_cpp tms_msg_db_generate_messages_cpp)
target_link_libraries(turtlebot_control ${catkin_LIBRARIES} )
