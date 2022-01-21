cmake_minimum_required(VERSION 2.8.3)
project(tms_rc_kobuki_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kobuki_msgs
  roscpp
  std_msgs
  tms_msg_rc
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES kobuki_control
#  CATKIN_DEPENDS geometry_msgs kobuki_msgs roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
 include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(kobuki_control src/kobuki_control.cpp)
add_dependencies(kobuki_control tms_msg_rc_generate_messages_cpp tms_msg_ss_generate_messages_cpp tms_msg_db_generate_messages_cpp)
target_link_libraries(kobuki_control ${catkin_LIBRARIES} )
