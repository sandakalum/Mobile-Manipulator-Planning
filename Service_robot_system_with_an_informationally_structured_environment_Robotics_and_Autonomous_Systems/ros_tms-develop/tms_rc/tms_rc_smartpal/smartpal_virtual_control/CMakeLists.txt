cmake_minimum_required(VERSION 2.8.3)
project(tms_rc_smartpal_virtual_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  geometry_msgs
  nav_msgs
  tms_msg_rc
)
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   #INCLUDE_DIRS include
   #LIBRARIES smartpal_control
   CATKIN_DEPENDS roscpp tf geometry_msgs nav_msgs message_runtime
   #DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_executable(smartpal_virtual_control

    src/smartpal5_virtual_control.cpp
)

 add_dependencies(smartpal_virtual_control tms_msg_rc_generate_messages_cpp)
#add_dependencies(tms_msg_rc_gencpp tms_msg_db_gencpp)

 target_link_libraries(smartpal_virtual_control ${catkin_LIBRARIES})
