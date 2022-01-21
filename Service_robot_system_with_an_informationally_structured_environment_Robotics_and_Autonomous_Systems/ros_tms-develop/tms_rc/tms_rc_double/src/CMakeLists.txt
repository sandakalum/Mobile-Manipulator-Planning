cmake_minimum_required(VERSION 2.8.3)
project(tms_rc_double)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tms_msg_db
  nav_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tms_rc_double
  CATKIN_DEPENDS geometry_msgs roscpp tms_msg_db tms_msg_rc nav_msgs
 # DEPENDS system_lib
  CATKIN_DEPENDS tf
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
 add_executable(double_control src/double_control.cpp) # src/ClientSocket.cpp src/Socket.cpp)
 add_dependencies(double_control tms_msg_rc_gencpp tms_msg_db_gencpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(double_control
   ${catkin_LIBRARIES}
 )

