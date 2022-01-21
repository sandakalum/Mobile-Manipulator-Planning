cmake_minimum_required(VERSION 2.8.3)
project(tms_rc_double)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tms_msg_db
  nav_msgs
  tf
)

add_service_files(
  FILES
  skype_srv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
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

add_executable(tms_ss_double_pozyx src/main.cpp)
add_dependencies(tms_ss_double_pozyx tms_msg_db_generate_messages_cpp tms_msg_ss_generate_messages_cpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(double_control
   ${catkin_LIBRARIES}
 )

  target_link_libraries(tms_ss_double_pozyx
   ${catkin_LIBRARIES}
 )

