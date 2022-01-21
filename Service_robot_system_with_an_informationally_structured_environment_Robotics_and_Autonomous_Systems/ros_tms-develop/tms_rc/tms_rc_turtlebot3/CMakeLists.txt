cmake_minimum_required(VERSION 2.8.3)
project(tms_rc_turtlebot3)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf
  tms_msg_db
  rospy
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tms_rc_double
  CATKIN_DEPENDS geometry_msgs roscpp tms_msg_db tms_msg_rc nav_msgs rospy
 # DEPENDS system_lib
  CATKIN_DEPENDS tf
)

#catkin_python_setup()

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
 #add_executable(turtlebot3_control src/double_control.cpp) # src/ClientSocket.cpp src/Socket.cpp)
 #add_dependencies(turtlebot3_control tms_msg_rc_gencpp tms_msg_db_gencpp)


 #add_executable(turtlebot3_virtual_control src/double_virtual_control.cpp) # src/ClientSocket.cpp src/Socket.cpp)
 #add_dependencies(turtlebot3_virtual_control tms_msg_rc_gencpp tms_msg_db_gencpp)

## Specify libraries to link a library or executable target against
 #target_link_libraries(turtlebot3_control
 #  ${catkin_LIBRARIES}
 #)

  #target_link_libraries(turtlebot3_virtual_control
   #${catkin_LIBRARIES}
 #)

