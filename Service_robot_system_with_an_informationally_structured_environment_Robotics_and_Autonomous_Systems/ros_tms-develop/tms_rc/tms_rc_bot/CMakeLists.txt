
cmake_minimum_required(VERSION 2.8.3)
project(tms_rc_bot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  
  ## ublox_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)

#add_message_files(
#  FILES
#  odom_rad.msg
#  nucleo_serial.msg
#)

#add_service_files(
#  FILES
#  nav_command.srv
#)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
)

#------------------------------------------------------------------------------------------------------------------
#chairbot_driver
#------------------------------------------------------------------------------------------------------------------
add_executable(chairbot_driver src/ChairbotDriver.cpp src/ClientSocket.cpp src/Socket.cpp)
target_link_libraries(chairbot_driver ${catkin_LIBRARIES})