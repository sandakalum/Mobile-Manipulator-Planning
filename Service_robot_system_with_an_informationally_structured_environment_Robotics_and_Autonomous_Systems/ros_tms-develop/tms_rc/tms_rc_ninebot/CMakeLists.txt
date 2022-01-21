
cmake_minimum_required(VERSION 2.8.3)
project(tms_rc_ninebot)

find_package(catkin REQUIRED COMPONENTS
  tms_msg_rc
  roscpp
  rospy
  tf
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)
find_package(Boost REQUIRED COMPONENTS thread)

#add_message_files(
#  FILES
#  #odom_rad.msg
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
#ninebot_keyop
#This node is to operate ninebot using keyboard
#------------------------------------------------------------------------------------------------------------------
add_executable(ninebot_keyop src/ninebot_keyop.cpp)
add_dependencies(ninebot_keyop tms_msg_rc_generate_messages_cpp)
target_link_libraries(ninebot_keyop ${catkin_LIBRARIES})

#------------------------------------------------------------------------------------------------------------------
#ninebot_keyop2
#This node is to operate ninebot using keyboard
#------------------------------------------------------------------------------------------------------------------
add_executable(ninebot_keyop2 src/ninebot_keyop2.cpp)
add_dependencies(ninebot_keyop2 tms_msg_rc_generate_messages_cpp)
target_link_libraries(ninebot_keyop2 ${catkin_LIBRARIES})


#------------------------------------------------------------------------------------------------------------------
#ninebot_stater
#publish odom_rad
#------------------------------------------------------------------------------------------------------------------
#add_executable(ninebot_stater src/ninebot_stater.cpp)
#add_dependencies(ninebot_stater ninebot_generate_messages_cpp)
#target_link_libraries(ninebot_stater ${catkin_LIBRARIES})

#------------------------------------------------------------------------------------------------------------------
#ninebot_tf
#change serial data to odom
#------------------------------------------------------------------------------------------------------------------
#add_executable(ninebot_tf src/ninebot_tf.cpp)
#add_dependencies(ninebot_tf ninebot_generate_messages_cpp)
#target_link_libraries(ninebot_tf ${catkin_LIBRARIES})



#------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------
#use in order to debug
#------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------

#------------------------------------------------------------------------------------------------------------------
#virtual_ninebot_stater
#publish virtual_odom_rad
#------------------------------------------------------------------------------------------------------------------
#add_executable(ninebot_virtual_stater src/ninebot_virtual_stater.cpp)
#add_dependencies(ninebot_virtual_stater ninebot_generate_messages_cpp)
#target_link_libraries(ninebot_virtual_stater ${catkin_LIBRARIES})

#------------------------------------------------------------------------------------------------------------------
#virtual_ninebot_laser
#publish virtual_scan
#------------------------------------------------------------------------------------------------------------------
#add_executable(ninebot_virtual_laser src/ninebot_virtual_laser.cpp)
#add_dependencies(ninebot_virtual_laser ninebot_generate_messages_cpp)
#target_link_libraries(ninebot_virtual_laser ${catkin_LIBRARIES})

#------------------------------------------------------------------------------------------------------------------
#virtual_ninebot_laser2
#publish virtual_scan
#------------------------------------------------------------------------------------------------------------------
#add_executable(ninebot_virtual_laser2 src/ninebot_virtual_laser2.cpp)
#add_dependencies(ninebot_virtual_laser2 ninebot_generate_messages_cpp)
#target_link_libraries(ninebot_virtual_laser2 ${catkin_LIBRARIES})
