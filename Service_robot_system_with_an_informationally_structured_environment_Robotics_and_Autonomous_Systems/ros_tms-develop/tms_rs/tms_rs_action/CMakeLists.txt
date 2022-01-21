cmake_minimum_required(VERSION 2.8.3)
project(tms_rs_action)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tms_msg_rs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   #INCLUDE_DIRS include
   #LIBRARIES smartpal_simple_task
   CATKIN_DEPENDS roscpp geometry_msgs nav_msgs message_runtime
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

add_executable(smartpal_simple_task

    src/tms_rs_simple_task.cpp
)

add_dependencies(smartpal_simple_task tms_msg_rs_generate_messages_cpp)

#add_dependencies(tms_msg_rs_gencpp tms_msg_rp_gencpp tms_msg_db_gencpp)

target_link_libraries(smartpal_simple_task ${catkin_LIBRARIES})

add_executable(simpletask_service

    src/simpletask_service.cpp
)

add_dependencies(simpletask_service tms_msg_rs_generate_messages_cpp)

#add_dependencies(tms_msg_rs_gencpp tms_msg_rp_gencpp tms_msg_db_gencpp)

target_link_libraries(simpletask_service ${catkin_LIBRARIES})

#add_executable(smartpal_rp_move

 #   src/tms_rs_rpmove.cpp
#)

#add_dependencies(smartpal_rp_move DEPENDS tms_msg_rc_gencpp tms_msg_rp_gencpp tms_msg_db_gencpp)

#target_link_libraries(smartpal_rp_move ${catkin_LIBRARIES})
