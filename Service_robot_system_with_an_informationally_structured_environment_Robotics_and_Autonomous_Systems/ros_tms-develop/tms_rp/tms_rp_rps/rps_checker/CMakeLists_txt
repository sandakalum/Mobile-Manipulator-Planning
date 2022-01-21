cmake_minimum_required(VERSION 2.8.3)
project(tms_rp_rps_checker)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#   INCLUDE_DIRS include
   LIBRARIES tms_rp_rps_checker
   CATKIN_DEPENDS roscpp std_msgs
   DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  /usr/include/qt4/
  /usr/include/qt4/QtCore
  /usr/include/qt4/QtGui
)


## Declare a cpp executable
 #~ add_executable(rps_path_checker src/rps_path_checker.cpp)
 add_executable(rps_map_publisher src/rps_map_publisher.cpp)
 #~ add_executable(rps_map_publisher1 src/rps_map_publisher1.cpp)
 add_executable(rps_map_checker src/rps_map_checker.cpp)
 #~ add_executable(rps_alarm_checker src/rps_alarm_checker.cpp)

## Specify libraries to link a library or executable target against
 #~ target_link_libraries(rps_path_checker
   #~ ${catkin_LIBRARIES}
 #~ )
 target_link_libraries(rps_map_publisher
   ${catkin_LIBRARIES}
 )
 #~ target_link_libraries(rps_map_publisher1
   #~ ${catkin_LIBRARIES}
 #~ )
 target_link_libraries(rps_map_checker
   ${catkin_LIBRARIES}
 )
 #~ target_link_libraries(rps_alarm_checker
   #~ ${catkin_LIBRARIES}
 #~ )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(rps_map_publisher DEPENDS tms_msg_db_gencpp tms_msg_rp_gencpp tms_msg_rc_gencpp)
add_dependencies(rps_map_checker DEPENDS tms_msg_db_gencpp tms_msg_rp_gencpp tms_msg_rc_gencpp)
 #~ add_dependencies(rps_path_checker rps_checker_generate_messages_cpp)
 #~ add_dependencies(rps_alarm_checker rps_checker_generate_messages_cpp)

