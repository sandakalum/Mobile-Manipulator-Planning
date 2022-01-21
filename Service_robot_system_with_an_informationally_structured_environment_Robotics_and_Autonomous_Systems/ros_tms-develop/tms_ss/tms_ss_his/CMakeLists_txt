cmake_minimum_required(VERSION 2.8.3)
project(tms_ss_his)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES tms_ss_his
  CATKIN_DEPENDS roscpp sensor_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rcv9axis src/rcv9axis.cpp)
add_executable(get_person_position src/get_person_position.cpp)

add_dependencies(rcv9axis DEPENDS tms_ss_fss_exe_gencpp tms_msg_ss_gencpp)
add_dependencies(get_person_position DEPENDS tms_ss_fss_exe_gencpp tms_msg_ss_gencpp)

target_link_libraries(rcv9axis ${catkin_LIBRARIES})
target_link_libraries(get_person_position ${catkin_LIBRARIES})

