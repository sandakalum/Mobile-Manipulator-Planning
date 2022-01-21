cmake_minimum_required(VERSION 2.8.3)
project(tms_sa_bas_person)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)

#######################################
## Declare ROS messages and services ##
#######################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES tms_sa_bas_person
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bas_person src/bas_person.cpp include/tms_sa_bas_person/HumanModel.cpp)
add_dependencies(bas_person DEPENDS tms_msg_ss_gencpp tms_msg_db_gencpp)
target_link_libraries(bas_person ${catkin_LIBRARIES})

