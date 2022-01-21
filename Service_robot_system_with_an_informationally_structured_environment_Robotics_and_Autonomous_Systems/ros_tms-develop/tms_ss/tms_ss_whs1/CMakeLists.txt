cmake_minimum_required(VERSION 2.8.3)
project(tms_ss_whs1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tms_msg_db
  tms_msg_ss
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tms_ss_whs1
#  CATKIN_DEPENDS roscpp tms_msg_db tms_msg_ss
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tms_ss_whs1 src/main.cpp)

add_dependencies(tms_ss_whs1 tms_msg_db_generate_messages_cpp)

target_link_libraries(tms_ss_whs1
   ${catkin_LIBRARIES}
)
