cmake_minimum_required(VERSION 2.8.3)
project(tms_ts_ts)

find_package(catkin REQUIRED COMPONENTS nodelet roscpp rospy std_msgs tms_msg_ts nodelet)
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES tms_ts_nodelet
   CATKIN_DEPENDS roscpp nodelet tms_msg_ts
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

 add_library(tms_ts_nodelet src/tms_ts_nodelet.cpp) 
 add_dependencies(tms_ts_nodelet tms_msg_ts_generate_messages_cpp tms_msg_db_generate_messages_cpp)
 
 add_executable(ts_master src/ts_master.cpp)
 add_dependencies(ts_master tms_msg_ts_generate_messages_cpp)
 
 target_link_libraries(ts_master ${catkin_LIBRARIES})

