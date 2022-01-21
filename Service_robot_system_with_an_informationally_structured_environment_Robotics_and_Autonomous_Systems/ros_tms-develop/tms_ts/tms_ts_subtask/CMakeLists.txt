cmake_minimum_required(VERSION 2.8.3)
project(tms_ts_subtask)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tms_msg_db
  tms_msg_ts
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tms_ts_subtask
  CATKIN_DEPENDS roscpp std_msgs tms_msg_db tms_msg_ts
 # DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tms_ts_subtask src/tms_ts_subtask.cpp)
add_dependencies(tms_ts_subtask tms_msg_db_generate_messages_cpp tms_msg_ts_generate_messages_cpp)
target_link_libraries(tms_ts_subtask ${catkin_LIBRARIES})

add_executable(tms_ts_subtask_957 src/tms_ts_subtask_957.cpp)
add_dependencies(tms_ts_subtask_957 tms_msg_db_generate_messages_cpp tms_msg_ts_generate_messages_cpp)
target_link_libraries(tms_ts_subtask_957 ${catkin_LIBRARIES})


