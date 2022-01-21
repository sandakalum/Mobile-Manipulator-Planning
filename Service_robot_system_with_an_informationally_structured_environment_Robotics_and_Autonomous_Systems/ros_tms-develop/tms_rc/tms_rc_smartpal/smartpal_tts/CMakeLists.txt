cmake_minimum_required(VERSION 2.8.3)
project(tms_rc_smartpal_tts)
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs
  tms_msg_rc
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(smartpal_tts src/smartpal_tts.cpp)
add_dependencies(smartpal_tts tms_msg_rc_gencpp)
target_link_libraries(smartpal_tts ${catkin_LIBRARIES})

