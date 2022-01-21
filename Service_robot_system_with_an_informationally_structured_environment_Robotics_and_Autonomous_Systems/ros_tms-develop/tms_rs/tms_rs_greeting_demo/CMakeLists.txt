cmake_minimum_required(VERSION 2.8.3)
project(tms_rs_greeting_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tms_msg_rs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tms_rs_greeting_demo
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
# add_executable(tms_rs_greeting src/tms_rs_greeting.cpp)
# add_dependencies(tms_rs_greeting tms_msg_rs_gencpp tms_rs_generate_messages_cpp tms_rc_generate_messages_cpp)
# target_link_libraries(tms_rs_greeting
#   ${catkin_LIBRARIES}
# )
 
 add_executable(tms_rs_motion src/tms_rs_motion.cpp)
 add_dependencies(tms_rs_motion tms_msg_rs_gencpp)
 target_link_libraries(tms_rs_motion
   ${catkin_LIBRARIES}
 )
 
 add_executable(tms_rs_speech src/tms_rs_speech.cpp)
 add_dependencies(tms_rs_speech tms_msg_rs_gencpp)
 target_link_libraries(tms_rs_speech
   ${catkin_LIBRARIES}
 )
 
 add_executable(ref_demo src/refrigerator_demo.cpp)
 add_dependencies(ref_demo tms_msg_rc_gencpp)
 target_link_libraries(ref_demo
   ${catkin_LIBRARIES}
 )
 
 add_executable(greeting_demo src/greeting_demo.cpp)
 add_dependencies(greeting_demo tms_msg_rc_gencpp)
 target_link_libraries(greeting_demo
   ${catkin_LIBRARIES}
 )
 
 add_executable(greeting_demo2 src/greeting_demo_2.cpp)
 add_dependencies(greeting_demo2 tms_msg_rc_gencpp)
 target_link_libraries(greeting_demo2
   ${catkin_LIBRARIES}
 )
