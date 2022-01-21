cmake_minimum_required(VERSION 2.8.3)
project(tms_rc_pioneer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
  INCLUDE_DIRS /usr/local/Aria/include #http://robots.mobilerobots.com/wiki/ARIA >>> install ARIA 2.8.1
#  LIBRARIES pioneer_test
  CATKIN_DEPENDS roscpp message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/Aria/include
)

link_directories(/usr/local/Aria/lib)
add_executable(test1_move src/test1_move.cpp)
target_link_libraries(test1_move ${catkin_LIBRARIES})
target_link_libraries(test1_move Aria pthread dl rt)
add_dependencies(test1_move DEPENDS tms_msg_ss_gencpp)
