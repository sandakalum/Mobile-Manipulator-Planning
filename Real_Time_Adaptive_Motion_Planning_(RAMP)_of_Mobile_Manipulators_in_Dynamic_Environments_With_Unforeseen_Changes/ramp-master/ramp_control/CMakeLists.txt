cmake_minimum_required(VERSION 2.8.3)
project(ramp_control)

find_package(catkin REQUIRED COMPONENTS genmsg geometry_msgs message_generation nav_msgs ramp_msgs roscpp std_msgs)

#######################################
## Declare ROS messages and services ##
#######################################



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp 
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

## Debugging flag for using gdb
set (CMAKE_CXX_FLAGS "-g")

add_executable(${PROJECT_NAME} src/main.cpp src/mobile_robot.cpp src/utility.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ramp_msgs_generate_messages_cpp)

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
target_link_libraries(keyboard_teleop ${catkin_LIBRARIES})
add_dependencies(keyboard_teleop ramp_msgs_generate_messages_cpp)

