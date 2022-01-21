cmake_minimum_required(VERSION 2.8.3)
project(ramp_sensing)

find_package(catkin REQUIRED COMPONENTS map_msgs message_generation nav_msgs ramp_msgs roscpp tf)
find_package(OpenCV REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp ramp_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

## Debugging flag for using gdb
set (CMAKE_CXX_FLAGS "-std=c++11 -g -O0")


add_executable(${PROJECT_NAME} src/circle_packer.cpp src/GridMap2D.cpp src/main.cpp src/obstacle.cpp src/utility.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(${PROJECT_NAME} ramp_msgs_generate_messages_cpp)

