cmake_minimum_required(VERSION 2.8.3)
project(trajectory_evaluation)

### Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS message_generation ramp_msgs roscpp)

########################################
### Declare ROS messages and services ##
########################################


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS 
)

############
### Build ##
############

include_directories(include ${catkin_INCLUDE_DIRS})

### Debugging flag for using gdb
set(CMAKE_CXX_FLAGS "-g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")


### Declare a cpp executable
add_executable(${PROJECT_NAME} src/main.cpp src/collision_detection.cpp src/euclidean_distance.cpp src/evaluate.cpp src/orientation.cpp src/utility.cpp)
#
## Add the -std argument to compile enum
#set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS -std=c++0x)
#
### Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#
### Add cmake target dependencies of the executable/library
### as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ramp_msgs_generate_messages_cpp)






## ============= Testing Section =============================================

catkin_add_gtest(trajectory_evaluation_testFunctionality test/trajectory_evaluation_testFunctionality.cpp src/collision_detection.cpp src/euclidean_distance.cpp src/evaluate.cpp src/orientation.cpp src/utility.cpp)

target_link_libraries(trajectory_evaluation_testFunctionality ${catkin_LIBRARIES} pthread)

catkin_add_gtest(trajectory_evaluation_testPerformance test/trajectory_evaluation_testPerformance.cpp src/collision_detection.cpp src/euclidean_distance.cpp src/evaluate.cpp src/orientation.cpp src/utility.cpp)
target_link_libraries(trajectory_evaluation_testPerformance ${catkin_LIBRARIES} pthread)

##============================================================================


