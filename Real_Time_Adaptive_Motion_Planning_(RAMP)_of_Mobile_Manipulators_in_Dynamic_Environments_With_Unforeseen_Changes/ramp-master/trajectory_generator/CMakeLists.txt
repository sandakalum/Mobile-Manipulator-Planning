cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation ramp_msgs roscpp)


#########################################
#### Declare ROS messages and services ##
#########################################


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS 
)


#############
#### Build ##
#############

#### Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

#### Debugging flag for using gdb
set (CMAKE_CXX_FLAGS "-g")



#### Declare a cpp executable
add_executable(${PROJECT_NAME} src/bezier_curve.cpp src/circle.cpp src/line.cpp src/main.cpp src/mobile_base.cpp src/prediction.cpp src/utility.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ReflexxesTypeII)
add_dependencies(${PROJECT_NAME} ramp_msgs_generate_messages_cpp)



## ============= Testing Section =============================================

catkin_add_gtest(trajectory_generator_testFunctionality test/trajectory_generator_testFunctionality.cpp src/bezier_curve.cpp src/circle.cpp src/line.cpp src/mobile_base.cpp src/prediction.cpp src/utility.cpp)
target_link_libraries(trajectory_generator_testFunctionality ${catkin_LIBRARIES} ReflexxesTypeII)

catkin_add_gtest(trajectory_generator_testPerformance test/trajectory_generator_testPerformance.cpp src/bezier_curve.cpp src/circle.cpp src/line.cpp src/mobile_base.cpp src/prediction.cpp src/utility.cpp)
target_link_libraries(trajectory_generator_testPerformance ${catkin_LIBRARIES} ReflexxesTypeII)

##============================================================================
