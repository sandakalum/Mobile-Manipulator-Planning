cmake_minimum_required(VERSION 2.8.3)
project(ramp_planner)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation ramp_msgs roscpp std_msgs)



########################################
### Declare ROS messages and services ##
########################################


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

############
### Build ##
############

# Specify include directories
include_directories(include ${catkin_INCLUDE_DIRS} /opt/ros/kinetic/include)

## Debugging flag for using gdb
set (CMAKE_CXX_FLAGS "-g")

## Declare a cpp executable
add_executable(${PROJECT_NAME} src/bezier_curve.cpp src/main.cpp src/planner.cpp src/control_handler.cpp src/knot_point.cpp src/modification_request_handler.cpp src/modifier.cpp src/motion_state.cpp src/parameter_handler.cpp src/path.cpp src/population.cpp src/ramp_trajectory.cpp src/range.cpp src/trajectory_request_handler.cpp src/evaluation_request_handler.cpp src/utility.cpp)

# Add the -std argument to compile enum
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS -std=c++0x)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ramp_msgs_generate_messages_cpp)


## Declare a cpp executable
add_executable(obstacle_trajectory src/control_handler.cpp src/knot_point.cpp src/main_obstacle_trajectory.cpp src/motion_state.cpp src/path.cpp src/ramp_trajectory.cpp src/range.cpp src/trajectory_request_handler.cpp src/utility.cpp)

# Add the -std argument to compile enum
set_target_properties(obstacle_trajectory PROPERTIES COMPILE_FLAGS -std=c++0x)

## Specify libraries to link a library or executable target against
target_link_libraries(obstacle_trajectory ${catkin_LIBRARIES} yaml-cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(obstacle_trajectory ramp_msgs_generate_messages_cpp)


## Declare a cpp executable
add_executable(obstacle_twist_all src/main_obstacle_twist_all.cpp src/range.cpp src/utility.cpp)

# Add the -std argument to compile enum
set_target_properties(obstacle_twist_all PROPERTIES COMPILE_FLAGS -std=c++0x)

## Specify libraries to link a library or executable target against
target_link_libraries(obstacle_twist_all ${catkin_LIBRARIES} yaml-cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(obstacle_twist_all ramp_msgs_generate_messages_cpp)



# System-level testing executables
add_executable(run_test_case src/bezier_curve.cpp src/main_run_test_case.cpp src/planner.cpp src/control_handler.cpp src/knot_point.cpp src/modification_request_handler.cpp src/modifier.cpp src/motion_state.cpp src/parameter_handler.cpp src/path.cpp src/population.cpp src/ramp_trajectory.cpp src/range.cpp src/trajectory_request_handler.cpp src/evaluation_request_handler.cpp src/utility.cpp)
set_target_properties(run_test_case PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(run_test_case ${catkin_LIBRARIES} yaml-cpp)

add_executable(generate_test_case src/bezier_curve.cpp src/main_generate_test_case.cpp src/planner.cpp src/control_handler.cpp src/knot_point.cpp src/modification_request_handler.cpp src/modifier.cpp src/motion_state.cpp src/parameter_handler.cpp src/path.cpp src/population.cpp src/ramp_trajectory.cpp src/range.cpp src/trajectory_request_handler.cpp src/evaluation_request_handler.cpp src/utility.cpp)
set_target_properties(generate_test_case PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(generate_test_case ${catkin_LIBRARIES} yaml-cpp)




## ============= Testing Section =============================================
catkin_add_gtest(fullPath_testFunctionality test/fullPath_testFunctionality.cpp src/range.cpp src/utility.cpp)
set_target_properties(fullPath_testFunctionality PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(fullPath_testFunctionality ${catkin_LIBRARIES} pthread)
##============================================================================
