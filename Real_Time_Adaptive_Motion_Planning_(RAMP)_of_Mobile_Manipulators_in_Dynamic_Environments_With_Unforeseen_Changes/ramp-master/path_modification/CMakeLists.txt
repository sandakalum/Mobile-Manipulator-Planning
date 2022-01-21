cmake_minimum_required(VERSION 2.8.3)
project(path_modification)
#
find_package(catkin REQUIRED COMPONENTS message_generation ramp_msgs roscpp)
#
#
########################################
### Declare ROS messages and services ##
########################################
#
#
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS 
)


############
### Build ##
############

### Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

### Debugging flag for using gdb
set (CMAKE_CXX_FLAGS "-g")

### Declare a cpp executable
add_executable(${PROJECT_NAME} src/main.cpp src/change.cpp src/constraints.cpp src/crossover.cpp src/delete.cpp src/insert.cpp src/modifier.cpp src/move.cpp src/range.cpp src/swap.cpp src/utility.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ramp_msgs_generate_messages_cpp)




#========# Unit tests ==========#

catkin_add_gtest(insert_test test/insert_test.cpp src/constraints.cpp src/utility.cpp src/insert.cpp)
target_link_libraries(insert_test ${catkin_LIBRARIES})

catkin_add_gtest(delete_test test/delete_test.cpp src/constraints.cpp src/utility.cpp src/delete.cpp)
target_link_libraries(delete_test ${catkin_LIBRARIES})

catkin_add_gtest(change_test test/change_test.cpp src/constraints.cpp src/utility.cpp src/change.cpp)
target_link_libraries(change_test ${catkin_LIBRARIES})

catkin_add_gtest(swap_test test/swap_test.cpp src/constraints.cpp src/utility.cpp src/swap.cpp)
target_link_libraries(swap_test ${catkin_LIBRARIES})




#========# Sub-system tests ==========#

catkin_add_gtest(path_modification_testFunctionality test/path_modification_testFunctionality.cpp src/change.cpp src/constraints.cpp src/crossover.cpp src/delete.cpp src/insert.cpp src/modifier.cpp src/move.cpp src/range.cpp src/swap.cpp src/utility.cpp)
target_link_libraries(path_modification_testFunctionality ${catkin_LIBRARIES} pthread)

catkin_add_gtest(path_modification_testPerformance test/path_modification_testPerformance.cpp src/change.cpp src/constraints.cpp src/crossover.cpp src/delete.cpp src/insert.cpp src/modifier.cpp src/move.cpp src/range.cpp src/swap.cpp src/utility.cpp)
target_link_libraries(path_modification_testPerformance ${catkin_LIBRARIES} pthread)

##============================================================================
