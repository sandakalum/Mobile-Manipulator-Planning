cmake_minimum_required(VERSION 2.8.3)
project(ramp_debug)

find_package(catkin REQUIRED roscpp nav_msgs ramp_msgs)

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


add_executable(viewPredicted src/viewPredicted.cpp src/utility.cpp src/trajectory_request_handler.cpp)
set_target_properties(viewPredicted PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(viewPredicted ${catkin_LIBRARIES})
add_dependencies(viewPredicted ramp_msgs_generate_messages_cpp)


add_executable(publishTwistCommand src/publish_twist_command.cpp)
set_target_properties(publishTwistCommand PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(publishTwistCommand ${catkin_LIBRARIES})
add_dependencies(publishTwistCommand ramp_msgs_generate_messages_cpp)


add_executable(publishTrajectoryCommand src/publish_trajectory_command.cpp src/utility.cpp)
set_target_properties(publishTrajectoryCommand PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(publishTrajectoryCommand ${catkin_LIBRARIES})
add_dependencies(publishTrajectoryCommand ramp_msgs_generate_messages_cpp)

add_executable(getYaw src/getYaw.cpp)
set_target_properties(getYaw PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(getYaw ${catkin_LIBRARIES})
add_dependencies(getYaw ramp_msgs_generate_messages_cpp)

add_executable(imminent_collision src/imminent_collision.cpp)
set_target_properties(imminent_collision PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(imminent_collision ${catkin_LIBRARIES})
add_dependencies(imminent_collision ramp_msgs_generate_messages_cpp)

add_executable(interrupt_trajectory src/interrupt_trajectory.cpp src/utility.cpp)
set_target_properties(interrupt_trajectory PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(interrupt_trajectory ${catkin_LIBRARIES})
add_dependencies(interrupt_trajectory ramp_msgs_generate_messages_cpp)

add_executable(evaluation_test src/evaluation_test.cpp src/utility.cpp)
set_target_properties(evaluation_test PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(evaluation_test ${catkin_LIBRARIES})
add_dependencies(evaluation_test ramp_msgs_generate_messages_cpp)


add_executable(testCollision src/test_collision.cpp src/utility.cpp)
set_target_properties(testCollision PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(testCollision ${catkin_LIBRARIES})
add_dependencies(testCollision ramp_msgs_generate_messages_cpp)

add_executable(test_move_op src/test_move_op.cpp src/utility.cpp)
set_target_properties(test_move_op PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(test_move_op ${catkin_LIBRARIES})
add_dependencies(test_move_op ramp_msgs_generate_messages_cpp)


