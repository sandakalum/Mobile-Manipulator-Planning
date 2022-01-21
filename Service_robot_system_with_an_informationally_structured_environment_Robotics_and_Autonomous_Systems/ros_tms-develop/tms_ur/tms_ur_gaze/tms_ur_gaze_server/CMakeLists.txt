cmake_minimum_required(VERSION 2.8.3)
project(tms_ur_gaze_server)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  nodelet
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)

## set the plugin name
set(PLUGIN_NAME tms_ur_gaze)

add_service_files(
  FILES
  object_list.srv
  recognized_text.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES tmd_ur_gaze_server ${PLUGIN_MAME}
   CATKIN_DEPENDS message_runtime pluginlib nodelet roscpp rospy std_msgs
)

## setup the include dirs
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## nodelet library
add_library(${PLUGIN_NAME}
  src/nodelet.cpp
  src/evaluator.cpp
  src/voice_receiver.cpp
)
add_dependencies(${PLUGIN_NAME} tms_ur_gaze_server_generate_messages_cpp)
target_link_libraries(${PLUGIN_NAME}
  ${catkin_LIBRARIES}
)

## nodelet loader
add_executable(tms_ur_gaze_server
  src/tms_ur_gaze_server.cpp
)
add_dependencies(tms_ur_gaze_server tms_ur_gaze_server_generate_messages_cpp)
target_link_libraries(tms_ur_gaze_server
  ${catkin_LIBRARIES}
)
