cmake_minimum_required(VERSION 2.8.3)
project(vitalmonitor)

find_package(catkin REQUIRED COMPONENTS
  OpenCV REQUIRED
  roscpp
  rospy
  sensor_msgs
  message_generation
  std_msgs
  cv_bridge
  image_transport
)
 add_message_files(
  FILES
  face_info.msg
  emotion.msg
)

generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vitalmonitor image_sender
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs 
  DEPENDS opencv
)

###########
## Build ##
###########

include_directories(
  include
  $ENV{OPENNI2_INCLUDE}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories($ENV{OPENNI2_REDIST})
link_directories($ENV{OPENNI2_REDIST}/OpenNI2/Drivers)
#link_directories(~/OpenNI2/Tools/OpenNI2/Drivers)


#find_library(OpenNI2_LIBRARY NAMES OpenNI2 libOpenNI2 libOniFile libSenDuck
#  PATHS
#    ~/OpenNI2
#    ~/OpenNI2/Redist
#    ~/OpenNI2/Redist/OpenNI2/Drivers
#    ~/OpenNI2/Tools/OpenNI2/Drivers
#  )

## Declare a cpp executable

 add_executable(vitalmonitor src/vitalmonitor.cpp src/noiseReduction.cpp src/waveFilter.cpp)

 add_dependencies(vitalmonitor 
  tms_msg_rc_gencpp
  tms_msg_db_gencpp
  )


## Specify libraries to link a library or executable target against
 target_link_libraries(vitalmonitor
   ${catkin_LIBRARIES}
  OpenNI2
  OniFile
  SenDuck
  PS1080
  PSLink
   ${OpenCV_LIBS} 
 )
