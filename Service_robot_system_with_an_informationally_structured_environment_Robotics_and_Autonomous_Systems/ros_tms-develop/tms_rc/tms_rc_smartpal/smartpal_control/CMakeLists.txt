cmake_minimum_required(VERSION 2.8.3)
project(tms_rc_smartpal_control)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  tf 
  geometry_msgs 
  nav_msgs
  tms_msg_rc
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   #INCLUDE_DIRS include
   #LIBRARIES smartpal_control
   CATKIN_DEPENDS roscpp tf geometry_msgs nav_msgs message_runtime
   #DEPENDS system_lib
)

###########
## Build ##
###########
set(smartpalIDLPath ~/catkin_ws/src/ros_tms/tms_rc/tms_rc_smartpal/smartpal_control/src/idl/sp5/)

include_directories(
  include
  SYSTEM
  ${smartpalIDLPath}
  ${catkin_INCLUDE_DIRS}
)

add_executable(smartpal5_control
   
    src/smartpal5_control.cpp 
   
    src/sp5_client.cpp    
    src/sp5_cmd_mobility.cpp
    src/sp5_cmd_arm.cpp
    src/sp5_cmd_gripper.cpp
    src/sp5_cmd_lumbar.cpp

    src/idl/sp5/CommonSK.cc

    src/idl/sp5/MobilitySK.cc

    src/idl/sp5/ArmUnitSK.cc
    
    src/idl/sp5/GripperUnitSK.cc
    src/idl/sp5/GripperExtAxesSK.cc

    src/idl/sp5/LumbarUnitSK.cc
    src/idl/sp5/LumbarExtAxesSK.cc
)

add_dependencies(smartpal5_control tms_msg_rc_gencpp)

target_link_libraries(smartpal5_control ${catkin_LIBRARIES} omniORB4 omnithread)

