cmake_minimum_required(VERSION 2.8.3)
project(tms_ss_pot)
# Debug Build
# set(CMAKE_BUILD_TYPE Debug)

###################################
## packege  find   configuration ##
###################################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tms_msg_ss
  tms_msg_db
  visualization_msgs
  cv_bridge
  geometry_msgs
  )

#find_package(Qt5 REQUIRED COMPONENTS QtCore QtGui)
#include(${QT_USE_FILE})

find_package(Qt5Widgets)
find_package(Qt5Core)
find_package(Qt5Gui)

find_package(Boost REQUIRED COMPONENTS thread)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES tms_ss_pot
    CATKIN_DEPENDS message_runtime roscpp std_msgs
    #DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  /usr/local/include/opencv2
  /usr/local/include/opencv
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${QT_INCLUDE_DIRS}
)

## Declare a cpp library
 add_library(tms_ss_pot
    include/target.cpp
    include/laser.cpp
    include/particle_filter.cpp
    include/multiple_particle_filter.cpp
 )

add_executable(urg_tracker src/urg_tracker.cpp include/target.cpp include/laser.cpp include/particle_filter.cpp include/multiple_particle_filter.cpp include/condens/condens.cpp)
add_executable(urg_single_tracker src/urg_single_tracker.cpp include/target.cpp include/laser.cpp include/particle_filter.cpp include/multiple_particle_filter.cpp include/condens/condens.cpp)
add_executable(change_nan src/change_nan.cpp include/condens/condens.cpp)
add_executable(visualize src/tracker_visualization.cpp include/condens/condens.cpp)
add_executable(laser_visualize_single src/laser_visualization_single.cpp include/condens/condens.cpp)
add_executable(laser_visualize_multi src/laser_visualization_multi.cpp include/condens/condens.cpp)
add_executable(ninebot_cropper src/ninebot_cropper.cpp)

add_dependencies(urg_tracker tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
add_dependencies(urg_single_tracker tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
add_dependencies(change_nan tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
add_dependencies(visualize tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
add_dependencies(laser_visualize_single tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
add_dependencies(laser_visualize_multi tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
add_dependencies(ninebot_cropper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(urg_tracker ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(urg_single_tracker ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(change_nan ${catkin_LIBRARIES})
target_link_libraries(visualize ${catkin_LIBRARIES})
target_link_libraries(laser_visualize_single ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(laser_visualize_multi ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(ninebot_cropper ${catkin_LIBRARIES})
