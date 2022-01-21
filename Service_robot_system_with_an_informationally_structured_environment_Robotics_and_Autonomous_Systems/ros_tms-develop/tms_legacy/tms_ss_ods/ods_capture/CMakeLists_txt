cmake_minimum_required(VERSION 2.8.3)
project(tms_ss_ods_capture)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_ros
  roscpp
  sensor_msgs
  tms_msg_ss
)

catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
 add_executable(ods_capture_PointXYZI src/ods_capture_PointXYZI.cpp)
 add_executable(ods_capture_PointXYZ src/ods_capture_PointXYZ.cpp)
 add_executable(ods_capture_PointXYZRGB src/ods_capture_PointXYZRGB.cpp)
 add_executable(ods_capture_RGBImage src/ods_capture_RGBImage.cpp)
 add_executable(ods_capture_DepthImage src/ods_capture_DepthImage.cpp)
 add_executable(ods_capture_IRImage src/ods_capture_IRImage.cpp)
 add_executable(ods_capture src/ods_capture.cpp)
 add_executable(ods_smoothing_capture src/ods_smoothing_capture.cpp)
 add_executable(ods_capture_exbed src/ods_capture_exbed.cpp)
 
 target_link_libraries(ods_capture
   ${catkin_LIBRARIES}
 )
 target_link_libraries(ods_capture_PointXYZ
   ${catkin_LIBRARIES}
 )
 target_link_libraries(ods_capture_PointXYZRGB
   ${catkin_LIBRARIES}
 )
 target_link_libraries(ods_capture_PointXYZI
   ${catkin_LIBRARIES}
 )
 target_link_libraries(ods_capture_RGBImage
   ${catkin_LIBRARIES}
 )
 target_link_libraries(ods_capture_DepthImage
   ${catkin_LIBRARIES}
 )
 target_link_libraries(ods_capture_IRImage
   ${catkin_LIBRARIES}
 )
 target_link_libraries(ods_smoothing_capture
   ${catkin_LIBRARIES}
 )
 target_link_libraries(ods_capture_exbed
   ${catkin_LIBRARIES}
 )

add_dependencies(ods_capture DEPENDS tms_msg_ss_gencpp)
add_dependencies(ods_capture_PointXYZ DEPENDS tms_msg_ss_gencpp)
add_dependencies(ods_capture_PointXYZRGB DEPENDS tms_msg_ss_gencpp)
add_dependencies(ods_capture_PointXYZI DEPENDS tms_msg_ss_gencpp)
add_dependencies(ods_capture_RGBImage DEPENDS tms_msg_ss_gencpp)
add_dependencies(ods_capture_DepthImage DEPENDS tms_msg_ss_gencpp)
add_dependencies(ods_capture_IRImage DEPENDS tms_msg_ss_gencpp)
add_dependencies(ods_smoothing_capture DEPENDS tms_msg_ss_gencpp)
add_dependencies(ods_capture_exbed DEPENDS tms_msg_ss_gencpp)
