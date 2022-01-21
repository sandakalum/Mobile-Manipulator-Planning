cmake_minimum_required(VERSION 2.8.3)
project(tms_ss_fss_exe)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tms_ss_fss_exe
  CATKIN_DEPENDS roscpp sensor_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fss_accuracy            src/fss_accuracy.cpp)
add_executable(fss_preprocess          src/fss_preprocess.cpp)
add_executable(fss_transform           src/fss_transform.cpp)
add_executable(fss_cluster             src/fss_cluster.cpp)
add_executable(fss_classify            src/fss_classify.cpp)
add_executable(fss_track               src/fss_track.cpp)
add_executable(fss_chair_buffer        src/fss_chair_buffer.cpp)
add_executable(fss_person_tracking     src/fss_person_tracking.cpp include/tms_ss_fss_exe/Cluster.cpp)

add_dependencies(fss_accuracy DEPENDS tms_msg_ss_gencpp)
add_dependencies(fss_preprocess DEPENDS tms_msg_ss_gencpp)
add_dependencies(fss_transform DEPENDS tms_msg_ss_gencpp)
add_dependencies(fss_cluster DEPENDS tms_msg_ss_gencpp)
add_dependencies(fss_classify DEPENDS tms_msg_ss_gencpp)
add_dependencies(fss_track DEPENDS tms_msg_ss_gencpp)
add_dependencies(fss_chair_buffer DEPENDS tms_msg_ss_gencpp)
add_dependencies(fss_person_tracking DEPENDS tms_msg_ss_gencpp)

target_link_libraries(fss_accuracy ${catkin_LIBRARIES})
target_link_libraries(fss_preprocess ${catkin_LIBRARIES})
target_link_libraries(fss_transform ${catkin_LIBRARIES})
target_link_libraries(fss_cluster ${catkin_LIBRARIES})
target_link_libraries(fss_classify ${catkin_LIBRARIES})
target_link_libraries(fss_track ${catkin_LIBRARIES})
target_link_libraries(fss_chair_buffer ${catkin_LIBRARIES})
target_link_libraries(fss_person_tracking ${catkin_LIBRARIES})

