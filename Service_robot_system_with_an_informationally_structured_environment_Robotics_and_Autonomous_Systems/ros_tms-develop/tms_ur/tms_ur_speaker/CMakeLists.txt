cmake_minimum_required(VERSION 2.8.3)
project(tms_ur_speaker)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation message_runtime)

add_service_files(FILES speaker_srv.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
   LIBRARIES tms_ur_speaker
   CATKIN_DEPENDS rospy std_msgs message_generation
)
