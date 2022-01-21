cmake_minimum_required(VERSION 2.8.3)
project(tms_ur_slack)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation message_runtime)

add_service_files(FILES slack_srv.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
   LIBRARIES tms_ur_slack
   CATKIN_DEPENDS rospy std_msgs message_generation
)
