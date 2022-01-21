cmake_minimum_required(VERSION 2.8.3)
project(tms_ts_smach)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_ros
)

catkin_package(
  CATKIN_DEPENDS rospy smach smach_ros
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
