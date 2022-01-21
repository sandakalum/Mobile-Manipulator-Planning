cmake_minimum_required(VERSION 2.8.3)
project(ramp_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg message_generation nav_msgs std_msgs trajectory_msgs)


#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(FILES BezierCurve.msg EvaluationRequest.msg EvaluationResponse.msg KnotPoint.msg MotionState.msg Path.msg RampTrajectory.msg Range.msg Obstacle.msg ObstacleList.msg Population.msg TrajectoryRequest.msg TrajectoryResponse.msg)

## Generate services in the 'srv' folder
add_service_files(FILES EvaluationSrv.srv ModificationRequest.srv TrajectorySrv.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES nav_msgs std_msgs trajectory_msgs)

catkin_package(CATKIN_DEPENDS message_runtime nav_msgs std_msgs trajectory_msgs)

