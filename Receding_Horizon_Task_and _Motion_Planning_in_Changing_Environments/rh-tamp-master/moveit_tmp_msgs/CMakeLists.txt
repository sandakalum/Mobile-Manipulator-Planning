cmake_minimum_required(VERSION 3.0.2)
project(moveit_tmp_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	moveit_msgs
)

# ROS messages, services and actions
add_message_files(DIRECTORY msg FILES
	SubTrajectory.msg
)


add_action_files(DIRECTORY action FILES
	ExecuteTMPSolution.action
)

generate_messages(DEPENDENCIES moveit_msgs)

catkin_package(
	CATKIN_DEPENDS
		message_runtime
		moveit_msgs
)
