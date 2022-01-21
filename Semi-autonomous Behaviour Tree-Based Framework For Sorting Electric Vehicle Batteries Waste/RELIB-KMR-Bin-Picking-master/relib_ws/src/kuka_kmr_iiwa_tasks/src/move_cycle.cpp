/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_cycle");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // BEGIN_TUTORIAL

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP = "lbr_arm";
    

    // The :move_group_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    // ^^^^^^^^^^^^^
    //
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint");
    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo LBR", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    // visual_tools.trigger();

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    //   ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

 

	ros::Rate rate(1); // 10 hz
	std::vector<double> joint_group_positions;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	   // Start the demo

    visual_tools.prompt("Press 'next' to start moving according to plan");

	// STARTING ROUTINE
	while(ros::ok){

		moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	
		geometry_msgs::Pose target_pose1;
		target_pose1.orientation.w = 1.0;
		target_pose1.position.x = 0.5;
		target_pose1.position.y = -0.6;
		target_pose1.position.z = 1.5;
		move_group.setPoseTarget(target_pose1);

		// Now, we call the planner to compute the plan and visualize it.
		// Note that we are just planning, not asking move_group
		// to actually move the robot.
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;

		bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

		// Visualizing plans
		// ^^^^^^^^^^^^^^^^^
		// We can also visualize the plan as a line with markers in RViz.
		ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
		visual_tools.publishAxisLabeled(target_pose1, "pose1");
		visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		// visual_tools.trigger();
		//visual_tools.prompt("Plan 1: Robot moviing...");
		move_group.move();

		rate.sleep();
		// Get current state
		current_state = move_group.getCurrentState();
		// Next get the current set of joint values for the group.
		current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

		// Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
		joint_group_positions[3] = +1.0;  // radians
		joint_group_positions[4] = -1.0;  // radians
		joint_group_positions[5] = +0.5;  // radians
		move_group.setJointValueTarget(joint_group_positions);

		success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

		// Visualize the plan in RViz
		visual_tools.deleteAllMarkers();
		visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		// visual_tools.trigger();
		//visual_tools.prompt("Plan 2 Robot moviing...");

		// Move the actual robot on Gazebo
		move_group.move();
		rate.sleep();
		// TODO: Add a chrono wait here
		current_state = move_group.getCurrentState();
		// Next get the current set of joint values for the group.
		
		current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

		// Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
		joint_group_positions[0] = +0.8;  // radians
		joint_group_positions[1] = +0.3;  // radians
		joint_group_positions[2] = +.0;  // radians
		joint_group_positions[3] = +1.0;  // radians
		joint_group_positions[4] = -1.0;  // radians
		joint_group_positions[5] = +0.5;  // radians
		move_group.setJointValueTarget(joint_group_positions);

		success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (joint space goal) %s", success ? "" : "FAILED");

		// Visualize the plan in RViz
		visual_tools.deleteAllMarkers();
		visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		// visual_tools.trigger();
		//visual_tools.prompt("Plan 3: Robot moviing...");

		// Move the actual robot on Gazebo
		move_group.move();
		rate.sleep();
		// TODO: Add a chrono wait here
		current_state = move_group.getCurrentState();
		// Next get the current set of joint values for the group.
		
		current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

		// Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
		joint_group_positions[0] = 0;  // radians
		joint_group_positions[1] = 0;  // radians
		joint_group_positions[2] = 0;  // radians
		joint_group_positions[3] = 0;  // radians
		joint_group_positions[4] = 0;  // radians
		joint_group_positions[5] = 0.7;  // radians
		joint_group_positions[6] = 0;  // radians
		move_group.setJointValueTarget(joint_group_positions);

		success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (joint space goal) %s", success ? "" : "FAILED");

		// Visualize the plan in RViz
		visual_tools.deleteAllMarkers();
		visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		// visual_tools.trigger();
		//visual_tools.prompt("Plan 4: Robot moviing...");

		// Move the actual robot on Gazebo
		move_group.move();


	}
    
    // END_TUTORIAL

    ros::shutdown();
    return 0;
}