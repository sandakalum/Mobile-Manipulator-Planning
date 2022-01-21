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

/**Refracted by: Hector Cruz @University of birmingham**/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <kuka_msgs/ArmPose.h>

#define PI 3.141516


static const std::string PLANNING_GROUP = "lbr_arm";
static const std::string EYE_POSE = "eye";
static const std::string TOWARDS_TABLE  = "towards_table";
static const std::string BACK = "back";
static const std::string HOME = "home table";
static const std::string EXTENDED ="extended";
static const std::string FRONT_1 = "front_1";
static const std::string FRONT_2 = "front_2";


bool move_pose_callback(kuka_msgs::ArmPose::Request& req, kuka_msgs::ArmPose::Response& res){

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPlanningTime(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    move_group.setMaxVelocityScalingFactor(1.0);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint");
    visual_tools.deleteAllMarkers();

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("Arm_pose", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    move_group.setNamedTarget(req.pose_id);

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Arm_pose", "Visualizing plan %s ", success ? "" : "FAILED");
    
    bool target_reached = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    res.status = target_reached;
    ROS_INFO_NAMED("Arm Pose", "pose  %s", target_reached ? "REACHED SUCCESSFULLY" : "FAILED");

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"arm_pose_server");
    
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::ServiceServer pose_server = nh.advertiseService("arm_pose", move_pose_callback);

    kuka_msgs::ArmPose::Request  req;
    kuka_msgs::ArmPose::Response res;
    if(argv[1] != NULL){
        req.pose_id = std::string(argv[1]);
    }else{
        req.pose_id = "home_table";
    }
    ROS_INFO("Sending robot to its initial configuration: ' %s '", req.pose_id.c_str());
    bool status = move_pose_callback(req,res);
    ROS_INFO_STREAM("Arm pose ready!");

    ros::waitForShutdown();
    //ros::spin();
    return 0;
}