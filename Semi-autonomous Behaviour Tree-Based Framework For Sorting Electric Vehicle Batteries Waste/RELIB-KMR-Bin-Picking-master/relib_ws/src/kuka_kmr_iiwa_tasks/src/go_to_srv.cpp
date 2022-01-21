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
/*Editedt by: Hector Cruz @ERLB University of Birmingham*/
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <kuka_msgs/Place.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#define PI 3.141516

const float offest_z_ee_to_palm = 0.246;
static const std::string PLANNING_GROUP = "lbr_arm";

float pitch = 0;


ros::ServiceClient vg10_trigger;
// moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
// std::vector<double> joint_group_positions;
// moveit::planning_interface::MoveGroupInterface::Plan my_plan;


bool place_execution(kuka_msgs::Place::Request& req, kuka_msgs::Place::Response& res ){
    move_group.setPlanningTime(2.0);
    if (req.bin != 0){

    }else {
        ROS_INFO("Putting object to custom pose");
        tf2::Quaternion orientation;
        orientation.setRPY(0 , pitch, 0);

        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
        geometry_msgs::Pose target_pose1;
        target_pose1.orientation = tf2::toMsg(orientation);
        target_pose1.position.x = req.place_pose.pose.position.x;
        target_pose1.position.y = req.place_pose.pose.position.y;
        target_pose1.position.z = req.place_pose.pose.position.z + offest_z_ee_to_palm;
        move_group.setPoseTarget(target_pose1);


        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        moveit::planning_interface::MoveItErrorCode log =  move_group.move();

        if (log == moveit::planning_interface::MoveItErrorCode::SUCCESS){
            res.status = 1;
            //move_group.detachObject("object");  

        }else{
            res.status = 0;
            move_group.detachObject("object");  
        }
        }
    ros::service::waitForService("vg10_actioner",2);
    std_srvs::SetBool activate;
    activate.request.data = false;
    if(vg10_trigger.call(activate)){
        ROS_INFO_STREAM("VG10 deactivated...");
    }
    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "go_to_srv");
    ros::NodeHandle node_handle;

    ros::ServiceServer place_server = node_handle.advertiseService("place_2", place_execution);
    vg10_trigger = node_handle.serviceClient<std_srvs::SetBool>("vg10_actioner");
    ros::AsyncSpinner spinner(1);
    spinner.start();


	pitch = 180*PI/180;


    // move_group.setMaxAccelerationScalingFactor(0.3);
    // move_group.setMaxVelocityScalingFactor(0.4);

    // // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // // class to add and remove collision objects in our "virtual world" scene
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // // Raw pointers are frequently used to refer to the planning group for improved performance.
    // const robot_state::JointModelGroup* joint_model_group =
    //     move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // // Visualization
    // // ^^^^^^^^^^^^^
    // //
    // // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    // namespace rvt = rviz_visual_tools;
    // moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint");
    // visual_tools.deleteAllMarkers();

    // // Remote control is an introspection tool that allows users to step through a high level script
    // // via buttons and keyboard shortcuts in RViz
    // visual_tools.loadRemoteControl();

    // // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    // text_pose.translation().z() = 1.75;
    // visual_tools.publishText(text_pose, "MoveGroupInterface Demo LBR", rvt::WHITE, rvt::XLARGE);



    ros::waitForShutdown();
    return 0;
}