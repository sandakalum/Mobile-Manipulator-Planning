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
/*Edited by: Hector Cruz @ UoB ERL*/
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/tf.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/GetModelState.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define PI 3.141516

const float offest_z_ee_to_palm = 0.246;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go_to");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);

    ros::ServiceClient get_model_srv = node_handle.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
    spinner.start();

    gazebo_msgs::GetModelState get_model_msg;

    std::string object_name;

    float x, y, z, offset;
    float roll = 0.0;
    float pitch = PI;
    float yaw = 0.0;
    bool mode;
    int constraint = 0;
    if (argv[1] != NULL)
    {
        if (*(argv[1]) == 'o')
        {
            mode = true;
            object_name = std::string(argv[2]);
            offset = std::stof(argv[3]);
        }
        else
        {
            mode = false;
            x = std::stof(argv[2]);
            y = std::stof(argv[3]);
            z = std::stof(argv[4]);
            roll = std::stof(argv[5]) * PI / 180;
            pitch = std::stof(argv[6]) * PI / 180;
            yaw = std::stof(argv[7]) * PI / 180;
            if(argv[8] != NULL){
                constraint = std::atoi(argv[8]);
            }
        }
    }
    else
    {
        ROS_INFO("Please enter arguments");
        return 0;
    }

    // BEGIN_TUTORIAL

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP = "lbr_arm";

    // The :move_group_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setMaxAccelerationScalingFactor(0.9);
    move_group.setMaxVelocityScalingFactor(0.9);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group =
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
    //ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    //   ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    ros::Rate rate(1); // 10 hz
    std::vector<double> joint_group_positions;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // ros::Rate rate(10.0);

    tf2::Vector3 object_position;
    tf2::Quaternion orientation;
    geometry_msgs::TransformStamped world2base_footprint;

    if (mode)
    {
        while (node_handle.ok())
        {
            try
            {
                world2base_footprint = tfBuffer.lookupTransform("base_footprint", "map", ros::Time(0));
                break;
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }
        tf2::Transform tf2_world2base_footprint;

        tf2::fromMsg(world2base_footprint.transform, tf2_world2base_footprint);
        object_position = tf2_world2base_footprint * object_position;


        get_model_msg.request.relative_entity_name = "world";
        get_model_msg.request.model_name = object_name;
        orientation.setRPY(0, PI, 0);
        if (get_model_srv.call(get_model_msg))
        {

            object_position.setX(get_model_msg.response.pose.position.x);
            object_position.setY(get_model_msg.response.pose.position.y);
            object_position.setZ(get_model_msg.response.pose.position.z + offest_z_ee_to_palm + offset);

            ROS_INFO("%s found at X: %f Y: %f Z: %f", object_name.c_str(),
                     double(object_position.getX()),
                     double(object_position.getY()),
                     double(object_position.getZ() - offest_z_ee_to_palm - offset));
        }
        else
        {
            ROS_ERROR("Filed to call get_model state gazebo srv :(");
        }
    }
    else
    {
        object_position.setX(x);
        object_position.setY(y);
        object_position.setZ(z + offest_z_ee_to_palm);
        orientation.setRPY(roll, pitch, yaw);
    }
    //tf2::Vector3 object_position(x,y,z + offest_z_ee_to_palm );
    //ROS_INFO("Before Z: %f", z + offest_z_ee_to_palm);

    x = object_position.getX();
    y = object_position.getY();
    z = object_position.getZ();

    ROS_INFO("Respective of base_footprint  X: %f Y: %f, Z: %f", x, y, z);

    if (constraint)
    {
        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
        const Eigen::Affine3d link_7_tf = current_state->getGlobalLinkTransform("link_7");
        auto traslation = link_7_tf.translation();
        const Eigen::Matrix3d rotation = link_7_tf.rotation();

        Eigen::Quaterniond q(rotation);

        std::cout << "Traslation:" << std::endl;
        std::cout << traslation << std::endl;

        std::cout << "Rotation" << std::endl;
        std::cout << rotation << std::endl;

        std::cout << "Quaternion" << std::endl;
        std::cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;

        // Constraints
        move_group.setPlanningTime(20.0); // Give enogh time to find soultuon when constraints
        moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = "link_7";
        ocm.header.frame_id = "base_footprint";

        ocm.orientation.x = q.x();
        ocm.orientation.y = q.y();
        ocm.orientation.z = q.z();
        ocm.orientation.w = q.w();

        ocm.absolute_x_axis_tolerance = 0.15;
        ocm.absolute_y_axis_tolerance = 0.15;
        ocm.absolute_z_axis_tolerance = 3.2;
        ocm.weight = 100.0;

        moveit_msgs::Constraints test_constraints;
        test_constraints.orientation_constraints.push_back(ocm);

        move_group.setPathConstraints(test_constraints);
    }

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation = tf2::toMsg(orientation);
    target_pose1.position.x = x;
    target_pose1.position.y = y;
    target_pose1.position.z = z;

    move_group.setPoseTarget(target_pose1);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.

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
    
    move_group.clearPathConstraints();

    ROS_INFO_STREAM("-------------Final pose reached!");

    ros::shutdown();
    
    return 0;
}