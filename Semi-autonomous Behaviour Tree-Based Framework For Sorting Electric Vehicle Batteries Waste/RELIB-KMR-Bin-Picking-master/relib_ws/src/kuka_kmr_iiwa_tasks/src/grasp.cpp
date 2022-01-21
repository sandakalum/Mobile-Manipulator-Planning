/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan, Ridhwan Luthra*/
/*Refracted by: Hector Cruz ERLB @University of Birmingham*/
// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <iterator>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <kuka_msgs/ObjectPose.h>
#include <kuka_msgs/Place.h>
#include <geometry_msgs/PoseStamped.h>

class Grasp
{

private:
    //ros::NodeHandle nh_;
    ros::Subscriber number_subscriber;
    ros::ServiceServer grasp_server;
    ros::ServiceServer place_server;
    ros::ServiceServer reachability_server;
    ros::ServiceClient vg10_trigger;
    ros::ServiceClient path_recoder_server;

    std::map<int, geometry_msgs::PoseStamped> bins;
    geometry_msgs::PoseStamped bin_pose;
    const std::string GLOBAL_FRAME = "base_footprint";
    const float offest_z_ee_to_palm = 0.246;
    const float TORAD = M_PI / 180;

public:
    const std::string PLANNING_GROUP = "lbr_arm";
    std::vector<std::string> objects_id = {"object", "table2"};

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

public:
    Grasp(ros::NodeHandle *nh_)
    {

        grasp_server = nh_->advertiseService("grasp", &Grasp::grasp_execution, this);
        // place_server = nh_->advertiseService("place", &Grasp::place_execution, this);
        place_server = nh_->advertiseService("place_2", &Grasp::place_executionTwo, this);

        reachability_server = nh_->advertiseService("is_reachable", &Grasp::is_reachable, this);
        vg10_trigger = nh_->serviceClient<std_srvs::SetBool>("vg10_actioner");

        path_recoder_server = nh_->serviceClient<std_srvs::SetBool>("init_path_recorder");

        bin_pose.header.frame_id = GLOBAL_FRAME;
        bin_pose.pose.position.x = -0.15;
        bin_pose.pose.position.y = 0.1;
        bin_pose.pose.position.z = 0.8;
        bins.insert(std::pair<int, geometry_msgs::PoseStamped>(1, bin_pose));
    }
    ~Grasp() {}

    bool is_reachable(kuka_msgs::ObjectPose::Request &req, kuka_msgs::ObjectPose::Response &res)
    {
        // Kinematically there are more chances of grasping if the object is in the left side of the lbr base
        if (req.obj_pose.pose.position.y >= 0.08 && req.obj_pose.pose.position.y <= 0.28 && req.obj_pose.pose.position.x <= 1.1)
        {
            this->planning_scene_interface.removeCollisionObjects(objects_id);
            this->move_group.clearPathConstraints();
            geometry_msgs::PoseStamped target;
            target = req.obj_pose;
            target.pose.position.z += 0.246;
            target.header.frame_id = "base_footprint";
            tf2::Quaternion orientation;
            orientation.setRPY(0, M_PI, 0);
            target.pose.orientation = tf2::toMsg(orientation);
            this->move_group.setPlanningTime(10.0);
            ROS_INFO("Checking reachability for %f %f %f", target.pose.position.x, target.pose.position.y, target.pose.position.z);
            this->move_group.setPoseTarget(target);
            bool success = (this->move_group.plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            res.status = success ? 1 : 0;
            this->move_group.clearPoseTargets();
        }
        else
        {
            res.status = 0;
        }

        return true;
    }
    bool grasp_execution(kuka_msgs::ObjectPose::Request &req, kuka_msgs::ObjectPose::Response &res)
    {
        this->addCollisionObjects();
        ros::WallDuration(0.5).sleep();
        this->move_group.setPlanningTime(3.0);
        this->move_group.setMaxAccelerationScalingFactor(0.2);
        this->move_group.setMaxVelocityScalingFactor(0.5);
        ros::service::waitForService("vg10_actioner", 2);
        std_srvs::SetBool activate;
        activate.request.data = true;
        if (vg10_trigger.call(activate))
        {
            ROS_INFO_STREAM("VG10 activated...");
        }

        int trials = 10;
        int counter = 0;

        move_group.allowReplanning(true);
        
        ROS_INFO("Plannig grasping for: %f %f %f", req.obj_pose.pose.position.x,
                                                    req.obj_pose.pose.position.y, req.obj_pose.pose.position.z );

        std_srvs::SetBool srv;
        srv.request.data = 1;
        if (path_recoder_server.call(srv))
        {
            ROS_INFO_STREAM("Recorder Activated");
        }
        moveit::planning_interface::MoveItErrorCode log = pick(req.obj_pose);
        if (log == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
            res.status = 1;
            this->move_group.setPlanningTime(60.0);
            tf2::Quaternion orientation;
            orientation.setRPY(0.0, 180 * TORAD, 0.0);
            ROS_INFO("Putting object to custom pose");
            moveit_msgs::OrientationConstraint ocm;
            ocm.link_name = "link_7";
            ocm.header.frame_id = "base_footprint";
            ocm.orientation = tf2::toMsg(orientation);

            ocm.absolute_x_axis_tolerance = 0.1;
            ocm.absolute_y_axis_tolerance = 0.1;
            ocm.absolute_z_axis_tolerance = 3.1;
            ocm.weight = 1.0;
            moveit_msgs::Constraints test_constraints;
            test_constraints.orientation_constraints.push_back(ocm);

            moveit_msgs::JointConstraint joint_constraint;
            auto joint_a4_current_value = current_state->getJointPositions("joint_a4");
            auto joint_a5_current_value = current_state->getJointPositions("joint_a5");
            joint_constraint.joint_name = "joint_a5";
            joint_constraint.position = *joint_a5_current_value;
            joint_constraint.tolerance_above = 0.1;
            joint_constraint.tolerance_below = 0.1;
            joint_constraint.weight = 1.1;
            test_constraints.joint_constraints.push_back(joint_constraint);

            // joint_constraint.

            move_group.setPathConstraints(test_constraints);

            const Eigen::Affine3d link_7_tf = current_state->getGlobalLinkTransform("link_7");
            auto translation = link_7_tf.translation();
            // const Eigen::Matrix3d rotation = link_7_tf.rotation();

            geometry_msgs::PoseStamped target_pose1;
            target_pose1.header.frame_id = "base_footprint";
            target_pose1.pose.orientation = tf2::toMsg(orientation);
            target_pose1.pose.position.x = 0.85;
            target_pose1.pose.position.y = 0.0;
            target_pose1.pose.position.z = translation[2];
            ROS_INFO(" ");
            ROS_INFO("Going from: %f %f %f ",translation[0], translation[1], translation[2]);
            ROS_INFO("To... from: %f %f %f ",target_pose1.pose.position.x, target_pose1.pose.position.y,target_pose1.pose.position.z);

            this->move_group.setPoseTarget(target_pose1);
            this->move_group.move();

            const Eigen::Affine3d link_7_tf_2 = move_group.getCurrentState()->getGlobalLinkTransform("link_7");
            auto translation_2 = link_7_tf.translation();
            ROS_INFO("GT... %f %f %f ",translation_2[0], translation_2[1], translation_2[2]);
            ROS_INFO(" ");
        }
        else
        {
            res.status = 0;
            this->move_group.detachObject("object");
        }
        srv.request.data = 0;
        if (path_recoder_server.call(srv))
        {
            ROS_INFO_STREAM("Recorder Deactivated");
        }
        ros::WallDuration(1.0).sleep();
        this->planning_scene_interface.removeCollisionObjects(objects_id);
        this->move_group.clearPoseTargets();
        this->move_group.clearPathConstraints();
        this->move_group.clearTrajectoryConstraints();

        ROS_INFO_STREAM("Objects cleared");

        return true;
    }

    bool place_executionTwo(kuka_msgs::Place::Request &req, kuka_msgs::Place::Response &res)
    {
        this->move_group.setPlanningTime(50.0);
        this->move_group.setMaxAccelerationScalingFactor(0.3);
        this->move_group.setMaxVelocityScalingFactor(0.6);
        if (req.bin != 0)
        {
        }
        else if (req.place_pose.pose.position.x != 0)
        {
            tf2::Quaternion orientation;
            orientation.setRPY(0, 180 * TORAD, 0.0);
            ROS_INFO("Putting object to custom pose");
            moveit_msgs::OrientationConstraint ocm;
            ocm.link_name = "link_7";
            ocm.header.frame_id = "base_footprint";
            ocm.orientation = tf2::toMsg(orientation);
            // ocm.orientation.x = 0.0;
            // ocm.orientation.y = 1.0;
            // ocm.orientation.z = 0.0;
            // ocm.orientation.w = 0.0;
            ocm.absolute_x_axis_tolerance = 0.15;
            ocm.absolute_y_axis_tolerance = 0.15;
            ocm.absolute_z_axis_tolerance = 3.0;
            ocm.weight = 100.0;
            moveit_msgs::Constraints test_constraints;
            test_constraints.orientation_constraints.push_back(ocm);
            move_group.setPathConstraints(test_constraints);
            // move_group.clearPathConstraints();

            moveit::core::RobotStatePtr current_state = this->move_group.getCurrentState();
            geometry_msgs::PoseStamped target_pose1;

            target_pose1.header.frame_id = "base_footprint";
            target_pose1.pose.orientation = tf2::toMsg(orientation);
            target_pose1.pose.position.x = req.place_pose.pose.position.x;
            target_pose1.pose.position.y = req.place_pose.pose.position.y;
            target_pose1.pose.position.z = req.place_pose.pose.position.z + offest_z_ee_to_palm;

            ROS_INFO("Placing at: %f %f %f", target_pose1.pose.position.x, target_pose1.pose.position.y, target_pose1.pose.position.z - offest_z_ee_to_palm);
            ROS_INFO("orientation: %f %f %f %f", target_pose1.pose.orientation.x, target_pose1.pose.orientation.y, target_pose1.pose.orientation.z, target_pose1.pose.orientation.w);

            this->move_group.setPoseTarget(target_pose1);

            // bool success = (this->move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            moveit::planning_interface::MoveItErrorCode log = this->move_group.move();
            if (log == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                res.status = 1;
                move_group.detachObject("object");
                move_group.clearPathConstraints();
                // target_pose1.pose.position.z-= 0.05;
                // this->move_group.setPoseTarget(target_pose1);
                // this->move_group.move();
            }
            else
            {
                res.status = 0;
                move_group.detachObject("object");
                move_group.clearPathConstraints();
            }

            ros::service::waitForService("vg10_actioner", 2);
            std_srvs::SetBool activate;
            activate.request.data = false;
            if (vg10_trigger.call(activate))
            {
                ROS_INFO_STREAM("VG10 deactivated...");
            }
            // Going R units along Z+ to elevate the arm
            // this->move_group.setPathConstraints(test_constraints);

            target_pose1.pose.position.z += 0.20;
            this->move_group.setPoseTarget(target_pose1);
            // ROS_INFO("Going up?????????????????????");

            ros::Duration(1.0).sleep();
            log = this->move_group.move();

            if (log == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                res.status = 1;
            }
            else
            {
                res.status = 0;
            }
            move_group.clearPathConstraints();
            this->move_group.clearPoseTargets();
        }

        return true;
    }
    // bool place_execution(kuka_msgs::Place::Request& req, kuka_msgs::Place::Response& res ){
    //     this->move_group.setPlanningTime(2.0);
    //     if (req.bin != 0){
    //         // Call place from MoveGroup Interface
    //         geometry_msgs::PoseStamped pose_temp = bins.at(int(req.bin));
    //         float x = pose_temp.pose.position.x;
    //         float y = pose_temp.pose.position.y;
    //         float z = pose_temp.pose.position.z;

    //         ROS_INFO("Putting object to bin %i in: {%f, %f, %f}",req.bin,x,y,z);

    //         moveit::planning_interface::MoveItErrorCode  log = place(pose_temp);
    //         if (log == moveit::planning_interface::MoveItErrorCode::SUCCESS){
    //             res.status = 1;

    //         }else{
    //             res.status = 0;
    //             move_group.detachObject("object");
    //         }
    //     }else if(req.place_pose.pose.position.x != 0){
    //         ROS_INFO("Putting object to custom pose");

    //         moveit::planning_interface::MoveItErrorCode  log = place(req.place_pose);
    //         if (log == moveit::planning_interface::MoveItErrorCode::SUCCESS){
    //             res.status = 1;
    //         }else{
    //             res.status = 0;
    //             move_group.detachObject("object");
    //         }
    //     }
    //     ros::service::waitForService("vg10_actioner",2);
    //     std_srvs::SetBool activate;
    //     activate.request.data = false;
    //     if(vg10_trigger.call(activate)){
    //         ROS_INFO_STREAM("VG10 deactivated...");
    //     }
    //     return true;
    // }

    void addCollisionObjects()
    {
        // Create vector to hold 3 collision objects.
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.resize(3);

        // Add the first table where the cube will originally be kept.
        collision_objects[0].id = "table1";
        collision_objects[0].header.frame_id = "map";

        /* Define the primitive and its dimensions. */
        collision_objects[0].primitives.resize(1);
        collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[0].primitives[0].dimensions.resize(3);
        collision_objects[0].primitives[0].dimensions[0] = 1.0;
        collision_objects[0].primitives[0].dimensions[1] = 1.9;
        collision_objects[0].primitives[0].dimensions[2] = 0.10;

        /* Define the pose of the table. */
        collision_objects[0].primitive_poses.resize(1);
        collision_objects[0].primitive_poses[0].position.x = 1.3;
        collision_objects[0].primitive_poses[0].position.y = 0.0;
        collision_objects[0].primitive_poses[0].position.z = 0.751;
        // END_SUB_TUTORIAL

        collision_objects[0].operation = collision_objects[0].ADD;

        // BEGIN_SUB_TUTORIAL table2
        // Add the second table where we will be placing the cube.
        collision_objects[1].id = "table2";
        collision_objects[1].header.frame_id = GLOBAL_FRAME;

        /* Define the primitive and its dimensions. */
        collision_objects[1].primitives.resize(1);
        collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
        collision_objects[1].primitives[0].dimensions.resize(3);
        collision_objects[1].primitives[0].dimensions[0] = 0.4;
        collision_objects[1].primitives[0].dimensions[1] = 0.2;
        collision_objects[1].primitives[0].dimensions[2] = 0.7;

        /* Define the pose of the table. */
        collision_objects[1].primitive_poses.resize(1);
        collision_objects[1].primitive_poses[0].position.x = 10.0;
        collision_objects[1].primitive_poses[0].position.y = -0.7;
        collision_objects[1].primitive_poses[0].position.z = 0.7 / 2;
        // END_SUB_TUTORIAL

        collision_objects[1].operation = collision_objects[1].ADD;

        // BEGIN_SUB_TUTORIAL object
        // Define the object that we will be manipulating
        collision_objects[2].header.frame_id = GLOBAL_FRAME;
        collision_objects[2].id = "object";

        /* Define the primitive and its dimensions. */
        collision_objects[2].primitives.resize(1);
        collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
        collision_objects[2].primitives[0].dimensions.resize(3);
        collision_objects[2].primitives[0].dimensions[0] = 0.0222;
        collision_objects[2].primitives[0].dimensions[1] = 0.03149;
        collision_objects[2].primitives[0].dimensions[2] = 0.0035;

        /* Define the pose of the object. */
        collision_objects[2].primitive_poses.resize(1);
        collision_objects[2].primitive_poses[0].position.x = 2.2;
        collision_objects[2].primitive_poses[0].position.y = 0.4;
        collision_objects[2].primitive_poses[0].position.z = 1.0;
        // END_SUB_TUTORIAL

        //collision_objects[2].operation = collision_objects[2].ADD;

        this->planning_scene_interface.applyCollisionObjects(collision_objects);
    }

    moveit::planning_interface::MoveItErrorCode pick(geometry_msgs::PoseStamped &obj_pose)
    {
        // BEGIN_SUB_TUTORIAL pick1
        // Create a vector of grasps to be attempted, currently only creating single grasp.
        // This is essentially useful when using a grasp generator to generate and test multiple grasps.
        std::vector<moveit_msgs::Grasp> grasps;
        grasps.resize(1);

        // Setting grasp pose
        // ++++++++++++++++++++++
        // This is the pose of panda_link8. |br|
        // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
        // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
        // transform from `"panda_link8"` to the palm of the end effector.
        // grasps[0].grasp_pose.header.frame_id = GLOBAL_FRAME;
        // tf2::Quaternion orientation;
        // orientation.setRPY(0.0 , M_PI, 0.0);
        // grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
        // grasps[0].grasp_pose.pose.position.x = 0.8;
        // grasps[0].grasp_pose.pose.position.y = 0.4;
        // grasps[0].grasp_pose.pose.position.z =  height + 0.256;

        grasps[0].grasp_pose = obj_pose;
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, M_PI, 0.0);
        grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
        grasps[0].grasp_pose.pose.position.z += 0.256; //0.256 becasue the offest between link_7 and the palm of VG10 gripper
        ROS_INFO("Height grasp Z: %f", grasps[0].grasp_pose.pose.position.z - 0.256);
        // Setting pre-grasp approach
        // ++++++++++++++++++++++++++
        /* Defined with respect to frame_id */
        //grasps[0].pre_grasp_approach.direction.header.frame_id = GLOBAL_FRAME;
        grasps[0].pre_grasp_approach.direction.header.frame_id = obj_pose.header.frame_id;

        /* Direction is set as positive x axis */
        // grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
        grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
        grasps[0].pre_grasp_approach.min_distance = 0.15;   
        grasps[0].pre_grasp_approach.desired_distance = 0.16;

        // Setting post-grasp retreat
        // ++++++++++++++++++++++++++
        /* Defined with respect to frame_id */
        grasps[0].post_grasp_retreat.direction.header.frame_id = obj_pose.header.frame_id;
        /* Direction is set as positive z axis */
        grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
        grasps[0].post_grasp_retreat.min_distance = 0.21;
        grasps[0].post_grasp_retreat.desired_distance = 0.22;

        //Creating more graspings by modifying just Yaw variantes
        // for (int i = 1; i <= 3; i++)
        // {
        //     grasps[i] = grasps[0];
        //     orientation.setRPY(0.0, M_PI, (M_PI/2)*i);
        //     grasps[i].grasp_pose.pose.orientation = tf2::toMsg(orientation);
        // }

        // Setting posture of eef before grasp
        // +++++++++++++++++++++++++++++++++++
        this->openGripper(grasps[0].pre_grasp_posture);
        // END_SUB_TUTORIAL

        // BEGIN_SUB_TUTORIAL pick2
        // Setting posture of eef during grasp
        // +++++++++++++++++++++++++++++++++++
        this->closedGripper(grasps[0].grasp_posture);
        // END_SUB_TUTORIAL

        // BEGIN_SUB_TUTORIAL pick3
        // Set support surface as table1.
        move_group.setSupportSurfaceName("table1");
        // Call pick to pick up the object using the grasps given
        moveit::planning_interface::MoveItErrorCode status = move_group.pick("object", grasps);

        return status;
    }

    moveit::planning_interface::MoveItErrorCode place(geometry_msgs::PoseStamped &_place_pose)
    {
        // BEGIN_SUB_TUTORIAL place
        // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
        // location in verbose mode." This is a known issue. |br|
        // |br|
        // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
        // a single place location.
        std::vector<moveit_msgs::PlaceLocation> place_location;
        place_location.resize(1);

        // Setting place location pose
        // +++++++++++++++++++++++++++
        // place_location[0].place_pose.header.frame_id = GLOBAL_FRAME;
        // tf2::Quaternion orientation;
        // orientation.setRPY(0.0, 0.0, M_PI/2);
        // place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

        /* For place location, we set the value to the exact location of the center of the object. */
        // place_location[0].place_pose.pose.position.x = -0.15;
        // place_location[0].place_pose.pose.position.y = -0.1;
        // place_location[0].place_pose.pose.position.z = 0.79; //0.75
        place_location[0].place_pose.header.frame_id = GLOBAL_FRAME;
        place_location[0].place_pose = _place_pose;
        ROS_INFO("Place to X: %f, Y: %f, Z: %f", _place_pose.pose.position.x, _place_pose.pose.position.y, _place_pose.pose.position.z);
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, M_PI / 2);
        place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

        // Setting pre-place approach
        // ++++++++++++++++++++++++++
        /* Defined with respect to frame_id */
        place_location[0].pre_place_approach.direction.header.frame_id = GLOBAL_FRAME;
        /* Direction is set as negative z axis */
        place_location[0].pre_place_approach.direction.vector.z = -1.0;
        place_location[0].pre_place_approach.min_distance = 0.0001;
        place_location[0].pre_place_approach.desired_distance = 0.01;

        // Setting post-grasp retreat
        // ++++++++++++++++++++++++++
        /* Defined with respect to frame_id */
        place_location[0].post_place_retreat.direction.header.frame_id = GLOBAL_FRAME;
        /* Direction is set as negative y axis */
        place_location[0].post_place_retreat.direction.vector.z = 1.0;
        place_location[0].post_place_retreat.min_distance = 0.0001;
        place_location[0].post_place_retreat.desired_distance = 0.01;

        // Setting posture of eef after placing object
        // +++++++++++++++++++++++++++++++++++++++++++
        /* Similar to the pick case */
        openGripper(place_location[0].post_place_posture);

        // Set support surface as table2.
        move_group.setSupportSurfaceName("table2");
        // Call place to place the object using the place locations given.
        return move_group.place("object", place_location);
    }

    void openGripper(trajectory_msgs::JointTrajectory &posture)
    {
        // BEGIN_SUB_TUTORIAL open_gripper
        /* Add both finger joints of panda robot. */
        // if(vaccum_off.call(srv) )
        //     ROS_INFO_STREAM("Vaccum OFF");
        posture.joint_names.resize(1); //was2
        posture.joint_names[0] = "joint_a7";
        // posture.joint_names[1] = "panda_finger_joint2";

        /* Set them as open, wide enough for the object to fit. */
        posture.points.resize(1);
        posture.points[0].positions.resize(1); //was 2
        posture.points[0].positions[0] = -131 * M_PI / 180;
        // posture.points[0].positions[1] = 0.04;
        posture.points[0].time_from_start = ros::Duration(0.5);
        // END_SUB_TUTORIAL
    }

    void closedGripper(trajectory_msgs::JointTrajectory &posture)
    {
        // BEGIN_SUB_TUTORIAL closed_gripper
        /* Add both finger joints of panda robot. */
        // if(vaccum_on.call(srv) )
        //     ROS_INFO_STREAM("Vaccum ON");
        posture.joint_names.resize(1);
        posture.joint_names[0] = "joint_a7";
        // posture.joint_names[1] = "panda_finger_joint2";

        /* Set them as closed. */
        posture.points.resize(1);
        posture.points[0].positions.resize(1);
        posture.points[0].positions[0] = -131 * M_PI / 180;
        // posture.points[0].positions[1] = 0.00;
        posture.points[0].time_from_start = ros::Duration(0.5);
        // END_SUB_TUTORIAL
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_mpl_interface");
    ros::NodeHandle nh;

    // Creating grasp object
    ros::AsyncSpinner spinner(2);
    spinner.start();

    Grasp grasp_action(&nh);

    ROS_INFO_STREAM("Arm Motion Planning node ready!");
    ros::waitForShutdown();
    return 0;
}
