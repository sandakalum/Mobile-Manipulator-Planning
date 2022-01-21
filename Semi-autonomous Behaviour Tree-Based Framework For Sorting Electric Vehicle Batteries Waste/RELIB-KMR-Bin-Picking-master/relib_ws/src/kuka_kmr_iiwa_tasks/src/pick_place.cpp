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

// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <vector>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#define PI 3.141516
int N_SERVERS = 1;

static const int gripper_depth = 0.11788;


// void vaccum(bool action, ros::ServiceClient& on_server , ros::ServiceClient& off_server, std_srvs::Empty& _srv){
//     if(action){
//         if(on_server.call(_srv) ){
//         ROS_INFO_STREAM("Vaccum ON");
//         }
//         else{
//             ROS_ERROR("For some reason the vaccum was not turned ON");
//         }
//     }else{
//         if(off_server.call(_srv) ){
//             ROS_INFO_STREAM("Vaccum OFF");
//         }
//         else{
//             ROS_ERROR("For some reason the vaccum was not turned OFF");
//         }
//     }
// }

void vaccum(bool action, std::vector<ros::ServiceClient>& on_server , std::vector<ros::ServiceClient>& off_server, std_srvs::Empty& _srv){
    if(action){
        for(int j=0; j<N_SERVERS;j++){
            if(on_server[j].call(_srv) ){
                ROS_INFO_STREAM("Vaccum "<< j<<" ON");
            }
            else{
                // ROS_ERROR("For some reason the vaccum %i was not turned ON",j);
            }
        }
    }else{
        for(int j=0; j<N_SERVERS;j++){
            if(off_server[j].call(_srv) ){
                ROS_INFO_STREAM("Vaccum "<< j<<" OFF");
            }
            else{
                // ROS_ERROR("For some reason the vaccum %i was not turned OFF",j);
            }
        }
    }
}

void openGripper(trajectory_msgs::JointTrajectory& posture)
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
    posture.points[0].positions[0] = -131*M_PI/180;
    // posture.points[0].positions[1] = 0.04;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
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
    posture.points[0].positions[0] = -131*M_PI/180;
    // posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group, float& height)
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
    grasps[0].grasp_pose.header.frame_id = "base_footprint";
    tf2::Quaternion orientation;
    orientation.setRPY(0.0 , M_PI, 0.0);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.8;
    grasps[0].grasp_pose.pose.position.y = 0.4;
    grasps[0].grasp_pose.pose.position.z =  height + 0.256; //1.0461

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_footprint";
    /* Direction is set as positive x axis */
    // grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.001;
    grasps[0].pre_grasp_approach.desired_distance = 0.15;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_footprint";
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.000001;
    grasps[0].post_grasp_retreat.desired_distance = 0.15;

    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    openGripper(grasps[0].pre_grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick2
    // Setting posture of eef during grasp
    // +++++++++++++++++++++++++++++++++++
    closedGripper(grasps[0].grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick3
    // Set support surface as table1.
    move_group.setSupportSurfaceName("table1");
    // Call pick to pick up the object using the grasps given
    move_group.pick("object", grasps);
    

    // END_SUB_TUTORIAL
}

moveit::planning_interface::MoveItErrorCode place(moveit::planning_interface::MoveGroupInterface& move_group, float& height_object)
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
    place_location[0].place_pose.header.frame_id = "base_footprint";
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, M_PI/2);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    /* For place location, we set the value to the exact location of the center of the object. */
    place_location[0].place_pose.pose.position.x = -0.15;
    place_location[0].place_pose.pose.position.y = -0.1;
    place_location[0].place_pose.pose.position.z = height_object; //0.75

    // Setting pre-place approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].pre_place_approach.direction.header.frame_id = "base_footprint";
    /* Direction is set as negative z axis */
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.0001;
    place_location[0].pre_place_approach.desired_distance = 0.01;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].post_place_retreat.direction.header.frame_id = "base_footprint";
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
    // END_SUB_TUTORIAL
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    // BEGIN_SUB_TUTORIAL table1
    //
    // Creating Environment
    // ^^^^^^^^^^^^^^^^^^^^
    // Create vector to hold 3 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_footprint";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.7;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.8;
    collision_objects[0].primitive_poses[0].position.y = 0.4;
    collision_objects[0].primitive_poses[0].position.z = 0.7/2;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

    // BEGIN_SUB_TUTORIAL table2
    // Add the second table where we will be placing the cube.
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "base_footprint";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.4;
    collision_objects[1].primitives[0].dimensions[1] = 0.2;
    collision_objects[1].primitives[0].dimensions[2] = 0.7;

    /* Define the pose of the table. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = -0.7;
    collision_objects[1].primitive_poses[0].position.z = 0.7/2;
    // END_SUB_TUTORIAL

    collision_objects[1].operation = collision_objects[1].ADD;

    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    collision_objects[2].header.frame_id = "base_footprint";
    collision_objects[2].id = "object";

    /* Define the primitive and its dimensions. */
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.222;
    collision_objects[2].primitives[0].dimensions[1] = 0.3149;
    collision_objects[2].primitives[0].dimensions[2] = 0.035;

    /* Define the pose of the object. */
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.8;
    collision_objects[2].primitive_poses[0].position.y = 0.4;
    collision_objects[2].primitive_poses[0].position.z = 0.7175;
    // END_SUB_TUTORIAL

    //collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_place");
    ros::NodeHandle nh;
    std::vector<ros::ServiceClient> on_servers_clients;
    std::vector<ros::ServiceClient> off_servers_clients;

    std_srvs::Empty srv;
    float pick_height = atof(argv[1]);
    float place_height = atof(argv[2]);
    int put = atoi(argv[3]);

    if(argv[4]!= NULL)
        N_SERVERS = atoi(argv[4]);    
    
    for(int i = 0; i<N_SERVERS; i++){
        on_servers_clients.push_back(nh.serviceClient<std_srvs::Empty>("/vaccum_" + std::to_string(i) + "/on"));
    }
    for(int i = 0; i<N_SERVERS; i++){
        off_servers_clients.push_back(nh.serviceClient<std_srvs::Empty>("/vaccum_" + std::to_string(i) + "/off"));
    }

    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "lbr_arm";

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setPlanningTime(2.0);
    move_group.setMaxAccelerationScalingFactor(0.4);
    move_group.setMaxVelocityScalingFactor(0.5);
    
    std::cout<<"EE NAME: "<<move_group.getEndEffector()<<std::endl;
    std::vector<std::string> objects_id = {"object","table1","table2"};
    addCollisionObjects(planning_scene_interface);

    // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();

    //vaccum(0,vaccum_on, vaccum_off, srv);
    vaccum(1,on_servers_clients, off_servers_clients, srv);
    ros::WallDuration(0.5).sleep();
    pick(move_group,pick_height);

    ros::WallDuration(3.0).sleep();


    // moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    // geometry_msgs::PoseStamped eff_current_pose = move_group.getCurrentPose("link_7");
    // eff_current_pose.pose.orientation = tf2::toMsg(orientation);
    // // eff_current_pose.pose.position.x = 0.8;
    // // eff_current_pose.pose.position.y = 0.4; 
    // eff_current_pose.pose.position.z = eff_current_pose.pose.position.z + 0.15;
    // move_group.setPoseTarget(eff_current_pose.pose);
    // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("Moving Away", "Moving away with object %s", success ? "SUCCESFULL" : "FAILED");
    // move_group.move();


    if (put == 1){
        move_group.setMaxAccelerationScalingFactor(0.3);
        move_group.setMaxVelocityScalingFactor(0.3);
        int trials = 10;
        int counter = 0;
        while ((place(move_group, place_height)!= moveit::planning_interface::MoveItErrorCode::SUCCESS) && (counter<trials)){
            ROS_INFO_STREAM("RETRYING...");
            ros::WallDuration(0.5).sleep();
            counter ++;
        }; 
        ros::WallDuration(0.3).sleep();
        vaccum(0,on_servers_clients, off_servers_clients, srv);
    }

    // vaccum(0,vaccum_on, vaccum_off, srv);
    ros::WallDuration(2.0).sleep();
    planning_scene_interface.removeCollisionObjects(objects_id);


    // Getting away of the object after releasing
    // current_state = move_group.getCurrentState();
    // eff_current_pose = move_group.getCurrentPose("link_7");
    // eff_current_pose.pose.position.z = eff_current_pose.pose.position.z + 0.15;
    // move_group.setPoseTarget(eff_current_pose.pose);
    // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("Moving Away", "Moving away from object %s", success ? "SUCCESFULL" : "FAILED");
    // move_group.move();

    ROS_INFO_STREAM("--------------Finished!");
    ros::shutdown();
    return 0;
}
