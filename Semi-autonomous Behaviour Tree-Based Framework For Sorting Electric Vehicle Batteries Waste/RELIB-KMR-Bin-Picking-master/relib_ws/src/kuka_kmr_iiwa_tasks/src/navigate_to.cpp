// C++ Lib
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <iterator>
#include <map>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <math.h>
#include <boost/make_shared.hpp>
#include <bits/stdc++.h>
#include <sstream>

// ROS
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <kuka_msgs/BestWorkstation.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
float x = 0;
float y = 0;
float yaw = 0;
int pose_idx = 1;
char option = 'x';

std::string frame = "map";

int main(int argc, char **argv)
{

    ros::init(argc, argv, "nagivate_to");
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    if (argv[1] != NULL)
    {
        option = *(argv[1]);
        if (option == 'p')
        {
            if (argv[2] != NULL)
                pose_idx = atoi(argv[2]);
        }
        else
        {
            if (argv[2] != NULL)
            {
                x = atof(argv[2]);
            }
            if (argv[3] != NULL)
            {
                y = atof(argv[3]);
            }
            if (argv[4] != NULL)
            {
                yaw = atof(argv[4]) * M_PI / 180;
            }
            if (argv[5] != NULL)
            {
                frame = std::string(argv[5]);
            }
            std::cout << "X: " << x << " Y: " << y << " Yaw: " << yaw << std::endl;
        }
    }

    int number_of_poses = 1;
    std::vector<double> pose_vector_param;
    std::map<int, geometry_msgs::PoseStamped> candidate_poses_dictionary;
    geometry_msgs::PoseStamped temp_pose;

    if (ros::param::get("number_of_poses", number_of_poses))
    {
        for (int i = 1; i <= number_of_poses; i++)
        {
            if (ros::param::get("possible_poses/pose_" + std::to_string(i), pose_vector_param))
            {
                // Extracting candidate poses and adding them to a dictionary
                temp_pose.pose.position.x = pose_vector_param[0];
                temp_pose.pose.position.y = pose_vector_param[1];
                temp_pose.pose.position.z = pose_vector_param[2];
                temp_pose.pose.orientation.x = pose_vector_param[3];
                temp_pose.pose.orientation.y = pose_vector_param[4];
                temp_pose.pose.orientation.z = pose_vector_param[5];
                temp_pose.pose.orientation.w = pose_vector_param[6];
                candidate_poses_dictionary.insert(std::pair<int, geometry_msgs::PoseStamped>(i, temp_pose));
            }
            else
            {
                ROS_ERROR("Failed to get workstation %i parameters from server.", i);
            }
        }
        ROS_INFO("All candidate poses captured!");
    }
    else
    {
        ROS_ERROR("Failed to get 'number_of_poses' param from server.");
    }

    tf::Vector3 odom_basis(x, y, 0);
    tf::Vector3 map_coord;
    MoveBaseClient ac("move_base", true);
    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;

    ros::Rate rate(10.0);
    if (option != 'p')
    {
        if (frame != "map")
        {
            geometry_msgs::TransformStamped transformStamped;
            while (nh.ok())
            {
                try
                {
                    transformStamped = tfBuffer.lookupTransform("map", "odom", ros::Time(0));
                    break;
                }
                catch (tf2::TransformException &ex)
                {
                    ROS_WARN("%s", ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }
                rate.sleep();
            }
            tf::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);
            tf::Matrix3x3 m(q);
            tf::Vector3 translation(
                transformStamped.transform.translation.x,
                transformStamped.transform.translation.y,
                transformStamped.transform.translation.z);

            tf::Transform odom2map(m, translation);
            map_coord = odom2map * odom_basis;
            goal.target_pose.pose.position.x = map_coord.getX();
            goal.target_pose.pose.position.y = map_coord.getY();
        }
        else
        {
            goal.target_pose.pose.position.x = x;
            goal.target_pose.pose.position.y = y;
        }

        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, yaw);
        goal.target_pose.pose.orientation.w = 1;
        goal.target_pose.pose.orientation = tf2::toMsg(orientation);
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.z = 0;
    }
    else
    {
        x = candidate_poses_dictionary[pose_idx].pose.position.x;
        y = candidate_poses_dictionary[pose_idx].pose.position.y;
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.position.z = 0;
        goal.target_pose.pose.orientation = candidate_poses_dictionary[pose_idx].pose.orientation;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
    }

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to (%f, %f ) in the %s frame", x, y, frame.c_str());
    else
        ROS_INFO("The base failed to move to (%f, %f ) in the %s frame for some reason", x, y, frame.c_str());

    return 0;
}