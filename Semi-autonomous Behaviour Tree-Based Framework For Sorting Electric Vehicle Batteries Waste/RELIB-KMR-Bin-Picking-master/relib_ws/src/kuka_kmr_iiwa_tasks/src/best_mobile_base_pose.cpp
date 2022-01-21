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
#include "ros/ros.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kuka_msgs/BestWorkstation.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

#define SIGNIFICANT_FIGURES 5

class BestMobileBasePose
{

private:
    // ROS
    ros::NodeHandle nh;
    ros::ServiceServer extract_ws_service;
    ros::Subscriber mobile_base_pose_sub;
    ros::Publisher candidate_poses_marker_array_pub;
    ros::Publisher selected_pose_marker_pub;

    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> mobile_base_pose;
    std::ofstream config_file;
    std::vector<double> pose_vector_param;

    std::string candidate_poses_frame_id = "/map";
    std::map<int, geometry_msgs::PoseStamped> candidate_poses_dictionary, lbr_base_candidate_poses_dictionary;
    visualization_msgs::MarkerArray candidate_poses_markers_array;

    std::string config_directory = "/home/hector/RELIB-KMR-Bin-Picking/relib_ws/src/kuka_kmr_iiwa_tasks/config/workstation_poses.yaml";
    int number_of_poses = 1;
    float lbr_base_offset_x = 0;
    float lbr_base_offset_y = 0;
    float lbr_base_offset_z = 0;

    int view_spot_one_id  = 75;
    int view_spot_two_id = 32;

    tf2::Vector3 traslation_base_footprint2lbr_base;


public:
    BestMobileBasePose(ros::NodeHandle *_nh) : nh(*_nh)
    {
        extract_ws_service = _nh->advertiseService("best_pose", &BestMobileBasePose::extractWorkstationCallback, this);
        candidate_poses_marker_array_pub = _nh->advertise<visualization_msgs::MarkerArray>("posible_poses", 10);
        selected_pose_marker_pub = _nh->advertise<visualization_msgs::Marker>("selected_pose", 1);

        extractAndSetParameters();
        //Requered to transform the mobile base workstation poses to lbr base poses
        // xyz="0.365 -0.18 0.66"
        // lbr_base_offset_x = 0.365;
        // lbr_base_offset_y = -0.18;
        // lbr_base_offset_z = 0.66;
        traslation_base_footprint2lbr_base.setValue(0.365, -0.18, 0.66);
    }
    ~BestMobileBasePose()
    {
    }
    void extractAndSetParameters();
    void createMarkerPoses();
    double roundOff(double N, double n);
    void recordPoses();
    double ComputeScore(geometry_msgs::Point &from, geometry_msgs::Point to);
    bool extractWorkstationCallback(kuka_msgs::BestWorkstation::Request &req, kuka_msgs::BestWorkstation::Response &res);
};

void BestMobileBasePose::extractAndSetParameters()
{

    visualization_msgs::Marker candidate_pose;
    candidate_pose.header.frame_id = candidate_poses_frame_id;
    candidate_pose.header.stamp = ros::Time::now();
    candidate_pose.ns = "available_poses";
    candidate_pose.action = visualization_msgs::Marker::ADD;
    candidate_pose.pose.orientation.w = 1.0;
    candidate_pose.type = visualization_msgs::Marker::SPHERE;
    // POINTS markers use x and y scale for width/height respectively
    candidate_pose.scale.x = 0.05;
    candidate_pose.scale.y = 0.05;
    candidate_pose.scale.z = 0.01;
    // Candidate Poses  are yellow
    candidate_pose.color.r = 1.0f;
    candidate_pose.color.g = 1.0f;
    candidate_pose.color.b = 0.0f;
    candidate_pose.color.a = 1.0;

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

                // Creating Makrer for every single candidate pose and adding it to a Marker Array
                candidate_pose.id = i;
                candidate_pose.pose.position = temp_pose.pose.position;
                candidate_poses_markers_array.markers.push_back(candidate_pose);
            }
            else
            {
                ROS_ERROR("Failed to get mobile pose%i parameters from server.", i);
            }

            if (ros::param::get("possible_poses/lbr_base_pose_" + std::to_string(i), pose_vector_param))
            {
                // Extracting candidate poses and adding them to a dictionary
                temp_pose.pose.position.x = pose_vector_param[0];
                temp_pose.pose.position.y = pose_vector_param[1];
                temp_pose.pose.position.z = pose_vector_param[2];
                temp_pose.pose.orientation.x = 0.0;
                temp_pose.pose.orientation.y = 0.0;
                temp_pose.pose.orientation.z = 0.0;
                temp_pose.pose.orientation.w = 1.0;
                lbr_base_candidate_poses_dictionary.insert(std::pair<int, geometry_msgs::PoseStamped>(i, temp_pose));

            }
            else
            {
                ROS_ERROR("Failed to get lbr base pose %i parameters from server.", i);
            }
        }
        ROS_INFO("All candidate poses captured!");
    }
    else
    {
        ROS_ERROR("Failed to get 'number_of_poses' param from server.");
    }
}

double BestMobileBasePose::roundOff(double N, double n)
{
    int h;
    double b, d, e, i, j, m, f;
    b = N;
    // Counting the no. of digits to the left of decimal point
    // in the given no.
    for (i = 0; b >= 1; ++i)
        b = b / 10;

    d = n - i;
    b = N;
    b = b * pow(10, d);
    e = b + 0.5;
    if ((float)e == (float)ceil(b))
    {
        f = (ceil(b));
        h = f - 2;
        if (h % 2 != 0)
        {
            e = e - 1;
        }
    }
    j = std::floor(e);
    m = std::pow(10, d);
    j = j / m;
    return j;
}

void BestMobileBasePose::recordPoses()
{
    char command;
    int pose_number = 1;
    tf2::Vector3 tf2_vec;
    config_file.open(config_directory.c_str());
    if (config_file.is_open())
    {
        config_file << "# Format:  Position / Orientation \n";
        config_file << "# Ws_x : [X, Y, Z, qx, qy, qz, qw ] \n";
        config_file << "possible_poses:\n";
        while (ros::ok())
        {
            ROS_INFO("Press enter to record pose %i or press 'q' to stop and save", pose_number);
       
            std::cin >> command;
            if (command != 'q')
            {
                ROS_INFO("Waiting for new mobile base pose...");
                mobile_base_pose = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", nh);
                if (mobile_base_pose != NULL)
                {
                    config_file << "  pose_" << pose_number << ": ";
                    config_file << "[ ";
                    config_file << roundOff(mobile_base_pose.get()->pose.pose.position.x, SIGNIFICANT_FIGURES) << ", ";
                    config_file << roundOff(mobile_base_pose.get()->pose.pose.position.y, SIGNIFICANT_FIGURES) << ", ";
                    config_file << roundOff(mobile_base_pose.get()->pose.pose.position.z, SIGNIFICANT_FIGURES) << ", ";
                    config_file << roundOff(mobile_base_pose.get()->pose.pose.orientation.x, SIGNIFICANT_FIGURES) << ", ";
                    config_file << roundOff(mobile_base_pose.get()->pose.pose.orientation.y, SIGNIFICANT_FIGURES) << ", ";
                    config_file << roundOff(mobile_base_pose.get()->pose.pose.orientation.z, SIGNIFICANT_FIGURES) << ", ";
                    config_file << roundOff(mobile_base_pose.get()->pose.pose.orientation.w, SIGNIFICANT_FIGURES);
                    config_file << " ] \n";

                    //Convert mobile_base_pose to tf then multyplied by the traslation vector between base_footprint and lbr_base
                    tf2::Transform tf2_world2base_footprint;

                    tf2_world2base_footprint.setOrigin(tf2::Vector3(mobile_base_pose.get()->pose.pose.position.x,
                                                                    mobile_base_pose.get()->pose.pose.position.y,
                                                                    mobile_base_pose.get()->pose.pose.position.z)); 

                    tf2_world2base_footprint.setRotation(tf2::Quaternion(mobile_base_pose.get()->pose.pose.orientation.x,
                                                                         mobile_base_pose.get()->pose.pose.orientation.y,
                                                                         mobile_base_pose.get()->pose.pose.orientation.z,
                                                                         mobile_base_pose.get()->pose.pose.orientation.w));
                    // Convertion 
                    tf2_vec = tf2_world2base_footprint * traslation_base_footprint2lbr_base;

                    config_file << "  lbr_base_pose_" << pose_number << ": ";
                    config_file << "[ ";
                    config_file << roundOff(double(tf2_vec.getX()) , SIGNIFICANT_FIGURES) << ", ";
                    config_file << roundOff(double(tf2_vec.getY()) , SIGNIFICANT_FIGURES) << ", ";
                    config_file << roundOff(double(tf2_vec.getZ()) , SIGNIFICANT_FIGURES);
                    config_file << " ] \n";
                    ROS_INFO("Pose %i saved! ", pose_number);

                    pose_number++;
                }
            }
            else
            {
                config_file << "\n";
                config_file << "number_of_poses: " << pose_number - 1 << "\n";
                config_file.close();
                break;
            }
        }

        ROS_INFO("%i Poses successfully saved at %s", pose_number - 1, config_directory.c_str());
    }
    else
    {
        ROS_WARN("It could not create file at %s", config_directory.c_str());
    }
}
double BestMobileBasePose::ComputeScore(geometry_msgs::Point &from, geometry_msgs::Point to)
{   // Euclidean distance + penalty of angle between points

    float delta_x = to.x - from.x;
    float delta_y = to.y - from.y;
    // float angle_in_degrees =  std::atan2(delta_y, delta_x) * 180 / M_PI;
    return sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + std::exp(std::abs(delta_y));
}

bool BestMobileBasePose::extractWorkstationCallback(kuka_msgs::BestWorkstation::Request &req, kuka_msgs::BestWorkstation::Response &res)
{

    candidate_poses_marker_array_pub.publish(candidate_poses_markers_array);
    int ws_dictionary_size = lbr_base_candidate_poses_dictionary.size();
    std::vector<double> distances_between_ws;
    std::vector<int> ws_idx(ws_dictionary_size);
    ROS_INFO("Dictionary Size: %i", ws_dictionary_size);

    // Initialize original index locations
    std::iota(ws_idx.begin(), ws_idx.end(), 1);
    // Compute distances betweeen possible ws to the object
    geometry_msgs::Point lbr_base_pose;
    geometry_msgs::Point temp_pose;
    if (!req.view_point)
    {
        for (int j = 0; j < ws_dictionary_size; j++)
        {
            // Do not take into account bin locations
            if (j != (77 + 1) && j != (78 + 1) && j != (79 + 1) && j != (80 + 1))
            {
                temp_pose.x = candidate_poses_dictionary[j + 1].pose.position.x;
                temp_pose.y = candidate_poses_dictionary[j + 1].pose.position.y;
                distances_between_ws.push_back(this->ComputeScore(temp_pose, req.target_pose.pose.position));
            }
            else
            {
                distances_between_ws.push_back(1000000.0);
            }

            // ROS_INFO(" Distance from Candidate(%i) (%f, %f) to target (%f, %f) is: %f ", ws_idx[j], temp_pose.x, temp_pose.y,
            //          req.target_pose.pose.position.x, req.target_pose.pose.position.y,
            //          distances_between_ws[j]);
        }
        // Sorting index
        int temp_i;
        for (int i = 0; i < ws_dictionary_size; i++)
        {
            for (int j = i + 1; j < ws_dictionary_size; j++)
            {
                if (distances_between_ws[j] < distances_between_ws[i])
                {
                    float temp_f = distances_between_ws[i];
                    distances_between_ws[i] = distances_between_ws[j];
                    distances_between_ws[j] = temp_f;

                    temp_i = ws_idx[i];
                    ws_idx[i] = ws_idx[j];
                    ws_idx[j] = temp_i;
                }
            }
        }
    }
    else
    {
        mobile_base_pose = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", nh);

        distances_between_ws.push_back(this->ComputeScore(
            candidate_poses_dictionary[view_spot_one_id].pose.position, mobile_base_pose->pose.pose.position));
        distances_between_ws.push_back(this->ComputeScore(
            candidate_poses_dictionary[view_spot_two_id].pose.position, mobile_base_pose->pose.pose.position));

        // Assign the closest view_point
        if (distances_between_ws[0] > distances_between_ws[1]){
            ws_idx[0] = view_spot_two_id;

        }else{
            ws_idx[0] = view_spot_one_id;
            distances_between_ws[0] = distances_between_ws[1];

        }
    }

    // Aferter soritng, first element corresponds to the pose that has the  least distance with the object provided
    res.workstation_id = ws_idx[0];
    visualization_msgs::Marker selected_pose;
    selected_pose.header.frame_id = candidate_poses_frame_id;
    selected_pose.header.stamp = ros::Time::now();
    selected_pose.ns = "selected_pose";
    selected_pose.action = visualization_msgs::Marker::ADD;
    selected_pose.pose.orientation.w = 1.0;
    selected_pose.type = visualization_msgs::Marker::SPHERE;
    // POINTS markers use x and y scale for width/height respectively
    selected_pose.scale.x = 0.05;
    selected_pose.scale.y = 0.05;
    selected_pose.scale.z = 0.01;
    // Selected pose  is green
    selected_pose.color.r = 0.0f;
    selected_pose.color.g = 1.0f;
    selected_pose.color.b = 0.0f;
    selected_pose.color.a = 1.0;

    selected_pose.pose.position = candidate_poses_dictionary[ws_idx[0]].pose.position;
    selected_pose_marker_pub.publish(selected_pose);
    ROS_INFO("Selected ws: %i with distance: %f", ws_idx[0], distances_between_ws[0]);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "best_mobile_base_pose");
    ros::NodeHandle nh("");

    int record_poses = 0;
    if (argv[1] != NULL)
        record_poses = atoi(argv[1]);

    BestMobileBasePose best_mobile_pose(&nh);

    if (record_poses)
    {
        best_mobile_pose.recordPoses();
        ros::shutdown();
    }

    ROS_INFO("Best Workstation service ready!");
    ros::spin();

    return 0;
}