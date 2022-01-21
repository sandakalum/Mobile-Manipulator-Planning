#include <iostream>
#include <cstdio>
// #include <stdio>
#include <fstream>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/SetBool.h>

int record = 3;

bool add(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    record = req.data;
    res.success = true;
    return true;
}

float computeEuclideanDistance(geometry_msgs::Transform &from, geometry_msgs::Transform &to)
{

    float d_x = std::abs(to.translation.x - from.translation.x);
    float d_y = std::abs(to.translation.y - from.translation.y);
    float d_z = std::abs(to.translation.z - from.translation.z);

    return std::sqrt(d_x * d_x + d_y * d_y + d_z * d_z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_recorder");

    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("init_path_recorder", add);

    std::ofstream FILE;

    std::string file_loc = "/home/hector/Desktop/data/path_trajectory.csv";

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    std::vector<geometry_msgs::Transform> path_vector;

    ros::Rate rate(20.0);

    while (nh.ok())
    {
        ros::spinOnce();

        if (record == 1)
        {
            // ROS_INFO("Record started");

            geometry_msgs::TransformStamped transformStamped;
            try
            {
                transformStamped = tfBuffer.lookupTransform("base_footprint", "vaccum_0",
                                                            ros::Time(0));
                path_vector.push_back(transformStamped.transform);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }
        else if (record == 0)
        {
            std::cout << "BREAK" << std::endl;
            break;
        }
        rate.sleep();
    }

	/*	Deletes the file if exists */
	if (std::remove(file_loc.c_str()) != 0)
		std::perror("There is no file to delete");
	else
		std::cout << "File deleted successfully"<<std::endl;
    // geometry_msgs::Pose;
    FILE.open(file_loc);
    // ROS_INFO("Path Vector Size: %lu", path_vector.size());
    std::cout << "Vector Size" << path_vector.size() << std::endl;

    // std::vector<geometry_msgs::Transform> filtered_vector;

   
    // for (int j)
   
    int i = 0;
    
    for (auto it = path_vector.begin(); it != path_vector.end(); it++, i++)
    {

        FILE << it->translation.x << "," << it->translation.y << "," << it->translation.z << ","
             << it->rotation.x << "," << it->rotation.y << "," << it->rotation.z << "," << it->rotation.w << "\n";
    }
    ROS_INFO("Record finished");

    FILE.close();
    ros::shutdown();
    return 0;
};