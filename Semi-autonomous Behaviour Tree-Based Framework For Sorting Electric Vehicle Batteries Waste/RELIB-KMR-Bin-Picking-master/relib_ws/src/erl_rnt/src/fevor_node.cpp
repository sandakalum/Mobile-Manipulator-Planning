/*C++17*/
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>

/*ROS*/
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

/*OpenGL*/
#define GL_SILENCE_DEPRECATION
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb/stb_image_write.h>

#include <erl_rnt/util.hpp>
#include <erl_rnt/status.hpp>
#include <erl_rnt/Recognise.h>
#include <erl_rnt/Features.h>
#include "fevor/fevor.hpp"

/*OpenCV*/
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


#define MATCH_THRESHOLD 10
#define FILE 0

rs2::video_frame *color_frame = NULL;
rs2::points points;

fevor::Detector detector("scene_workstation.pcd", "module_workstation.pcd", "3", "0.000895", "0.02", "30000");
float crop_x = 0; float crop_y = 0; int matched = 0; float crop_z = 0;


bool recognise_callback(erl_rnt::Recognise::Request& recognise_req, erl_rnt::Recognise::Response& recognise_res)
{
    ROS_INFO_STREAM("Detection started");
   // detector.pose_estimation();

    // if(matched > MATCH_THRESHOLD)
    // {
    //   recognise_res.status = BATTERY_RECOGNISED;
    // }
    // else
    // {
    //   recognise_res.status = NOTHING_RECOGNISED;
    // }
    // recognise_res.header.frame_id = recognise_req.header.frame_id;
    // recognise_res.header.stamp = ros::Time::now();

    return true;
}



int main (int argc, char **argv) 
{
    ros::init(argc, argv, "fevor_node");
    ros::NodeHandle nh;

    ros::ServiceServer recognise_server = nh.advertiseService("/erl_leaf/recognise", recognise_callback);
    //ros::Publisher     pose_pub     = nh.advertise<geometry_msgs::>("/erl_leaf/features", 100);


    while ( ros::ok() ) // Application still alive?
    {

        //ROS_INFO_STREAM("The depth camera is facing an object " << dist_to_center << " meters away.");
        ros::spinOnce();

    }

    return 0;
}