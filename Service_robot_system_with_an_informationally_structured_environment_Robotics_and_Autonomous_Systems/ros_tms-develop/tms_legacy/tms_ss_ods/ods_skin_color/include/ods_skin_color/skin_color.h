#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tms_msg_ss/ods_skincolor_extraction.h>
#include <tms_msg_ss/ods_pcd.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter_indices.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cxcore.h>
#include <opencv2/legacy/compat.hpp>

ros::ServiceServer service;
ros::ServiceClient commander_to_kinect_capture;

// 大域変数
const int IMAGE_WIDTH = 640;   //画像サイズ(横)
const int IMAGE_HEIGHT = 480;  //画像サイズ(縦)
int count = 0;

pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
std::vector< int > idx;
