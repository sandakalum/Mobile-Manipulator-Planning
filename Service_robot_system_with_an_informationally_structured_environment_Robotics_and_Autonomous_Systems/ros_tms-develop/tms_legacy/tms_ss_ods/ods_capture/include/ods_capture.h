#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <tms_msg_ss/ods_pcd.h>

// PCL specific includes
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

ros::Subscriber pcl_sub;
ros::Subscriber rgb_sub;
ros::Subscriber depth_sub;
ros::Subscriber ir_sub;
ros::ServiceServer service;
ros::ServiceClient commander_to_kinect_capture;

int n = 0;

typedef pcl::PointXYZRGB PointType1;
typedef pcl::PointXYZI PointType2;

sensor_msgs::PointCloud2::Ptr client(new sensor_msgs::PointCloud2());
sensor_msgs::Image::Ptr image(new sensor_msgs::Image);
