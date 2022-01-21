#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <tms_msg_ss/ods_pcd.h>
#include <tms_msg_ss/ods_person_detection.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cxcore.h>

#define PI 3.1415926
#define IMAGE_WIDTH 640

ros::ServiceServer service;
ros::ServiceClient commander_to_kinect_capture;

typedef struct
{
  pcl::PointCloud< pcl::PointXYZRGB > cloud;
  pcl::Normal normal;
  int inlier1;
  int inlier2;
  pcl::PointXYZRGB p1;
  pcl::PointXYZRGB p2;
} Endpoints;
