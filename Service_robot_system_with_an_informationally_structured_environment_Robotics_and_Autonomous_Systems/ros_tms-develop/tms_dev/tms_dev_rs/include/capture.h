#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

ros::Subscriber ir_sub;
ros::Subscriber pcl_sub;
int n = 0;

typedef pcl::PointXYZRGB PointType1;
typedef pcl::PointXYZI PointType2;
