//ヘッダーファイル
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.h>
#include <tms_msg_ss/ods_face_detection.h>
#include <tms_msg_ss/ods_pcd.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter_indices.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cxcore.h>

ros::ServiceServer service;
ros::ServiceClient commander_to_kinect_capture;

// 大域変数
const int IMAGE_WIDTH = 640;     //画像サイズ(横)
const int IMAGE_HEIGHT = 480;    //画像サイズ(縦)
const double pi = 3.1415926535;  //円周率
const double f = 526.37013657;   //固有値
const int TRY = 5;
const int SUC = 4;

cv_bridge::CvImagePtr cv_ptr;
pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
std::vector< int > idx;
cv::Mat mask(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);

int a = 0;         //顔認識成功判定
double angle = 0;  //顔の傾き
int channel = 1;   //モード0：標準、1：反転、2：回転
int X = 0, Y = 0;  //顔の中心位置
int trynum = 0;    //顔認識関数実行回数
int sucnum = 0;    //顔認識成功回数

typedef struct
{
  double x;
  double y;
  int z;
} Position;

std::string cascadeName =
    //"/opt/ros/groovy/share/OpenCV/haarcascades/haarcascade_profileface.xml";
    "/opt/ros/groovy/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml";

std::string nestedCascadeName = "/opt/ros/groovy/share/OpenCV/haarcascade_profileface.xml";
