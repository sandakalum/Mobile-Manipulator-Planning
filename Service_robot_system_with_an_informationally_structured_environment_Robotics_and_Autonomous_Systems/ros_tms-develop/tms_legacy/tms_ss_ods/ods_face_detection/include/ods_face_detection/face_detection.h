//�w�b�_�[�t�@�C��
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

// ���ϐ�
const int IMAGE_WIDTH = 640;     //�摜�T�C�Y(��)
const int IMAGE_HEIGHT = 480;    //�摜�T�C�Y(�c)
const double pi = 3.1415926535;  //�~����
const double f = 526.37013657;   //�ŗL�l
const int TRY = 5;
const int SUC = 4;

cv_bridge::CvImagePtr cv_ptr;
pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
std::vector< int > idx;
cv::Mat mask(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);

int a = 0;         //��F����������
double angle = 0;  //��̌X��
int channel = 1;   //���[�h0�F�W���A1�F���]�A2�F��]
int X = 0, Y = 0;  //��̒��S�ʒu
int trynum = 0;    //��F���֐����s��
int sucnum = 0;    //��F��������

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
