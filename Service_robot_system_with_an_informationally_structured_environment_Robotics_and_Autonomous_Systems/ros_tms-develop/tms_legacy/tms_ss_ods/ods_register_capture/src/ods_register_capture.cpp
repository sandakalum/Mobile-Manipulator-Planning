#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <ros/ros.h>
#include <tms_msg_ss/ods_pcd.h>

#define SMARTPAL4 1
#define SMARTPAL5 0
#define PI 3.1415926

double C_OFFSET_X = 0.02;
double C_OFFSET_Y = 0.14;
double C_OFFSET_Z = 1.3;
double C_OFFSET_YAW = 21.6;

ros::ServiceClient commander_to_kinect_capture;

int main(int argc, char **argv)
{
  printf("init\n");
  ros::init(argc, argv, "ods_register_capture");
  ros::NodeHandle n;

  commander_to_kinect_capture = n.serviceClient< tms_msg_ss::ods_pcd >("capture_cloud");

  //**************************
  // initialize
  //**************************
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr model(new pcl::PointCloud< pcl::PointXYZRGB >);

  //**************************
  // input cloud
  //**************************
  tms_msg_ss::ods_pcd srv;
  srv.request.id = 1;
  /*if(commander_to_kinect_capture.call(srv)){
      pcl::fromROSMsg (srv.response.cloud, *cloud);
  }*/

  pcl::io::loadPCDFile("src/ods_register_capture/data/register_capture/sample1.pcd", *cloud);
  pcl::io::loadPCDFile("src/ods_register_capture/data/register_capture/model_environment2.pcd", *model);

  for (int i = 0; i < cloud->points.size(); i++)
  {
    cloud->points[i].r = 255;
    cloud->points[i].g = 255;
    cloud->points[i].b = 255;
  }

  double robot_x = 3.500;
  double robot_y = 2.500;
  double robot_theta = 45.0 * (PI / 180.0);

  // int furniture_id = 14;
  // double furniture_x = 1.400;
  // double furniture_y = 1.900;
  // double furniture_z = 0.700;

  //絶対座標空間におけるカメラの位置を求める
  double camera_x = robot_x + C_OFFSET_Y * cos(robot_theta) + C_OFFSET_X * sin(robot_theta);
  double camera_y = robot_y + C_OFFSET_Y * sin(robot_theta) - C_OFFSET_X * cos(robot_theta);
  double camera_z = C_OFFSET_Z;
  double camera_yaw = C_OFFSET_YAW * (PI / 180.0);

  std::cout << "(" << camera_x << ", " << camera_y << ", " << camera_z << ", " << camera_yaw << ")" << std::endl;

  //モデルをカメラから見た姿勢に回転
  Eigen::Matrix4f t1(Eigen::Matrix4f::Identity());

  t1(0, 0) = sin(robot_theta);
  t1(0, 1) = -sin(camera_yaw) * cos(robot_theta);
  t1(0, 2) = cos(camera_yaw) * cos(robot_theta);
  t1(0, 3) = camera_x;
  t1(1, 0) = -cos(robot_theta);
  t1(1, 1) = -sin(camera_yaw) * sin(robot_theta);
  t1(1, 2) = cos(camera_yaw) * sin(robot_theta);
  t1(1, 3) = camera_y;
  t1(2, 0) = 0;
  t1(2, 1) = -cos(camera_yaw);
  t1(2, 2) = -sin(camera_yaw);
  t1(2, 3) = camera_z;
  t1(3, 0) = 0;
  t1(3, 1) = 0;
  t1(3, 2) = 0;
  t1(3, 3) = 1;

  pcl::transformPointCloud(*cloud, *cloud, t1);

  *model += *cloud;

  pcl::io::savePCDFile("src/ods_register_capture/data/register_capture/result.pcd", *model);

  return 0;
}
