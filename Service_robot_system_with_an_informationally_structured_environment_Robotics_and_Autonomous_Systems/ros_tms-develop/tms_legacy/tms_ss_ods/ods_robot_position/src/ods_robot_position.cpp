#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/array.hpp>

// include for ROS
#include "ros/ros.h"
#include <tms_msg_ss/ods_get_robots_pos.h>
#include <tms_msg_ss/ods_pcd.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tms_msg_rc/smartpal_control.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>

ros::ServiceServer service;
ros::ServiceClient commander_to_kinect_capture;

int TABLE;

#define MAP_X 50
#define MAP_Y 50

typedef struct
{
  pcl::PointCloud< pcl::PointXYZ > cloud;
  pcl::PointCloud< pcl::PointXYZRGB > cloud_rgb;
  pcl::PointIndices inliers;
  pcl::PointXYZ g;
  int map[MAP_X][MAP_Y];
  int tf;
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  double max_z;
  double min_z;
  int size_x;
  int size_y;
} object_map;

std::vector< object_map > Object_Map;

#define SMARTPAL4 0
#define SMARTPAL5 1
#define PI 3.1415926

double C_OFFSET_X;
double C_OFFSET_Y;
double C_OFFSET_Z;
double C_OFFSET_YAW;

typedef struct
{
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  double min_z;
  double max_z;
} Model;

Model m_size;

typedef struct
{
  double x;
  double y;
  double theta;
} Robot;

typedef struct
{
  double x;
  double y;
  double z;
} Furniture;

Robot robot;
Furniture table;

pcl::PointCloud< pcl::PointXYZRGB > viewpoint;

void transformation(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZ >& model)
{
  std::cout << "transformation" << std::endl;

  double camera_x = robot.x + C_OFFSET_Y * cos(robot.theta) + C_OFFSET_X * sin(robot.theta);
  double camera_y = robot.y + C_OFFSET_Y * sin(robot.theta) - C_OFFSET_X * cos(robot.theta);
  double camera_z = C_OFFSET_Z;
  double camera_yaw = C_OFFSET_YAW * (PI / 180.0);

  robot.theta *= PI / 180.0;

  Eigen::Matrix4f t1(Eigen::Matrix4f::Identity());

  t1(0, 0) = sin(robot.theta);
  t1(0, 1) = -sin(camera_yaw) * cos(robot.theta);
  t1(0, 2) = cos(camera_yaw) * cos(robot.theta);
  t1(0, 3) = camera_x;
  t1(1, 0) = -cos(robot.theta);
  t1(1, 1) = -sin(camera_yaw) * sin(robot.theta);
  t1(1, 2) = cos(camera_yaw) * sin(robot.theta);
  t1(1, 3) = camera_y;
  t1(2, 0) = 0;
  t1(2, 1) = -cos(camera_yaw);
  t1(2, 2) = -sin(camera_yaw);
  t1(2, 3) = camera_z;
  t1(3, 0) = 0;
  t1(3, 1) = 0;
  t1(3, 2) = 0;
  t1(3, 3) = 1;

  pcl::transformPointCloud(viewpoint, viewpoint, t1);
  pcl::transformPointCloud(cloud, cloud, t1);

  t1(0, 0) = 1;
  t1(0, 1) = 0;
  t1(0, 2) = 0;
  t1(0, 3) = table.x;
  t1(1, 0) = 0;
  t1(1, 1) = 1;
  t1(1, 2) = 0;
  t1(1, 3) = table.y;
  t1(2, 0) = 0;
  t1(2, 1) = 0;
  t1(2, 2) = 1;
  t1(2, 3) = table.z;
  t1(3, 0) = 0;
  t1(3, 1) = 0;
  t1(3, 2) = 0;
  t1(3, 3) = 1;

  pcl::transformPointCloud(model, model, t1);

  return;
}

void filtering(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZ >& model,
               pcl::PointIndices& inliers)
{
  std::cout << "filtering" << std::endl;

  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);

  m_size.min_x = 100.0;
  m_size.max_x = -100.0;
  m_size.min_y = 100.0;
  m_size.max_y = -100.0;
  m_size.min_z = 100.0;
  m_size.max_z = -100.0;

  for (int i = 0; i < model.points.size(); i++)
  {
    if (model.points[i].x < m_size.min_x)
      m_size.min_x = model.points[i].x;
    if (model.points[i].x > m_size.max_x)
      m_size.max_x = model.points[i].x;
    if (model.points[i].y < m_size.min_y)
      m_size.min_y = model.points[i].y;
    if (model.points[i].y > m_size.max_y)
      m_size.max_y = model.points[i].y;
    if (model.points[i].z < m_size.min_z)
      m_size.min_z = model.points[i].z;
    if (model.points[i].z > m_size.max_z)
      m_size.max_z = model.points[i].z;
  }

  for (int i = 0; i < cloud.points.size(); i++)
  {
    if (((m_size.min_x - 0.3 <= cloud.points[i].x) && (cloud.points[i].x <= m_size.max_x + 0.3)) &&
        ((m_size.min_y - 0.3 <= cloud.points[i].y) && (cloud.points[i].y <= m_size.max_y + 0.3)) &&
        ((m_size.min_z + 0.05 <= cloud.points[i].z) && (cloud.points[i].z <= m_size.max_z + 0.3)))
    {
      tmp_cloud->push_back(cloud.points[i]);
      inliers.indices.push_back(i);
    }
  }

  pcl::copyPointCloud(*tmp_cloud, cloud);

  return;
}

//ダウンサンプリングを行う
void downsampling(pcl::PointCloud< pcl::PointXYZ >& cloud, float th)
{
  pcl::VoxelGrid< pcl::PointXYZ > sor;
  sor.setInputCloud(cloud.makeShared());
  sor.setLeafSize(th, th, th);
  sor.filter(cloud);

  return;
}

//マッチング
void registration(pcl::PointCloud< pcl::PointXYZ >& cloud, pcl::PointCloud< pcl::PointXYZ >& model,
                  pcl::PointCloud< pcl::PointXYZ >& cloud_out, pcl::PointCloud< pcl::PointXYZRGB >& tmp_rgb,
                  Eigen::Matrix4f& m)
{
  pcl::IterativeClosestPoint< pcl::PointXYZ, pcl::PointXYZ > icp;
  icp.setInputSource(cloud.makeShared());
  icp.setInputTarget(model.makeShared());
  pcl::PointCloud< pcl::PointXYZ > Final;
  icp.align(Final);

  m = icp.getFinalTransformation();
  // std::cout << m << std::endl;

  pcl::transformPointCloud(cloud, cloud, m);
  pcl::transformPointCloud(tmp_rgb, tmp_rgb, m);
  pcl::transformPointCloud(viewpoint, viewpoint, m);
  pcl::copyPointCloud(tmp_rgb, cloud_out);

  cloud_out += model;

  return;
}

///////////////////////////////////////////////////////
//*******************main function*******************//
///////////////////////////////////////////////////////

bool robot_position(tms_msg_ss::ods_get_robots_pos::Request& req, tms_msg_ss::ods_get_robots_pos::Response& res)
{
  //***************************
  // local variable declaration
  //***************************
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud1(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZ >::Ptr model(new pcl::PointCloud< pcl::PointXYZ >);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  //***************************
  // capture kinect data
  //***************************
  tms_msg_ss::ods_pcd srv;
  srv.request.id = 3;
  if (commander_to_kinect_capture.call(srv))
  {
    pcl::fromROSMsg(srv.response.cloud, *cloud1);
  }

  if (SMARTPAL4)
  {
    C_OFFSET_X = 0.02;
    C_OFFSET_Y = 0.14;
    C_OFFSET_Z = 1.38;
    C_OFFSET_YAW = 21.6;
  }

  else if (SMARTPAL5)
  {
    C_OFFSET_X = 0.0;
    C_OFFSET_Y = 0.05;  // 0.01;
    C_OFFSET_Z = 1.05;
    C_OFFSET_YAW = 10.0;
  }

  viewpoint.resize(4);
  viewpoint[0].x = 0.0;
  viewpoint[0].y = 0.0;
  viewpoint[0].z = 0.0;
  viewpoint[1].x = 1.0;
  viewpoint[1].y = 0.0;
  viewpoint[1].z = 0.0;
  viewpoint[2].x = 0.0;
  viewpoint[2].y = 1.0;
  viewpoint[2].z = 0.0;
  viewpoint[3].x = 0.0;
  viewpoint[3].y = 0.0;
  viewpoint[3].z = 1.0;

  // std::cout << "viewpoint: " << viewpoint[0].x << " " << viewpoint[0].y << " " << viewpoint[0].z << std::endl;

  // pcl::io::loadPCDFile ("src/ods_robot_position/data/robot_position/table.pcd", *cloud1);
  // pcl::io::loadPCDFile ("src/ods_robot_position/data/robot_position/model_table1.pcd", *model);
  // pcl::io::loadPCDFile ("src/ods_robot_position/data/robot_position/model_shelf.pcd", *model);

  //***************************
  // Fill in the cloud data
  //***************************
  table.x = req.furniture.position.x / 1000.0;
  table.y = req.furniture.position.y / 1000.0;
  table.z = req.furniture.position.z / 1000.0;
  robot.x = req.robot.x / 1000.0;
  robot.y = req.robot.y / 1000.0;
  robot.theta = req.robot.theta;

  /*tms_msg_rc::smartpal_control srv3;
  srv3.request.unit = 1;
  srv3.request.cmd = 8;
  if(commander_to_get_robots_info.call(srv3)){
      robots.x = srv3.response.val[0];
      robots.y = srv3.response.val[1];
      robots.theta = srv3.response.val[2];
  }*/

  std::cout << "table: " << table.x << " " << table.y << " " << table.z << std::endl;
  std::cout << "robot: " << robot.x << " " << robot.y << " " << robot.theta << std::endl;
  pcl::fromROSMsg(req.model, *model);
  // pcl::io::savePCDFile("src/ods_robot_position/data/robot_position/model.pcd", *model);

  //***************************
  // transform input cloud
  //***************************
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tfm_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);

  transformation(*cloud1, *model);

  pcl::copyPointCloud(*model, *tfm_cloud);

  for (int i = 0; i < tfm_cloud->points.size(); i++)
  {
    tfm_cloud->points[i].r = 255;
    tfm_cloud->points[i].g = 255;
    tfm_cloud->points[i].b = 255;
  }

  for (int i = 0; i < cloud1->points.size(); i++)
  {
    cloud1->points[i].r = 255;
    cloud1->points[i].g = 0;
    cloud1->points[i].b = 0;
  }

  *tfm_cloud += *cloud1;

  pcl::io::savePCDFile("src/ods_robot_position/data/robot_position/tfm_cloud1.pcd", *tfm_cloud);
  pcl::io::savePCDFile("src/rods_obot_position/data/robot_position/tfm_cloud2.pcd", *cloud1);

  //***************************
  // filtering by using model
  //***************************
  filtering(*cloud1, *model, *inliers);

  pcl::io::savePCDFile("src/ods_robot_position/data/robot_position/filter.pcd", *cloud1);

  pcl::PointCloud< pcl::PointXYZ >::Ptr dsp_cloud(new pcl::PointCloud< pcl::PointXYZ >);
  pcl::copyPointCloud(*cloud1, *dsp_cloud);
  downsampling(*dsp_cloud, 0.02);

  //***************************
  // registration between two input pcd data
  //***************************
  Eigen::Matrix4f m1;
  std::cout << "registration" << std::endl;
  int n = 0;
  while (1)
  {
    // registration
    pcl::PointCloud< pcl::PointXYZ >::Ptr m_cloud(new pcl::PointCloud< pcl::PointXYZ >);
    registration(*dsp_cloud, *model, *m_cloud, *cloud1, m1);

    if ((double)(m1(0, 0) + m1(1, 1) + m1(2, 2) + m1(3, 3)) >= 4)
    {
      if (n > 2)
        break;
    }
    n++;
  }

  std::cout << viewpoint[0] << std::endl;

  res.m_robot.theta = atan2(viewpoint[3].y - viewpoint[0].y, viewpoint[3].x - viewpoint[0].x) * 180.0 / PI;
  res.m_robot.x = 1000.0 * (viewpoint[0].x + C_OFFSET_X * cos((res.m_robot.theta - 90.0) * PI / 180) -
                            C_OFFSET_Y * cos(res.m_robot.theta * PI / 180));
  res.m_robot.y = 1000.0 * (viewpoint[0].y + C_OFFSET_X * sin((res.m_robot.theta - 90.0) * PI / 180) -
                            C_OFFSET_Y * sin(res.m_robot.theta * PI / 180));

  std::cout << res.m_robot.x << " " << res.m_robot.y << " " << viewpoint[0].z - C_OFFSET_Z << " " << res.m_robot.theta
            << std::endl;

  FILE* fp;

  if ((fp = fopen("src/ods_robot_position/data/robot_position/robot_position.txt", "a")) == NULL)
  {
    std::cout << "file open error" << std::endl;
  }

  fprintf(fp, "%lf %lf %lf\n", res.m_robot.x, res.m_robot.y, res.m_robot.theta);

  fclose(fp);

  return true;
}

int main(int argc, char** argv)
{
  printf("init\n");
  ros::init(argc, argv, "ods_robot_position");
  ros::NodeHandle n;

  service = n.advertiseService("ods_robot_position", robot_position);

  commander_to_kinect_capture = n.serviceClient< tms_msg_ss::ods_pcd >("capture_cloud");

  ros::spin();

  return 0;
}
