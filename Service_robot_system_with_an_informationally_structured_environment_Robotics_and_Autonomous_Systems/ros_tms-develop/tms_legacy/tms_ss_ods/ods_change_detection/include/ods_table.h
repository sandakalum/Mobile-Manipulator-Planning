#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types_conversion.h>

// include for ROS
#include <ros/ros.h>
#include <tms_msg_ss/ods_furniture.h>
#include <tms_msg_ss/ods_pcd.h>
#include <tms_msg_db/tmsdb_ods_object_data.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sys/stat.h>

ros::ServiceServer service;
ros::ServiceClient commander_to_kinect_capture;
ros::ServiceClient commander_to_get_robots_pos;
ros::ServiceClient commander_to_ods_object_data;

#define PI 3.1415926
#define SIZE_X 100
#define SIZE_Y 100
#define SIZE_Z 100
#define VOXEL_SIZE 2
#define MAP_X 80
#define MAP_Y 80
#define MAP_H 32  // H
#define MAP_S 32  // S
#define Bhattacharyya 1

typedef struct
{
  pcl::PointCloud< pcl::PointXYZRGB > cloud_rgb;
  pcl::PointCloud< pcl::PointXYZHSV > cloud_hsv;
  pcl::PointXYZ g;
  int map[MAP_X][MAP_Y];
  float histogram[MAP_H][MAP_S];  // H-Sヒストグラム
  int tf;
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  double min_z;
  double max_z;
  int size_x;
  int size_y;
} object_map;

typedef struct
{
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  double min_z;
  double max_z;
} Model;

typedef struct
{
  double x;
  double y;
  double z;
  double theta;
} Robot;

typedef struct
{
  double x;
  double y;
  double z;
  double theta;
} Furniture;

typedef struct
{
  float index_h;  // Hのインデックス
  float index_s;  // Sのインデックス
  float value;    // histogram[index_h][index_s]の値
} TYPE;

std::vector< object_map > Object_Map;
Model m_size;
Robot robot;
Robot sensor;
Furniture table;

int T_voxel[SIZE_X][SIZE_Y][SIZE_Z];
int TABLE;
