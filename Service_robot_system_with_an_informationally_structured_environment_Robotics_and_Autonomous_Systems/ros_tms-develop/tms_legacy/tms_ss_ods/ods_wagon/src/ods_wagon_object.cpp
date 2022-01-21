#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <tms_msg_ss/ods_pcd.h>
#include <tms_msg_ss/ods_wagon_object.h>

#define PI 3.1415926

ros::ServiceServer service;
ros::ServiceClient commander_to_kinect_capture;

double C_OFFSET_X = 0.0;    // 0.21;
double C_OFFSET_Y = 0.045;  // 0.045;
double C_OFFSET_Z = 1.07;
double C_OFFSET_PITCH = 0.0 * (PI / 180.0);

typedef struct
{
  double max_x;
  double min_x;
  double max_z;
  double min_z;
} PLANE;

typedef struct
{
  double x;
  double y;
  double theta;
} Robot;

PLANE plane;
Robot robot;

void passfilter(pcl::PointCloud< pcl::PointXYZRGB >& cloud)
{
  std::cout << "passfilter" << std::endl;

  // Create the filtering object
  pcl::PassThrough< pcl::PointXYZRGB > pass;
  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-0.5, 0.5);
  pass.filter(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-1.0, 0.5);
  // pass.filter (cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.2, 1.0);
  pass.filter(cloud);

  return;
}

//平面検出を行う
void plane_detect(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZRGB >& cloud_out,
                  pcl::ModelCoefficients& coefficients)
{
  pcl::ExtractIndices< pcl::PointXYZRGB > extract;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation< pcl::PointXYZRGB > seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);

  while (1)
  {
    seg.setInputCloud(cloud.makeShared());
    seg.segment(*inliers, coefficients);

    std::cout << "plane:\n" << coefficients << std::endl;

    extract.setInputCloud(cloud.makeShared());
    extract.setIndices(inliers);

    std::cout << atan(-coefficients.values[2] / coefficients.values[3]) << " " << (1.0 * PI / 4.0) << std::endl;

    if ((atan(-coefficients.values[2] / coefficients.values[3]) < (-1.0 * PI / 4.0)) ||
        ((1.0 * PI / 4.0) < atan(-coefficients.values[2] / coefficients.values[3])))
    {
      extract.setInputCloud(cloud.makeShared());
      extract.setNegative(true);
      extract.filter(cloud);
    }

    else
    {
      extract.setNegative(true);
      extract.filter(cloud_out);
      extract.setNegative(false);
      extract.filter(cloud);
      break;
    }
  }

  return;
}

void plane_info(pcl::PointCloud< pcl::PointXYZRGB >& cloud)
{
  double min_x = 100.0;
  double max_x = -100.0;
  double min_z = 100.0;
  double max_z = -100.0;

  for (int i = 0; i < cloud.points.size(); i++)
  {
    if (cloud.points[i].x < min_x)
      min_x = cloud.points[i].x;
    else if (cloud.points[i].x > max_x)
      max_x = cloud.points[i].x;
    if (cloud.points[i].z < min_z)
      min_z = cloud.points[i].z;
    else if (cloud.points[i].z > max_z)
      max_z = cloud.points[i].z;
  }

  plane.min_x = min_x + 0.03;
  plane.max_x = max_x - 0.03;
  plane.min_z = min_z + 0.03;
  plane.max_z = max_z - 0.05;

  return;
}

//重心位置を求める
pcl::PointXYZRGB g_pos(pcl::PointCloud< pcl::PointXYZRGB >& cloud)
{
  pcl::PointXYZRGB g;

  for (int i = 0; i < cloud.size(); i++)
  {
    g.x += cloud.points[i].x;
    g.y += cloud.points[i].y;
    g.z += cloud.points[i].z;
  }

  g.x = g.x / cloud.size();
  g.y = g.y / cloud.size();
  g.z = g.z / cloud.size();
  g.r = 255;
  g.g = 255;
  g.b = 255;

  return g;
}

//クラスタリング
void clustering(pcl::PointCloud< pcl::PointXYZRGB >& cloud, std::vector< pcl::PointXYZRGB >& points,
                pcl::ModelCoefficients& coefficients)
{
  std::cout << "clustering" << std::endl;

  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree< pcl::PointXYZRGB >::Ptr tree(new pcl::search::KdTree< pcl::PointXYZRGB >);
  tree->setInputCloud(cloud.makeShared());

  std::vector< pcl::PointIndices > cluster_indices;
  pcl::EuclideanClusterExtraction< pcl::PointXYZRGB > ec;
  ec.setClusterTolerance(0.015);
  ec.setMinClusterSize(400);
  ec.setMaxClusterSize(10000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud.makeShared());
  ec.extract(cluster_indices);

  // std::cout << "クラスタリング開始" << std::endl;
  pcl::PointXYZRGB g1;

  int m = 0;
  float colors[6][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
  for (std::vector< pcl::PointIndices >::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_cluster(new pcl::PointCloud< pcl::PointXYZRGB >);

    for (std::vector< int >::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      cloud_cluster->points.push_back(cloud.points[*pit]);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    g1 = g_pos(*cloud_cluster);

    for (int i = 0; i < cloud_cluster->points.size(); i++)
    {
      cloud_cluster->points[i].r = colors[m % 6][0];
      cloud_cluster->points[i].g = colors[m % 6][1];
      cloud_cluster->points[i].b = colors[m % 6][2];
    }

    if ((0.02 < abs(coefficients.values[0] + g1.y * coefficients.values[1] + g1.z * coefficients.values[2] +
                    coefficients.values[3])) &&
        (abs(coefficients.values[0] + g1.y * coefficients.values[1] + g1.z * coefficients.values[2] +
             coefficients.values[3]) < 0.14))
    {
      if (coefficients.values[3] * (g1.x * coefficients.values[0] + g1.y * coefficients.values[1] +
                                    g1.z * coefficients.values[2] + coefficients.values[3]) >
          0)
      {
        if ((plane.min_x < g1.x) && (g1.x < plane.max_x) && (plane.min_z < g1.z) && (g1.z < plane.max_z))
        {
          points.push_back(g1);
          *tmp_cloud += *cloud_cluster;
          m++;
        }
      }
    }
  }

  cloud = *tmp_cloud;

  if (m < 2)
    std::cout << "Not find two clusters!!" << std::endl;

  pcl::io::savePCDFile("src/ods_wagon/data/ods_wagon_object/cluster_cloud.pcd", *tmp_cloud, false);

  return;
}

pcl::PointXYZRGB transformation(pcl::PointXYZRGB point)
{
  std::cout << "transformation" << std::endl;

  double camera_x = robot.x + C_OFFSET_Y * cos(robot.theta) + C_OFFSET_X * sin(robot.theta);
  double camera_y = robot.y + C_OFFSET_Y * sin(robot.theta) - C_OFFSET_X * cos(robot.theta);
  double camera_z = C_OFFSET_Z;
  double camera_pitch = C_OFFSET_PITCH * (PI / 180.0);

  //robot.theta *= PI/180.0;\

  Eigen::Affine3f t(Eigen::Affine3f::Identity());

  t(0, 0) = sin(robot.theta);
  t(0, 1) = -sin(camera_pitch) * cos(robot.theta);
  t(0, 2) = cos(camera_pitch) * cos(robot.theta);
  t(1, 0) = -cos(robot.theta);
  t(1, 1) = -sin(camera_pitch) * sin(robot.theta);
  t(1, 2) = cos(camera_pitch) * sin(robot.theta);
  t(2, 0) = 0;
  t(2, 1) = -cos(camera_pitch);
  t(2, 2) = -sin(camera_pitch);

  point = pcl::transformPoint(point, t);

  point.x += camera_x;
  point.y += camera_y;
  point.z += camera_z;

  return point;
}

bool wagon_object(tms_msg_ss::ods_wagon_object::Request& req, tms_msg_ss::ods_wagon_object::Response& res)
{
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud2(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  tms_msg_ss::ods_pcd srv;
  srv.request.id = 3;
  if (commander_to_kinect_capture.call(srv))
  {
    pcl::fromROSMsg(srv.response.cloud, *cloud);
  }

  // pcl::io::loadPCDFile("src/ods_wagon/data/ods_wagon_object/input.pcd", *cloud);
  pcl::io::savePCDFile("src/ods_wagon/data/ods_wagon_object/input.pcd", *cloud);

  pcl::copyPointCloud(*cloud, *tmp_cloud);

  /*for(int i=0;i<tmp_cloud->points.size();i++){
      tmp_cloud->points[i].r = 255;
      tmp_cloud->points[i].g = 255;
      tmp_cloud->points[i].b = 255;
  }*/

  pcl::io::savePCDFile("src/ods_wagon/data/ods_wagon_object/input_xyz.pcd", *tmp_cloud, false);

  passfilter(*cloud);

  pcl::io::savePCDFile("src/ods_wagon/data/ods_wagon_object/filter.pcd", *cloud, false);

  for (int i = 0; i < cloud->points.size(); i++)
  {
    cloud->points[i].r = 255;
    cloud->points[i].g = 0;
    cloud->points[i].b = 0;
  }

  // plane_detect(*cloud, *cloud2, *coefficients);

  plane_info(*cloud);

  *tmp_cloud += *cloud;

  pcl::io::savePCDFile("src/ods_wagon/data/ods_wagon_object/plane.pcd", *tmp_cloud, true);
  pcl::io::savePCDFile("src/ods_wagon/data/ods_wagon_object/plane_reduction.pcd", *cloud2, false);

  std::vector< pcl::PointXYZRGB > g_cloud;

  clustering(*cloud2, g_cloud, *coefficients);

  for (int i = 0; i < g_cloud.size(); i++)
  {
    g_cloud[i] = transformation(g_cloud[i]);
    geometry_msgs::Pose pose;
    pose.position.x = g_cloud[i].x;
    pose.position.y = g_cloud[i].y;
    pose.position.z = g_cloud[i].z;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
    res.objects.poses.push_back(pose);
  }

  return true;
}

int main(int argc, char** argv)
{
  printf("init\n");
  ros::init(argc, argv, "ods_wagon_object");
  ros::NodeHandle n;

  service = n.advertiseService("ods_wagon_object", wagon_object);

  commander_to_kinect_capture = n.serviceClient< tms_msg_ss::ods_pcd >("capture_cloud");

  ros::spin();

  return 0;
}
