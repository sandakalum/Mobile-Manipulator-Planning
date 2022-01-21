#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tms_msg_ss/ods_cylinder.h>
#include <tms_msg_ss/ods_pcd.h>

ros::ServiceServer service;
ros::ServiceClient commander_to_kinect_capture;

typedef pcl::PointXYZRGB PointType;

#define MAP_X 50
#define MAP_Y 50

typedef struct
{
  pcl::PointCloud< PointType > cloud;
  pcl::PointIndices inliers;
  pcl::PointXYZ g;
  double map[MAP_X][MAP_Y];
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  double size_x;
  double size_y;
  double radious;
} object;

std::vector< object > Object_Map;

void passfilter(pcl::PointCloud< PointType >& cloud)
{
  // Create the filtering object
  pcl::PassThrough< PointType > pass;
  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-0.8, 0.8);
  pass.filter(cloud);
  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-5.0, 2.0);
  pass.filter(cloud);
  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-1.0, 0.5);
  pass.filter(cloud);

  return;
}

//ダウンサンプリングを行う
void downsampling(pcl::PointCloud< PointType >& cloud, float th)
{
  pcl::VoxelGrid< PointType > sor;
  sor.setInputCloud(cloud.makeShared());
  sor.setLeafSize(th, th, th);
  sor.filter(cloud);

  return;
}

//平面検出を行う
void segmentate(pcl::PointCloud< PointType >& cloud, pcl::PointCloud< PointType >& p_cloud,
                pcl::ModelCoefficients& coefficients)
{
  pcl::ExtractIndices< PointType > extract;

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation< PointType > seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);

  seg.setInputCloud(cloud.makeShared());
  seg.segment(*inliers, coefficients);

  extract.setInputCloud(cloud.makeShared());
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(p_cloud);
  extract.setNegative(true);
  extract.filter(cloud);

  return;
}

//重心位置を求める
pcl::PointXYZ g_pos(pcl::PointCloud< PointType >& cloud)
{
  pcl::PointXYZ g;

  for (int i = 0; i < cloud.size(); i++)
  {
    g.x += cloud.points[i].x;
    g.y += cloud.points[i].y;
    g.z += cloud.points[i].z;
  }

  g.x = g.x / cloud.size();
  g.y = g.y / cloud.size();
  g.z = g.z / cloud.size();

  return g;
}

//射影変換(平面のz値を0にする→テーブル上面を基準面にする)
void transformation(pcl::PointCloud< PointType >& cloud, pcl::PointCloud< PointType >& p_cloud,
                    pcl::ModelCoefficientsPtr coefficients)
{
  Eigen::Matrix4f r1(Eigen::Matrix4f::Identity());  //平行移動
  Eigen::Matrix4f r2(Eigen::Matrix4f::Identity());  // x軸まわりの回転行列
  Eigen::Matrix4f r3(Eigen::Matrix4f::Identity());  // y軸まわりの回転行列
  pcl::PointXYZ g;
  pcl::PointCloud< PointType >::Ptr tmp_cloud(new pcl::PointCloud< PointType >);

  pcl::ProjectInliers< PointType > proj;
  proj.setModelType(pcl::SACMODEL_PLANE);

  proj.setInputCloud(p_cloud.makeShared());
  proj.setModelCoefficients(coefficients);
  proj.filter(p_cloud);

  // z=0の平面に射影変換
  //平行移動
  g = g_pos(p_cloud);
  r1(0, 0) = 1;
  r1(0, 1) = 0;
  r1(0, 2) = 0;
  r1(0, 3) = -g.x;
  r1(1, 0) = 0;
  r1(1, 1) = 1;
  r1(1, 2) = 0;
  r1(1, 3) = -g.y;
  r1(2, 0) = 0;
  r1(2, 1) = 0;
  r1(2, 2) = 1;
  r1(2, 3) = -g.z;
  r1(3, 0) = 0;
  r1(3, 1) = 0;
  r1(3, 2) = 0;
  r1(3, 3) = 1;
  pcl::transformPointCloud(p_cloud, p_cloud, r1);
  pcl::transformPointCloud(cloud, cloud, r1);

  //回転
  double rad;

  // x軸まわりの回転を考える
  rad = atan((coefficients->values[1]) / coefficients->values[2]);
  r2(0, 0) = 1;
  r2(0, 1) = 0;
  r2(0, 2) = 0;
  r2(0, 3) = 0;
  r2(1, 0) = 0;
  r2(1, 1) = cos(rad);
  r2(1, 2) = -sin(rad);
  r2(1, 3) = 0;
  r2(2, 0) = 0;
  r2(2, 1) = sin(rad);
  r2(2, 2) = cos(rad);
  r2(2, 3) = 0;
  r2(3, 0) = 0;
  r2(3, 1) = 0;
  r2(3, 2) = 0;
  r2(3, 3) = 1;
  pcl::transformPointCloud(p_cloud, p_cloud, r2);
  pcl::transformPointCloud(cloud, cloud, r2);
  pcl::copyPointCloud(p_cloud, *tmp_cloud);
  std::cout << atan((-coefficients->values[0]) / coefficients->values[2]) << std::endl;
  segmentate(*tmp_cloud, p_cloud, *coefficients);
  std::cout << atan((-coefficients->values[0]) / coefficients->values[2]) << std::endl;

  // y軸まわりの回転を考える
  rad = atan((-coefficients->values[0]) / coefficients->values[2]);
  r3(0, 0) = cos(rad);
  r3(0, 1) = 0;
  r3(0, 2) = sin(rad);
  r3(0, 3) = 0;
  r3(1, 0) = 0;
  r3(1, 1) = 1;
  r3(1, 2) = 0;
  r3(1, 3) = 0;
  r3(2, 0) = -sin(rad);
  r3(2, 1) = 0;
  r3(2, 2) = cos(rad);
  r3(2, 3) = 0;
  r3(3, 0) = 0;
  r3(3, 1) = 0;
  r3(3, 2) = 0;
  r3(3, 3) = 1;
  pcl::transformPointCloud(p_cloud, p_cloud, r3);
  pcl::transformPointCloud(cloud, cloud, r3);

  return;
}

//グリッド化
void elevation_map(object& m)
{
  // std::cout << "grid" << std::endl;
  double max_x = -10.0;
  double max_y = -10.0;
  double max_z = -10.0;
  double min_x = 10.0;
  double min_y = 10.0;
  double min_z = 10.0;

  pcl::PointCloud< PointType >::Ptr tmp_cloud(new pcl::PointCloud< PointType >);
  *tmp_cloud = m.cloud;

  for (int i = 0; i < tmp_cloud->size(); i++)
  {
    /*if((tmp_cloud->points[i].x < -1.0)||(tmp_cloud->points[i].y < -1.0)||(-tmp_cloud->points[i].z < -1.0)){
        continue;
    }*/
    if (tmp_cloud->points[i].x > max_x)
      max_x = tmp_cloud->points[i].x;
    if (tmp_cloud->points[i].x < min_x)
      min_x = tmp_cloud->points[i].x;
    if (tmp_cloud->points[i].y > max_y)
      max_y = tmp_cloud->points[i].y;
    if (tmp_cloud->points[i].y < min_y)
      min_y = tmp_cloud->points[i].y;
  }

  m.max_x = max_x;
  m.min_x = min_x;
  m.max_y = max_y;
  m.min_y = min_y;

  m.size_x = max_x - min_x;
  m.size_y = max_y - min_y;
  for (int i = 0; i < MAP_X; i++)
  {
    for (int j = 0; j < MAP_Y; j++)
    {
      m.map[i][j] = 0;
    }
  }
  for (int i = 0; i < tmp_cloud->size(); i++)
  {
    if (-tmp_cloud->points[i].z >
        m.map[(int)(100 * (tmp_cloud->points[i].x - min_x))][(int)(100 * (tmp_cloud->points[i].y - min_y))])
    {
      m.map[(int)(100 * (tmp_cloud->points[i].x - min_x))][(int)(100 * (tmp_cloud->points[i].y - min_y))] =
          -tmp_cloud->points[i].z;
    }
  }

  return;
}

double cylinder_radious(pcl::PointCloud< PointType >& cloud)
{
  // estimate point normals
  pcl::search::KdTree< PointType >::Ptr tree(new pcl::search::KdTree< PointType >());
  pcl::NormalEstimation< PointType, pcl::Normal > ne;
  pcl::PointCloud< pcl::Normal >::Ptr cloud_normals(new pcl::PointCloud< pcl::Normal >);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud.makeShared());
  ne.setKSearch(80);
  ne.compute(*cloud_normals);

  // cylinder detection
  std::cout << "cylinder detection" << std::endl;
  pcl::SACSegmentationFromNormals< PointType, pcl::Normal > seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // pcl::ExtractIndices<PointType> extract;

  seg.setOptimizeCoefficients(tree);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setNormalDistanceWeight(0.1);
  seg.setMaxIterations(10000);
  seg.setDistanceThreshold(0.02);
  seg.setRadiusLimits(0, 0.3);
  seg.setInputCloud(cloud.makeShared());
  seg.setInputNormals(cloud_normals);

  std::cout << "1 : " << seg.getDistanceFromOrigin() << std::endl;

  seg.segment(*inliers, *coefficients);

  std::cout << "2 : " << seg.getDistanceFromOrigin() << std::endl;

  //    extract.setInputCloud (cloud.makeShared());
  //    extract.setIndices (inliers);
  //    extract.setNegative (false);
  //    extract.filter (c_cloud);

  return coefficients->values[6];
}

//クラスタリング
void clustering(pcl::PointCloud< PointType >& cloud, pcl::PointCloud< PointType >& c_cloud)
{
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree< PointType >::Ptr tree(new pcl::search::KdTree< PointType >);
  tree->setInputCloud(cloud.makeShared());

  std::vector< pcl::PointIndices > cluster_indices;
  pcl::EuclideanClusterExtraction< PointType > ec;
  ec.setClusterTolerance(0.04);
  ec.setMinClusterSize(500);
  ec.setMaxClusterSize(10000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud.makeShared());
  ec.extract(cluster_indices);

  std::cout << "クラスタリング開始" << std::endl;
  int i = 0;
  float colors[6][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
  for (std::vector< pcl::PointIndices >::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    // std::cout << "クラスタ重心を求める" << std::endl;
    pcl::PointCloud< PointType >::Ptr cloud_cluster(new pcl::PointCloud< PointType >);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (std::vector< int >::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      cloud_cluster->points.push_back(cloud.points[*pit]);
      inliers->indices.push_back(*pit);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    for (int j = 0; j < cloud_cluster->points.size(); j++)
    {
      cloud_cluster->points[j].r = colors[i % 6][0];
      cloud_cluster->points[j].g = colors[i % 6][1];
      cloud_cluster->points[j].b = colors[i % 6][2];
    }

    pcl::PointXYZ g = g_pos(*cloud_cluster);
    if (g.z > 0.0)
      continue;

    object m1;
    m1.cloud = *cloud_cluster;
    m1.inliers = *inliers;
    elevation_map(m1);

    m1.radious = cylinder_radious(*cloud_cluster);

    Object_Map.push_back(m1);

    c_cloud += *cloud_cluster;

    i++;
  }

  return;
}

///////////////////////////////////////////////////////
//*******************main function*******************//
///////////////////////////////////////////////////////
bool cylinder_detection(tms_msg_ss::ods_cylinder::Request& req, tms_msg_ss::ods_cylinder::Response& res)
{
  std::cout << "cylinder detection" << std::endl;

  // local variable declaration
  pcl::PointCloud< PointType >::Ptr cloud(new pcl::PointCloud< PointType >);
  pcl::PointCloud< PointType >::Ptr tmp_cloud(new pcl::PointCloud< PointType >);

  // Fill in the cloud data
  tms_msg_ss::ods_pcd srv;
  srv.request.id = 3;
  if (commander_to_kinect_capture.call(srv))
  {
    pcl::fromROSMsg(srv.response.cloud, *cloud);
  }

  // pcl::io::loadPCDFile ("src/ods_cylinder/data/ods_cylinder/sample.pcd", *cloud);
  pcl::io::savePCDFile("src/ods_cylinder/data/ods_cylinder/input.pcd", *cloud, true);

  // filtering
  passfilter(*cloud);

  // planar segmentation
  std::cout << "planar segmentation" << std::endl;
  pcl::PointCloud< PointType >::Ptr p_cloud(new pcl::PointCloud< PointType >);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);  //テーブル上面の式
  segmentate(*cloud, *p_cloud, *coefficients);

  pcl::copyPointCloud(*cloud, *tmp_cloud);
  pcl::io::savePCDFile("src/ods_cylinder/data/ods_cylinder/remove_plane.pcd", *cloud, true);
  pcl::io::savePCDFile("src/ods_cylinder/data/ods_cylinder/plane.pcd", *p_cloud, true);

  // planar transformation
  transformation(*cloud, *p_cloud, coefficients);

  // segmentation on the table
  std::cout << "segmentation" << std::endl;
  pcl::PointCloud< PointType >::Ptr c_cloud(new pcl::PointCloud< PointType >);
  clustering(*cloud, *c_cloud);
  pcl::io::savePCDFile("src/ods_cylinder/data/ods_cylinder/cluster.pcd", *c_cloud, true);

  // making elevation-map per segment

  for (int i = 0; i < Object_Map.size(); i++)
  {
    pcl::PointCloud< PointType >::Ptr tmp_cloud2(new pcl::PointCloud< PointType >);
    for (int j = 0; j < Object_Map[i].inliers.indices.size(); j++)
    {
      tmp_cloud2->push_back(tmp_cloud->points[Object_Map[i].inliers.indices[j]]);
    }

    pcl::PointXYZ g = g_pos(*tmp_cloud2);

    //絶対座標系に変換
    geometry_msgs::Pose pose;
    pose.position.x = g.x;
    pose.position.y = g.y;
    pose.position.z = g.z;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
    res.objects.poses.push_back(pose);

    if (req.id == 1)
    {
      res.radious.push_back(Object_Map[i].radious);
    }
    else if (req.id == 2)
    {
      res.radious.push_back((Object_Map[i].max_x - Object_Map[i].min_x) / 2.0);
    }

    std::stringstream ss;
    ss << "src/ods_cylinder/data/ods_cylinder/cluster_" << i << ".pcd";
    pcl::io::savePCDFile(ss.str(), *tmp_cloud2, true);
  }

  return true;
}

int main(int argc, char** argv)
{
  printf("init\n");
  ros::init(argc, argv, "ods_cylinder");
  ros::NodeHandle n;

  service = n.advertiseService("ods_cylinder_detection", cylinder_detection);

  commander_to_kinect_capture = n.serviceClient< tms_msg_ss::ods_pcd >("capture_cloud");
  ros::spin();

  return 0;
}
