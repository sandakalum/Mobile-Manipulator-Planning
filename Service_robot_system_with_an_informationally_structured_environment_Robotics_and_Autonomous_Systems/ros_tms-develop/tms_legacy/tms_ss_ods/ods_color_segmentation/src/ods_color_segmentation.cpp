#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>

double C_OFFSET_X = 0.0;
double C_OFFSET_Y = 0.05;
double C_OFFSET_Z = 1.0;
double C_OFFSET_YAW = 21.0;

#define PI 3.1415926535

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
  double theta;
} Robot;

typedef struct
{
  double x;
  double y;
  double z;
  double theta;
} Furniture;

Model m_size;
Robot robot;
Furniture bed;

void clustering(pcl::PointCloud< pcl::PointXYZRGB >& f_cloud, pcl::PointCloud< pcl::PointXYZRGB >& cloud)
{
  std::cout << "clustering" << std::endl;

  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree< pcl::PointXYZRGB >::Ptr tree(new pcl::search::KdTree< pcl::PointXYZRGB >);
  tree->setInputCloud(f_cloud.makeShared());

  std::vector< pcl::PointIndices > cluster_indices;
  pcl::EuclideanClusterExtraction< pcl::PointXYZRGB > ec;
  ec.setClusterTolerance(0.03);
  ec.setMinClusterSize(10000);
  ec.setMaxClusterSize(200000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(f_cloud.makeShared());
  ec.extract(cluster_indices);

  // std::cout << "クラスタリング開始" << std::endl;
  int n = 0;
  float colors[6][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
  for (std::vector< pcl::PointIndices >::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

    std::cout << "part1" << std::endl;
    for (std::vector< int >::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      // cloud_cluster->points.push_back (f_cloud.points[*pit]);
      cloud.points[*pit].r = colors[n % 6][0];
      cloud.points[*pit].g = colors[n % 6][1];
      cloud.points[*pit].b = colors[n % 6][2];
    }
    // cloud_cluster->width = cloud_cluster->points.size ();
    // cloud_cluster->height = 1;
    // cloud_cluster->is_dense = true;

    /*for(int i=0;i<cloud_cluster->points.size();i++){
        cloud_cluster->points[i].r = colors[n%6][0];
        cloud_cluster->points[i].g = colors[n%6][1];
        cloud_cluster->points[i].b = colors[n%6][2];
    }*/
    //*tmp_cloud += *cloud_cluster;

    n++;
  }

  // pcl::copyPointCloud(*tmp_cloud, cloud);

  return;
}

void transformation(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZRGB >& model)
{
  std::cout << "transformation" << std::endl;

  double camera_x = robot.x + C_OFFSET_Y * cos(robot.theta) + C_OFFSET_X * sin(robot.theta);
  double camera_y = robot.y + C_OFFSET_Y * sin(robot.theta) - C_OFFSET_X * cos(robot.theta);
  double camera_z = C_OFFSET_Z;
  double camera_yaw = C_OFFSET_YAW * (PI / 180.0);

  bed.theta = 90.0;

  robot.theta *= PI / 180.0;
  bed.theta *= PI / 180.0;

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

  pcl::transformPointCloud(cloud, cloud, t1);

  t1(0, 0) = cos(bed.theta);
  t1(0, 1) = sin(bed.theta);
  t1(0, 2) = 0;
  t1(0, 3) = bed.x;
  t1(1, 0) = -sin(bed.theta);
  t1(1, 1) = cos(bed.theta);
  t1(1, 2) = 0;
  t1(1, 3) = bed.y;
  t1(2, 0) = 0;
  t1(2, 1) = 0;
  t1(2, 2) = 1;
  t1(2, 3) = bed.z;
  t1(3, 0) = 0;
  t1(3, 1) = 0;
  t1(3, 2) = 0;
  t1(3, 3) = 1;

  pcl::transformPointCloud(model, model, t1);

  return;
}

void filtering(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZRGB >& model)
{
  std::cout << "filtering" << std::endl;

  m_size.min_x = 1000.0;
  m_size.max_x = -1000.0;
  m_size.min_y = 1000.0;
  m_size.max_y = -1000.0;
  m_size.min_z = 1000.0;
  m_size.max_z = -1000.0;

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

  pcl::PassThrough< pcl::PointXYZRGB > pass;
  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("x");
  pass.setFilterLimits(m_size.min_x, 4.0 /*m_size.max_x*/);
  pass.filter(cloud);

  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("y");
  pass.setFilterLimits(m_size.min_y, m_size.max_y);
  pass.filter(cloud);

  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("z");
  pass.setFilterLimits(m_size.min_z, m_size.max_z + 0.2);
  pass.filter(cloud);

  return;
}

int main()
{
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr model(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr rgb_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointXYZRGB tmp;

  pcl::io::loadPCDFile("src/ods_color_segmentation/data/ods_color_segmentation/sample.pcd", *cloud);
  pcl::io::loadPCDFile("src/ods_color_segmentation/data/ods_color_segmentation/model_bed.pcd", *model);

  robot.x = 2.4;
  robot.y = 2.6;
  robot.theta = 50.0;
  bed.x = 3.5;
  bed.y = 3.5;
  bed.z = 0.0;
  bed.theta = 0.0;

  transformation(*cloud, *model);

  pcl::io::savePCDFile("src/ods_color_segmentation/data/ods_color_segmentation/transformed.pcd", *cloud);
  pcl::io::savePCDFile("src/ods_color_segmentation/data/ods_color_segmentation/transformed_model.pcd", *model);

  filtering(*cloud, *model);

  pcl::io::savePCDFile("src/ods_color_segmentation/data/ods_color_segmentation/filtered.pcd", *cloud);

  for (int i = 0; i < cloud->points.size(); i++)
  {
    tmp.x = cloud->points[i].r / 100.0;
    tmp.y = cloud->points[i].g / 100.0;
    tmp.z = cloud->points[i].b / 100.0;
    tmp.r = cloud->points[i].r;
    tmp.g = cloud->points[i].g;
    tmp.b = cloud->points[i].b;
    rgb_cloud->push_back(tmp);
  }
  pcl::io::savePCDFile("src/ods_color_segmentation/data/ods_color_segmentation/rgb_cloud.pcd", *rgb_cloud);

  clustering(*rgb_cloud, *cloud);

  pcl::io::savePCDFile("src/ods_color_segmentation/data/ods_color_segmentation/cluster.pcd", *cloud);

  return 0;
}
