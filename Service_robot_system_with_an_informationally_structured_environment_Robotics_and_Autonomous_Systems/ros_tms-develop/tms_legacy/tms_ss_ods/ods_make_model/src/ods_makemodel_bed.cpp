#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

typedef struct
{
  double x;
  double y;
  double z;
  double depth;
  double width;
  double height;
} BED;

typedef struct
{
  double x;
  double y;
  double z;
  double yaw;
  double pitch;
  double roll;
} CAMERA;

typedef struct
{
  double x;
  double y;
  double theta;
} ROBOT;

BED Bed;
CAMERA Camera;
ROBOT Robot;

#define LOOP 20
#define BED_X 115
#define BED_Y 235
#define BED_Z 70
#define PI 3.1415926

double C_OFFSET_X = 0.0;
double C_OFFSET_Y = 0.05;
double C_OFFSET_Z = 1.08;
double C_OFFSET_PITCH = 21.0;

int Voxel[BED_X][BED_Y][BED_Z];
double OFFSET;

//重心位置を求める
void init_bedparam()
{
  Bed.x = 2.0;
  Bed.y = 2.0;
  Bed.z = 0.0;
  Bed.depth = 2.14;
  Bed.width = 1.02;
  Bed.height = 0.60;

  return;
}

void init_cameraparam(int id)
{
  switch (id)
  {
    case 1:
      Robot.x = 3.0;
      Robot.y = 0.5;
      Robot.theta = 135.0;
      break;
    case 2:
      Robot.x = 1.9;
      Robot.y = 0.5;
      Robot.theta = 87.0;
      break;
    case 3:
      Robot.x = 0.93;
      Robot.y = 0.45;
      Robot.theta = 45.0;
      break;
    case 4:
      Robot.x = 0.93;
      Robot.y = 2.0;
      Robot.theta = -1.0;
      break;
    case 5:
      Robot.x = 1.05;
      Robot.y = 3.4;
      Robot.theta = 305.0;
      break;
    case 6:
      Robot.x = 1.9;
      Robot.y = 3.35;
      Robot.theta = 270.0;
      break;
    case 7:
      Robot.x = 3.0;
      Robot.y = 3.5;
      Robot.theta = 225.0;
      break;
    case 8:
      Robot.x = 3.0;
      Robot.y = 2.0;
      Robot.theta = 180.0;
      break;
  }

  Camera.x = Robot.x + C_OFFSET_Y * cos(Robot.theta) + C_OFFSET_X * sin(Robot.theta);
  Camera.y = Robot.y + C_OFFSET_Y * sin(Robot.theta) - C_OFFSET_X * cos(Robot.theta);
  Camera.z = C_OFFSET_Z;
  Camera.pitch = C_OFFSET_PITCH * (PI / 180.0);
  Robot.theta *= PI / 180.0;

  return;
}

void downsampling(pcl::PointCloud< pcl::PointXYZRGB >& cloud, float th)
{
  pcl::VoxelGrid< pcl::PointXYZRGB > sor;
  sor.setInputCloud(cloud.makeShared());
  sor.setLeafSize(th, th, th);
  sor.filter(cloud);
}

void transformation(pcl::PointCloud< pcl::PointXYZRGB >& cloud)
{
  Eigen::Matrix4f t1(Eigen::Matrix4f::Identity());

  t1(0, 0) = sin(Robot.theta);
  t1(0, 1) = -sin(Camera.pitch) * cos(Robot.theta);
  t1(0, 2) = cos(Camera.pitch) * cos(Robot.theta);
  t1(0, 3) = Camera.x;
  t1(1, 0) = -cos(Robot.theta);
  t1(1, 1) = -sin(Camera.pitch) * sin(Robot.theta);
  t1(1, 2) = cos(Camera.pitch) * sin(Robot.theta);
  t1(1, 3) = Camera.y;
  t1(2, 0) = 0;
  t1(2, 1) = -cos(Camera.pitch);
  t1(2, 2) = -sin(Camera.pitch);
  t1(2, 3) = Camera.z;
  t1(3, 0) = 0;
  t1(3, 1) = 0;
  t1(3, 2) = 0;
  t1(3, 3) = 1;

  pcl::transformPointCloud(cloud, cloud, t1);

  return;
}

void filtering(pcl::PointCloud< pcl::PointXYZRGB >& cloud)
{
  // Create the filtering object
  pcl::PassThrough< pcl::PointXYZRGB > pass;
  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("x");
  pass.setFilterLimits(Bed.x - Bed.width / 2.0 - 0.1, Bed.x + Bed.width / 2.0 + 0.1);
  pass.filter(cloud);

  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("y");
  pass.setFilterLimits(Bed.y - Bed.depth / 2.0 - 0.1, Bed.y + Bed.depth / 2.0 + 0.1);
  pass.filter(cloud);

  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("z");
  pass.setFilterLimits(Bed.z + 0.1, Bed.z + Bed.height + 0.1);
  pass.filter(cloud);

  return;
}

void registration(pcl::PointCloud< pcl::PointXYZRGB >& src_cloud, pcl::PointCloud< pcl::PointXYZRGB >& mem_cloud,
                  Eigen::Matrix4f& m)
{
  pcl::IterativeClosestPoint< pcl::PointXYZRGB, pcl::PointXYZRGB > icp;
  icp.setInputSource(src_cloud.makeShared());
  icp.setInputTarget(mem_cloud.makeShared());
  pcl::PointCloud< pcl::PointXYZRGB > Final;
  icp.align(Final);
  m = icp.getFinalTransformation();

  pcl::transformPointCloud(src_cloud, src_cloud, m);

  return;
}

void make_voxel(pcl::PointCloud< pcl::PointXYZRGB >& cloud)
{
  std::cout << "make_voxel" << std::endl;

  int vox[BED_X][BED_Y][BED_Z];

  for (int k = 0; k < BED_Z; k++)
  {
    for (int j = 0; j < BED_Y; j++)
    {
      for (int i = 0; i < BED_X; i++)
      {
        vox[i][j][k] = 0;
      }
    }
  }

  for (int i = 0; i < cloud.points.size(); i++)
  {
    int X = 100 * (cloud.points[i].x - (Bed.x - Bed.width / 2.0 - 0.1));
    int Y = 100 * (cloud.points[i].y - (Bed.y - Bed.depth / 2.0 - 0.1));
    int Z = 100 * (cloud.points[i].z);
    // std::cout << X << " " << Y << " " << Z << std::endl;
    if (((0 <= X) && (X < BED_X)) && ((0 <= Y) && (Y < BED_Y)) && ((0 <= Z) && (Z < BED_Z)))
    {
      if (!vox[X][Y][Z])
        vox[X][Y][Z] = 1;
    }
  }

  for (int k = 0; k < BED_Z; k++)
  {
    for (int j = 0; j < BED_Y; j++)
    {
      for (int i = 0; i < BED_X; i++)
      {
        Voxel[i][j][k] += vox[i][j][k];
      }
    }
  }

  return;
}

void make_bed(pcl::PointCloud< pcl::PointXYZRGB >& model)
{
  std::cout << "make_bed" << std::endl;

  for (int k = 0; k < BED_Z; k++)
  {
    for (int j = 0; j < BED_Y; j++)
    {
      for (int i = 0; i < BED_X; i++)
      {
        if (Voxel[i][j][k] > 5)
        {
          pcl::PointXYZRGB p;
          p.x = (double)i / 100.0;
          p.y = (double)j / 100.0;
          p.z = (double)k / 100.0;
          p.r = 255;
          p.g = 0;
          p.b = 0;
          model.points.push_back(p);
        }
      }
    }
  }

  return;
}

int main()
{
  //宣言
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr model(new pcl::PointCloud< pcl::PointXYZRGB >);

  pcl::visualization::CloudViewer viewer("Cloud Viewer");

  init_bedparam();

  for (int k = 0; k < BED_Z; k++)
  {
    for (int j = 0; j < BED_Y; j++)
    {
      for (int i = 0; i < BED_X; i++)
      {
        Voxel[i][j][k] = 0;
      }
    }
  }

  float colors[8][3] = {{125, 125, 125},
                        {255, 0, 0},
                        {0, 255, 0},
                        {0, 0, 255},
                        {255, 255, 0},
                        {0, 255, 255},
                        {255, 0, 255},
                        {255, 255, 255}};

  Eigen::Matrix4f m[7];

  for (int i = 0; i < 7; i++)
  {
    m[i](0, 0) = 1;
    m[i](0, 1) = 0;
    m[i](0, 2) = 0;
    m[i](0, 3) = 0;
    m[i](1, 0) = 0;
    m[i](1, 1) = 1;
    m[i](1, 2) = 0;
    m[i](1, 3) = 0;
    m[i](2, 0) = 0;
    m[i](2, 1) = 0;
    m[i](2, 2) = 1;
    m[i](2, 3) = 0;
    m[i](3, 0) = 0;
    m[i](3, 1) = 0;
    m[i](3, 2) = 0;
    m[i](3, 3) = 1;
  }

  // transform capture data
  for (int j = 1; j < LOOP; j++)
  {
    std::cout << "LOOP : " << j << std::endl;
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr bed(new pcl::PointCloud< pcl::PointXYZRGB >);
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr target(new pcl::PointCloud< pcl::PointXYZRGB >);

    for (int i = 1; i <= 8; i++)
    {
      std::stringstream ss;
      ss << "src/ods_make_model/data/bed/input" << i << "_" << j << ".pcd";
      pcl::io::loadPCDFile(ss.str(), *cloud);

      init_cameraparam(i);

      downsampling(*cloud, 0.01);

      transformation(*cloud);

      // filtering(*cloud);

      for (int k = 0; k < cloud->points.size(); k++)
      {
        cloud->points[k].r = colors[i - 1][0];
        cloud->points[k].g = colors[i - 1][1];
        cloud->points[k].b = colors[i - 1][2];
      }

      if (j == 1)
      {
        if (i > 1)
        {
          Eigen::Matrix4f m1;
          std::cout << "registration "
                    << " " << i << " -> " << i - 1 << std::endl;
          pcl::PointCloud< pcl::PointXYZRGB >::Ptr rgs_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
          int a = 0;
          while (a < 5)
          {
            std::cout << "a = " << a << std::endl;
            // registration(*cloud, *target, m1);

            // m[i-2] *= m1;

            // if((double)(m1(0,0)+m1(1,1)+m1(2,2)+m1(3,3)) >= 4)
            a++;

            *rgs_cloud = *cloud + *target;

            viewer.showCloud(rgs_cloud);
          }

          std::stringstream ss1;
          ss1 << "src/ods_make_model/data/bed/rgs_" << i << "_to_" << 1 << ".pcd";
          pcl::io::savePCDFile(ss1.str(), *rgs_cloud, false);
        }
        if (i == 1)
          pcl::copyPointCloud(*cloud, *target);
      }

      /*else{
          if(i > 1) pcl::transformPointCloud(*cloud, *cloud, m[i-2]);
      }*/

      *bed += *cloud;
    }

    std::stringstream ss2;
    ss2 << "src/ods_make_model/data/bed/bed_" << j << ".pcd";
    pcl::io::savePCDFile(ss2.str(), *bed, false);

    make_voxel(*bed);

    // std::stringstream ss;
    // ss << "src/ods_make_model/data/bed/bed_filter_" << j << ".pcd";
    // pcl::io::savePCDFile(ss.str(), *bed, false);
  }

  make_bed(*model);

  Eigen::Matrix4f t1(Eigen::Matrix4f::Identity());

  t1(0, 0) = 1;
  t1(0, 1) = 0;
  t1(0, 2) = 0;
  t1(0, 3) = -Bed.width / 2.0 - 0.1;
  t1(1, 0) = 0;
  t1(1, 1) = 1;
  t1(1, 2) = 0;
  t1(1, 3) = -Bed.depth / 2.0 - 0.1;
  t1(2, 0) = 0;
  t1(2, 1) = 0;
  t1(2, 2) = 1;
  t1(2, 3) = -0.1;
  t1(3, 0) = 0;
  t1(3, 1) = 0;
  t1(3, 2) = 0;
  t1(3, 3) = 1;

  pcl::transformPointCloud(*model, *model, t1);

  pcl::io::savePCDFile("src/ods_make_model/data/bed/model_bed.pcd", *model, true);

  while (!viewer.wasStopped())
  {
    viewer.showCloud(model);
  }
  return 0;
}
