#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#define SMARTPAL4 1
#define SMARTPAL5 0

double C_OFFSET_X;
double C_OFFSET_Y;
double C_OFFSET_Z;
double C_OFFSET_YAW;

int main(int argc, char **argv)
{
  //**************************
  // initialize
  //**************************
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr model1(new pcl::PointCloud< pcl::PointXYZRGB >);  // table1
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr model2(new pcl::PointCloud< pcl::PointXYZRGB >);  // table2
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr model3(new pcl::PointCloud< pcl::PointXYZRGB >);  // small shelf
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr model4(new pcl::PointCloud< pcl::PointXYZRGB >);  // big shelf
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr model5(new pcl::PointCloud< pcl::PointXYZRGB >);  // chair
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr model6(new pcl::PointCloud< pcl::PointXYZRGB >);  // chair
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr model7(new pcl::PointCloud< pcl::PointXYZRGB >);  // bed
  //**************************
  // input cloud
  //**************************
  pcl::io::loadPCDFile("src/ods_register_capture/data/place_model/sample1.pcd", *cloud);
  pcl::io::loadPCDFile("src/ods_register_capture/data/place_model/model_table.pcd", *model1);
  pcl::io::loadPCDFile("src/ods_register_capture/data/place_model/model_table2.pcd", *model2);
  pcl::io::loadPCDFile("src/ods_register_capture/data/place_model/model_shelf.pcd", *model3);
  pcl::io::loadPCDFile("src/ods_register_capture/data/place_model/model_bigshelf.pcd", *model4);
  pcl::io::loadPCDFile("src/ods_register_capture/data/place_model/model_chair.pcd", *model5);
  pcl::io::loadPCDFile("src/ods_register_capture/data/place_model/model_bed.pcd", *model7);

  for (int i = 0; i < model1->points.size(); i++)
  {
    model1->points[i].r = 255;
    model1->points[i].g = 0;
    model1->points[i].b = 255;
  }
  for (int i = 0; i < model2->points.size(); i++)
  {
    model2->points[i].r = 255;
    model2->points[i].g = 0;
    model2->points[i].b = 0;
  }
  for (int i = 0; i < model3->points.size(); i++)
  {
    model3->points[i].r = 0;
    model3->points[i].g = 255;
    model3->points[i].b = 0;
  }
  for (int i = 0; i < model4->points.size(); i++)
  {
    model4->points[i].r = 0;
    model4->points[i].g = 0;
    model4->points[i].b = 255;
  }
  for (int i = 0; i < model5->points.size(); i++)
  {
    model5->points[i].r = 255;
    model5->points[i].g = 255;
    model5->points[i].b = 0;
  }
  for (int i = 0; i < model7->points.size(); i++)
  {
    model7->points[i].r = 0;
    model7->points[i].g = 255;
    model7->points[i].b = 255;
  }

  *model6 = *model5;

  double robot_x = 3.000;
  double robot_y = 2.000;
  double robot_theta = 180.0;

  //絶対座標空間におけるカメラの位置を求める
  double camera_x = robot_x + C_OFFSET_Y * cos(robot_theta) + C_OFFSET_X * sin(robot_theta);
  double camera_y = robot_y + C_OFFSET_Y * sin(robot_theta) - C_OFFSET_X * cos(robot_theta);
  double camera_z = C_OFFSET_Z;
  double camera_yaw = C_OFFSET_YAW;
  std::cout << "(" << camera_x << ", " << camera_y << ", " << camera_z << ", " << camera_yaw << ")" << std::endl;

  //モデルをカメラから見た姿勢に回転
  Eigen::Matrix4f t1(Eigen::Matrix4f::Identity());

  t1(0, 0) = 1;
  t1(0, 1) = 0;
  t1(0, 2) = 0;
  t1(0, 3) = 1.4;
  t1(1, 0) = 0;
  t1(1, 1) = -1;
  t1(1, 2) = 0;
  t1(1, 3) = 1.9;
  t1(2, 0) = 0;
  t1(2, 1) = 0;
  t1(2, 2) = -1;
  t1(2, 3) = 0.7;
  t1(3, 0) = 0;
  t1(3, 1) = 0;
  t1(3, 2) = 0;
  t1(3, 3) = 1;

  pcl::transformPointCloud(*model1, *model1, t1);

  Eigen::Matrix4f t2(Eigen::Matrix4f::Identity());

  t2(0, 0) = 0;
  t2(0, 1) = 1;
  t2(0, 2) = 0;
  t2(0, 3) = 4.23;
  t2(1, 0) = 1;
  t2(1, 1) = 0;
  t2(1, 2) = 0;
  t2(1, 3) = 2.25;
  t2(2, 0) = 0;
  t2(2, 1) = 0;
  t2(2, 2) = -1;
  t2(2, 3) = 0.75;
  t2(3, 0) = 0;
  t2(3, 1) = 0;
  t2(3, 2) = 0;
  t2(3, 3) = 1;

  pcl::transformPointCloud(*model2, *model2, t2);

  Eigen::Matrix4f t3(Eigen::Matrix4f::Identity());

  t3(0, 0) = 0;
  t3(0, 1) = 0;
  t3(0, 2) = 1;
  t3(0, 3) = 3.925;
  t3(1, 0) = -1;
  t3(1, 1) = 0;
  t3(1, 2) = 0;
  t3(1, 3) = 1.41;
  t3(2, 0) = 0;
  t3(2, 1) = -1;
  t3(2, 2) = 0;
  t3(2, 3) = 0.91;
  t3(3, 0) = 0;
  t3(3, 1) = 0;
  t3(3, 2) = 0;
  t3(3, 3) = 1;

  pcl::transformPointCloud(*model3, *model3, t3);

  Eigen::Matrix4f t4(Eigen::Matrix4f::Identity());

  t4(0, 0) = 0;
  t4(0, 1) = 0;
  t4(0, 2) = 1;
  t4(0, 3) = 3.99;
  t4(1, 0) = -1;
  t4(1, 1) = 0;
  t4(1, 2) = 0;
  t4(1, 3) = 0.15;
  t4(2, 0) = 0;
  t4(2, 1) = -1;
  t4(2, 2) = 0;
  t4(2, 3) = 1.81;
  t4(3, 0) = 0;
  t4(3, 1) = 0;
  t4(3, 2) = 0;
  t4(3, 3) = 1;

  pcl::transformPointCloud(*model4, *model4, t4);

  Eigen::Matrix4f t5(Eigen::Matrix4f::Identity());

  t5(0, 0) = -1;
  t5(0, 1) = 0;
  t5(0, 2) = 0;
  t5(0, 3) = 1.4;
  t5(1, 0) = 0;
  t5(1, 1) = 1;
  t5(1, 2) = 0;
  t5(1, 3) = 2.6;
  t5(2, 0) = 0;
  t5(2, 1) = 0;
  t5(2, 2) = -1;
  t5(2, 3) = 0.43;
  t5(3, 0) = 0;
  t5(3, 1) = 0;
  t5(3, 2) = 0;
  t5(3, 3) = 1;

  pcl::transformPointCloud(*model5, *model5, t5);

  Eigen::Matrix4f t6(Eigen::Matrix4f::Identity());

  t6(0, 0) = 0;
  t6(0, 1) = -1;
  t6(0, 2) = 0;
  t6(0, 3) = 3.9;
  t6(1, 0) = -1;
  t6(1, 1) = 0;
  t6(1, 2) = 0;
  t6(1, 3) = 2.25;
  t6(2, 0) = 0;
  t6(2, 1) = 0;
  t6(2, 2) = -1;
  t6(2, 3) = 0.43;
  t6(3, 0) = 0;
  t6(3, 1) = 0;
  t6(3, 2) = 0;
  t6(3, 3) = 1;

  pcl::transformPointCloud(*model6, *model6, t6);

  Eigen::Matrix4f t7(Eigen::Matrix4f::Identity());

  t7(0, 0) = 0;
  t7(0, 1) = 1;
  t7(0, 2) = 0;
  t7(0, 3) = 3.5;
  t7(1, 0) = -1;
  t7(1, 1) = 0;
  t7(1, 2) = 0;
  t7(1, 3) = 3.5;
  t7(2, 0) = 0;
  t7(2, 1) = 0;
  t7(2, 2) = 1;
  t7(2, 3) = 0;
  t7(3, 0) = 0;
  t7(3, 1) = 0;
  t7(3, 2) = 0;
  t7(3, 3) = 1;

  pcl::transformPointCloud(*model7, *model7, t7);

  pcl::io::savePCDFile("src/ods_register_capture/data/place_model/new_model_table.pcd", *model1);
  pcl::io::savePCDFile("src/ods_register_capture/data/place_model/new_model_table2.pcd", *model2);
  pcl::io::savePCDFile("src/ods_register_capture/data/place_model/new_model_shelf.pcd", *model3);
  pcl::io::savePCDFile("src/ods_register_capture/data/place_model/new_model_bigshelf.pcd", *model4);
  pcl::io::savePCDFile("src/ods_register_capture/data/place_model/new_model_chair.pcd", *model5);
  pcl::io::savePCDFile("src/ods_register_capture/data/place_model/new_model_chair2.pcd", *model6);

  *model1 += *model2;
  *model1 += *model3;
  *model1 += *model4;
  *model1 += *model7;

  pcl::io::savePCDFile("src/ods_register_capture/data/place_model/model_environment1.pcd", *model1);

  *model1 += *model5;
  *model1 += *model6;

  pcl::io::savePCDFile("src/ods_register_capture/data/place_model/model_environment2.pcd", *model1);

  return 0;
}
