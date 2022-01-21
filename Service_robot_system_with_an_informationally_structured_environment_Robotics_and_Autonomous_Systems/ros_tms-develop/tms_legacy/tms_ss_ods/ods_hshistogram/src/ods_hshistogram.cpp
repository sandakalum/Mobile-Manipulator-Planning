#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types_conversion.h>

#include "ros/ros.h"
#include <tms_msg_ss/ods_pcd.h>

#define MAP_H 32
#define MAP_S 32

typedef struct
{
  pcl::PointCloud< pcl::PointXYZRGB > cloud_rgb;
  pcl::PointCloud< pcl::PointXYZHSV > cloud_hsv;
  int histogram[MAP_H][MAP_S];
} object_map;

typedef struct
{
  int index_h;
  int index_s;
  int value;
} TYPE;

//***********************************************
// passfilter
//
//***********************************************
void passfilter(pcl::PointCloud< pcl::PointXYZRGB >& cloud)
{
  std::cout << "passfilter" << std::endl;

  // 変数宣言
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PassThrough< pcl::PointXYZRGB > pass;  // パスフィルタをもちいるためのクラス

  // ワゴンはロボットの正面近くにあると仮定し、カメラ座標空間でフィルタをかける
  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-0.8, 0.8);  // -0.8[m] <= x <= 0.8[m]
  pass.filter(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.5, 0.5);  // -0.5[m] <= y <= 0.5[m]
  pass.filter(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.4, 1.5);  // 0.4[m] <= z <= 1.1[m]
  pass.filter(cloud);

  return;
}

//***********************************************
// plane_detect
//  入力点群のうち、もっとも大きな平面を検出し除去
//
//***********************************************
void plane_detect(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZRGB >& plane)
{
  // 変数宣言
  pcl::SACSegmentation< pcl::PointXYZRGB > seg;                          // RANSACを用いるためのクラス
  pcl::ExtractIndices< pcl::PointXYZRGB > extract;                       // 点群除去のためのクラス
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);  // 検出された平面のパラメータ
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  // 検出された平面を構成する点群の配列番号

  // RANSACの設定
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);  // モデルの設定
  seg.setMethodType(pcl::SAC_RANSAC);     // 手法の設定
  seg.setDistanceThreshold(0.01);         // 平面の許容誤差

  // 平面検出
  //   検出された平面のパラメータから棚板だと思われるものが見つかるまで平面検出を繰り返す
  seg.setInputCloud(cloud.makeShared());
  seg.segment(*inliers, *coefficients);  // RANSACの実行

  std::cout << "plane:\n" << coefficients << std::endl;

  extract.setInputCloud(cloud.makeShared());
  extract.setIndices(inliers);
  extract.setInputCloud(cloud.makeShared());
  extract.setNegative(false);
  extract.filter(plane);
  extract.setNegative(true);  // 入力点群を除去→true 残す→false
  extract.filter(cloud);

  return;
}

//***********************************************
// clustering
//  クラスタリングを行う．
//
//***********************************************
void segmentation(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZHSV >& hsv,
                  std::vector< object_map >& map)
{
  std::cout << "clustering" << std::endl;

  // 変数宣言
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::search::KdTree< pcl::PointXYZRGB >::Ptr tree(new pcl::search::KdTree< pcl::PointXYZRGB >);
  pcl::EuclideanClusterExtraction< pcl::PointXYZRGB > ec;  // クラスタリングのためのクラス
  std::vector< pcl::PointIndices > cluster_indices;  // 各クラスタを構成する点の点群番号を格納した配列

  tree->setInputCloud(cloud.makeShared());  // Kdtreeの作成

  // クラスタリングの設定
  ec.setClusterTolerance(0.015);  // 同一クラスタとみなす点間距離の設定
  ec.setMinClusterSize(300);      // クラスタ最小点数
  ec.setMaxClusterSize(5000);     // クラスタ最大点数
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud.makeShared());
  ec.extract(cluster_indices);  // クラスタリングの実行

  // クラスタリング後の処理
  int m = 0;
  float colors[6][3] = {
      {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};  // クラスタの色
  for (std::vector< pcl::PointIndices >::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_cluster(new pcl::PointCloud< pcl::PointXYZRGB >);
    pcl::PointCloud< pcl::PointXYZHSV >::Ptr cloud_cluster_hsv(new pcl::PointCloud< pcl::PointXYZHSV >);
    object_map tmp;

    for (int i = 0; i < MAP_H; i++)
    {
      for (int j = 0; j < MAP_S; j++)
      {
        tmp.histogram[i][j] = 0;
      }
    }

    // クラスタを構成する点群
    for (std::vector< int >::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      cloud_cluster->points.push_back(cloud.points[*pit]);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    pcl::PointCloudXYZRGBtoXYZHSV(*cloud_cluster, *cloud_cluster_hsv);

    tmp.cloud_rgb = *cloud_cluster;
    tmp.cloud_hsv = *cloud_cluster_hsv;

    /*for(int i=0;i<cloud_cluster_hsv->points.size();i++)
        tmp.histogram[cloud_cluster_hsv->points[i].h/32][cloud_cluster_hsv->points[i].s/32] += 1;

    map.push_back(tmp);*/

    std::stringstream ss;
    ss << "src/ods_hshistogram/data/hshistogram/cluster_" << m + 1 << ".pcd";
    pcl::io::savePCDFile(ss.str(), *cloud_cluster, true);

    for (int i = 0; i < cloud_cluster->points.size(); i++)
    {
      cloud_cluster->points[i].r = colors[m % 6][0];
      cloud_cluster->points[i].g = colors[m % 6][1];
      cloud_cluster->points[i].b = colors[m % 6][2];
    }

    *tmp_cloud += *cloud_cluster;
    hsv += *cloud_cluster_hsv;

    m++;
  }

  cloud = *tmp_cloud;

  pcl::io::savePCDFile("src/ods_wagon/data/ods_wagon/cluster_cloud.pcd", *tmp_cloud, false);

  return;
}

void compare_histogram(object_map object1, object_map object2)
{
  int distance = 0;
  TYPE max1, max2;
  TYPE min1, min2;

  max1.value = -1;
  max2.value = -1;
  max2.value = 1000000;
  max2.value = 1000000;

  for (int i = 0; i < MAP_H; i++)
  {
    for (int j = 0; j < MAP_S; j++)
    {
      distance += abs(object1.histogram[i][j] - object2.histogram[i][j]);

      if (object1.histogram[i][j] > max1.value)
      {
        max1.index_h = i;
        max1.index_s = j;
        max1.value = object1.histogram[i][j];
      }
      if (object1.histogram[i][j] < min1.value)
      {
        min1.index_h = i;
        min1.index_s = j;
        min1.value = object1.histogram[i][j];
      }
      if (object2.histogram[i][j] > max2.value)
      {
        max2.index_h = i;
        max2.index_s = j;
        max2.value = object2.histogram[i][j];
      }
      if (object2.histogram[i][j] < min2.value)
      {
        min2.index_h = i;
        min2.index_s = j;
        min2.value = object2.histogram[i][j];
      }
    }
  }

  FILE* fp, *fp2;

  if ((fp = fopen("src/ods_hshistogram/data/hshistogram/histogram.txt", "r")) == NULL)
  {
    std::cout << "fp file open error!!" << std::endl;
    return;
  }
  if ((fp2 = fopen("src/ods_hshistogram/data/hshistogram/histogram2.txt", "r")) == NULL)
  {
    std::cout << "fp2 file open error!!" << std::endl;
    return;
  }

  for (int i = 0; i < MAP_H; i++)
  {
    for (int j = 0; j < MAP_S; j++)
    {
      fprintf(fp, "%d ", object1.histogram[i][j]);
      fprintf(fp2, "%d ", object2.histogram[i][j]);
    }
    fprintf(fp, "\n");
    fprintf(fp2, "\n");
  }

  std::cout << "max1 : " << max1.index_h << " " << max1.index_s << " " << max1.value << std::endl;
  std::cout << "min1 : " << min1.index_h << " " << min1.index_s << " " << min1.value << std::endl;
  std::cout << "max2 : " << max2.index_h << " " << max2.index_s << " " << max2.value << std::endl;
  std::cout << "min2 : " << min2.index_h << " " << min2.index_s << " " << min2.value << std::endl;
  std::cout << distance << std::endl;

  return;
}

int main(int argc, char** argv)
{
  std::cout << "init" << std::endl;
  ros::init(argc, argv, "ods_hshistogram");
  ros::NodeHandle n;
  ros::ServiceClient commander_to_kinect_capture;
  tms_msg_ss::ods_pcd srv;
  std::vector< object_map > Object_Map1, Object_Map2;

  pcl::PointCloud< pcl::PointXYZRGB >::Ptr rgb_cloud1(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr rgb_cloud2(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZHSV >::Ptr hsv_cloud1(new pcl::PointCloud< pcl::PointXYZHSV >);
  pcl::PointCloud< pcl::PointXYZHSV >::Ptr hsv_cloud2(new pcl::PointCloud< pcl::PointXYZHSV >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr plane_cloud1(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr plane_cloud2(new pcl::PointCloud< pcl::PointXYZRGB >);

  /*commander_to_kinect_capture = n.serviceClient<tms_msg_ss::ods_pcd>("ods_capture");

  std::cout << "first capture" << std::endl;
  srv.request.id = 3;
  if(commander_to_kinect_capture.call(srv)){
      pcl::fromROSMsg(srv.response.cloud, *rgb_cloud1);
  }
  pcl::io::savePCDFile("src/ods_hshistogram/data/hshistogram/input1.pcd", *rgb_cloud1);

  std::cout << "second capture" << std::endl;
  int a;
  scanf("%d,"&a);
  if(commander_to_kinect_capture.call(srv)){
      pcl::fromROSMsg(srv.response.cloud, *rgb_cloud2);
  }
  pcl::io::savePCDFile("src/ods_hshistogram/data/hshistogram/input2.pcd", *rgb_cloud2);*/

  pcl::io::loadPCDFile("src/ods_hshistogram/data/hshistogram/input1.pcd", *rgb_cloud1);
  pcl::io::loadPCDFile("src/ods_hshistogram/data/hshistogram/input2.pcd", *rgb_cloud2);

  passfilter(*rgb_cloud1);
  passfilter(*rgb_cloud2);
  pcl::io::savePCDFile("src/ods_hshistogram/data/hshistogram/filter1.pcd", *rgb_cloud1);
  pcl::io::savePCDFile("src/ods_hshistogram/data/hshistogram/filter2.pcd", *rgb_cloud2);

  plane_detect(*rgb_cloud1, *plane_cloud1);
  plane_detect(*rgb_cloud2, *plane_cloud2);
  pcl::io::savePCDFile("src/ods_hshistogram/data/hshistogram/remove_plane1.pcd", *rgb_cloud1);
  pcl::io::savePCDFile("src/ods_hshistogram/data/hshistogram/remove_plane2.pcd", *rgb_cloud2);

  segmentation(*rgb_cloud1, *hsv_cloud1, Object_Map1);
  segmentation(*rgb_cloud2, *hsv_cloud2, Object_Map2);

  pcl::io::savePCDFile("src/ods_hshistogram/data/hshistogram/rgb_cluster1.pcd", *rgb_cloud1);
  pcl::io::savePCDFile("src/ods_hshistogram/data/hshistogram/rgb_cluster2.pcd", *rgb_cloud2);
  pcl::io::savePCDFile("src/ods_hshistogram/data/hshistogram/hsv_cluster1.pcd", *hsv_cloud1);
  pcl::io::savePCDFile("src/ods_hshistogram/data/hshistogram/hsv_cluster2.pcd", *hsv_cloud2);

  // compare_histogram(Object_Map1, Object_Map2);

  return 0;
}
