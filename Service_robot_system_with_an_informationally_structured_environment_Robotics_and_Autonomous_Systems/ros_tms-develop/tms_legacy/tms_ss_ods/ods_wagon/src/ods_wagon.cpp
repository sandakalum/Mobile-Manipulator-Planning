//**************************************************************************************
//**************************************************************************************
// ods_wagon
//	ロボットによるワゴン運搬のためのロボット搭載カメラによるワゴン検出
//  黒色のグリッパを検出し、その位置からワゴンの中心位置を求める.
//  ロボット：SmartPal5
//**************************************************************************************
//**************************************************************************************

//*************************************************
//インクルードファイル
//*************************************************

// pcl関連
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

// ros関連
#include <ros/ros.h>
#include <tms_msg_ss/ods_pcd.h>
#include <tms_msg_ss/ods_wagon.h>

//************************************************
//大域変数宣言
//************************************************

// rosサービス関連
ros::ServiceServer service;
ros::ServiceClient commander_to_kinect_capture;

// カメラのロボット中心位置に対する相対距離
double C_OFFSET_X;
double C_OFFSET_Y;
double C_OFFSET_Z;
double C_OFFSET_PITCH;

// ロボット・ワゴンの位置姿勢情報
geometry_msgs::Pose2D robot;
geometry_msgs::Pose2D wagon;

// 円周率
#define PI 3.1415926

//***********************************************
// passfilter
//  入力点群から黒色の点のみを抽出
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
  pass.setFilterLimits(-0.4, 0.4);  // -0.4[m] <= x <= 0.4[m]
  pass.filter(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.4, 0.4);  // -0.4[m] <= y <= 0.4[m]
  pass.filter(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.2, 1.1);  // 0.2[m] <= z <= 1.1[m]
  pass.filter(cloud);

  // 残った点群のうち、RGB値から黒色の点のみを抽出
  for (int i = 0; i < cloud.points.size(); i++)
  {
    if (((0 <= cloud.points[i].r) && (cloud.points[i].r < 30)) &&  // 0 <= R < 30
        ((0 <= cloud.points[i].g) && (cloud.points[i].g < 30)) &&  // 0 <= G < 30
        ((0 <= cloud.points[i].b) && (cloud.points[i].b < 30)))    // 0 <= B < 30
      tmp_cloud->push_back(cloud.points[i]);
  }

  // 抽出結果を返す
  pcl::copyPointCloud(*tmp_cloud, cloud);

  return;
}

//***********************************************
// g_pos
//  入力点群の重心位置を求める
//***********************************************
pcl::PointXYZ g_pos(pcl::PointCloud< pcl::PointXYZRGB >& cloud)
{
  // 変数宣言
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

//***********************************************
// plane_detect
//  入力点群のうち、もっとも大きな平面を検出し除去
//  黒色の棚板を除去
//***********************************************
void plane_detect(pcl::PointCloud< pcl::PointXYZRGB >& cloud)
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
  while (1)
  {
    seg.setInputCloud(cloud.makeShared());
    seg.segment(*inliers, *coefficients);  // RANSACの実行

    std::cout << "plane:\n" << coefficients << std::endl;

    extract.setInputCloud(cloud.makeShared());
    extract.setIndices(inliers);

    // 平面の傾き（カメラx軸まわり）をもとめ、水平の平面であれば棚板とする
    if ((atan(-coefficients->values[2] / coefficients->values[3]) < (-1.0 * PI / 4.0)) ||
        ((1.0 * PI / 4.0) < atan(-coefficients->values[2] / coefficients->values[3])))
    {  // x軸まわりに45度以上回転なら棚板（※カメラ座標空間）
      extract.setInputCloud(cloud.makeShared());
      extract.setNegative(true);  // 入力点群を除去→true 残す→false
      extract.filter(cloud);
    }

    // 棚板ではなかったら再度平面検出
    else
    {
      extract.setNegative(true);
      extract.filter(cloud);
      break;
    }
  }

  return;
}

//***********************************************
// transformation
//  グリッパの位置をカメラ座標系から絶対座標系に変換
//***********************************************
pcl::PointXYZ transformation(pcl::PointXYZ point)
{
  std::cout << "transformation" << std::endl;

  // 変数宣言
  Eigen::Affine3f t(Eigen::Affine3f::Identity());                                             // 変換行列
  double camera_x = robot.x + C_OFFSET_X * cos(robot.theta) - C_OFFSET_Y * sin(robot.theta);  // カメラ位置 x
  double camera_y = robot.y + C_OFFSET_X * sin(robot.theta) + C_OFFSET_Y * cos(robot.theta);  // カメラ位置 y
  double camera_z = C_OFFSET_Z;                                                               // カメラ位置 z
  double camera_pitch = C_OFFSET_PITCH * (PI / 180.0);                                        // カメラ角度 pitch

  std::cout << camera_x << " " << camera_y << " " << camera_z << " " << camera_pitch << std::endl;

  // 変換行列の設定
  t(0, 0) = sin(robot.theta);
  t(0, 1) = -sin(camera_pitch) * cos(robot.theta);
  t(0, 2) = cos(camera_pitch) * cos(robot.theta);
  t(1, 0) = -cos(robot.theta);
  t(1, 1) = -sin(camera_pitch) * sin(robot.theta);
  t(1, 2) = cos(camera_pitch) * sin(robot.theta);
  t(2, 0) = 0;
  t(2, 1) = -cos(camera_pitch);
  t(2, 2) = -sin(camera_pitch);

  // 座標変換
  point = pcl::transformPoint(point, t);

  // 平行移動
  point.x += camera_x;
  point.y += camera_y;
  point.z += camera_z;

  return point;
}

//***********************************************
// clustering
//  クラスタリングを行う．クラスタはグリッパを表す
//  ワゴングリッパの検出を行う
//***********************************************
void clustering(pcl::PointCloud< pcl::PointXYZRGB >& cloud, std::vector< pcl::PointXYZ >& points)
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
  ec.setMinClusterSize(700);      // クラスタ最小点数
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
    pcl::PointXYZ g;

    // クラスタを構成する点群
    for (std::vector< int >::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      cloud_cluster->points.push_back(cloud.points[*pit]);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // クラスタの重心位置
    g = g_pos(*cloud_cluster);

    g = transformation(g);

    std::cout << "g : " << g.x << " " << g.y << " " << g.z << std::endl;
    if (g.z < 0.9)
      continue;

    for (int i = 0; i < cloud_cluster->points.size(); i++)
    {
      cloud_cluster->points[i].r = colors[m % 6][0];
      cloud_cluster->points[i].g = colors[m % 6][1];
      cloud_cluster->points[i].b = colors[m % 6][2];
    }

    points.push_back(g);

    *tmp_cloud += *cloud_cluster;

    m++;
  }

  cloud = *tmp_cloud;

  // クラスタ（グリッパ）が2つ以上ない場合
  if (m < 2)
    std::cout << "Not find two clusters!!" << std::endl;

  pcl::io::savePCDFile("src/ods_wagon/data/ods_wagon/cluster_cloud.pcd", *tmp_cloud, false);

  return;
}

//***********************************************
// swap
//  配列で指定された2つの番号の要素を入れ替える
//  検出されたグリッパの順番を入れ替える
//***********************************************
void swap(std::vector< pcl::PointXYZ >& points, std::vector< double >& distance, int i, int j)
{
  pcl::PointXYZ tmp;
  double tmp2;

  tmp = points[i];
  tmp2 = distance[i];
  points[i] = points[j];
  distance[i] = distance[j];
  points[j] = tmp;
  distance[j] = tmp2;

  return;
}

//***********************************************
// quick_sort
//  配列の要素を昇順に並べる
//  検出されたグリッパの順番を距離が近い順に並べる
//***********************************************
void quick_sort(std::vector< pcl::PointXYZ >& points, int left, int right)
{
  // 変数宣言
  int i, j;
  double pivot, dst;
  std::vector< double > distance;  // 距離値を格納

  i = left;
  j = right;

  // 距離値を保存
  for (int k = 0; k < points.size(); k++)
  {
    dst = points[k].z;
    distance.push_back(dst);
  }

  // 中間値の設定
  pivot = distance[(left + right) / 2];

  // 並び替えるべき要素を見つける
  while (1)
  {
    while (distance[i] < pivot)
      i++;

    while (pivot < distance[j])
      j--;

    if (i >= j)
      break;

    swap(points, distance, i, j);  // 要素の並べ替える
    i++;
    j--;
  }

  if (left < i - 1)
    quick_sort(points, left, i - 1);
  if (j + 1 < right)
    quick_sort(points, j + 1, right);

  return;
}

//***********************************************
// wagon_position
//  グリッパの位置からワゴン中心位置を求める
//  ワゴンの幾何情報がわかっている前提
//***********************************************
void wagon_position(std::vector< pcl::PointXYZ >& points)
{
  std::cout << "wagon_position " << points.size() << std::endl;

  double w_x = 0.0, w_y = 0.0;
  if (points.size() == 4)
  {
    for (int i = 0; i < points.size(); i++)
    {
      w_x += points[i].x;
      w_y += points[i].y;
    }
    w_x /= 4.0;
    w_y /= 4.0;

    wagon.x = w_x;
    wagon.y = w_y;

    double dis1, dis2;
    dis1 = sqrt((points[1].x - points[0].x) * (points[1].x - points[0].x) +
                (points[1].y - points[0].y) * (points[1].y - points[0].y));
    dis2 = sqrt((points[2].x - points[0].x) * (points[2].x - points[0].x) +
                (points[2].y - points[0].y) * (points[2].y - points[0].y));

    std::cout << dis1 << " " << dis2 << std::endl;

    if (dis1 < dis2)
      wagon.theta = atan2(points[2].y - points[0].y, points[2].x - points[0].x) * 180.0 / PI;
    else
      wagon.theta = atan2(points[1].y - points[0].y, points[1].x - points[0].x) * 180.0 / PI;
  }

  else if (points.size() == 3)
  {
    double dis1, dis2, dis3;
    dis1 = sqrt((points[1].x - points[0].x) * (points[1].x - points[0].x) +
                (points[1].y - points[0].y) * (points[1].y - points[0].y));
    dis2 = sqrt((points[2].x - points[0].x) * (points[2].x - points[0].x) +
                (points[2].y - points[0].y) * (points[2].y - points[0].y));
    dis3 = sqrt((points[2].x - points[1].x) * (points[2].x - points[1].x) +
                (points[2].y - points[1].y) * (points[2].y - points[1].y));

    if (0.5 < dis1)
    {
      wagon.x = (points[0].x + points[1].x) / 2.0;
      wagon.y = (points[0].y + points[1].y) / 2.0;
    }
    else if (0.5 < dis2)
    {
      wagon.x = (points[0].x + points[2].x) / 2.0;
      wagon.y = (points[0].y + points[2].y) / 2.0;
    }
    else if (0.5 < dis3)
    {
      wagon.x = (points[1].x + points[2].x) / 2.0;
      wagon.y = (points[1].y + points[2].y) / 2.0;
    }
    else
      std::cout << "No match wagon" << std::endl;

    if ((0.38 < dis1) && (dis1 < 0.46))
      wagon.theta = atan2(points[1].y - points[0].y, points[1].x - points[0].x) * 180.0 / PI;
    else if ((0.38 < dis2) && (dis2 < 0.46))
      wagon.theta = atan2(points[2].y - points[0].y, points[2].x - points[0].x) * 180.0 / PI;
    else if ((0.38 < dis3) && (dis3 < 0.46))
      wagon.theta = atan2(points[2].y - points[1].y, points[2].x - points[1].x) * 180.0 / PI;
    else
      std::cout << "No match wagon" << std::endl;
  }

  else if (points.size() == 2)
  {
    double theta;
    double x1, x2, y1, y2;
    double dis1, dis2, dis3;

    dis1 = sqrt((points[1].x - points[0].x) * (points[1].x - points[0].x) +
                (points[1].y - points[0].y) * (points[1].y - points[0].y));

    if (0.46 <= dis1)
    {
      wagon.x = (points[0].x + points[1].x) / 2.0;
      wagon.y = (points[0].y + points[1].y) / 2.0;
    }

    else if ((0.38 <= dis1) && (dis1 < 0.46))
    {
      if (points[0].x != points[1].x)
        theta = (points[1].y - points[0].y) / (points[1].x - points[0].x);
      else
        theta = 90.0;

      x1 = 0.16 * cos(PI / 2.0 - theta) + (points[0].x + points[1].x) / 2.0;
      y1 = 0.16 * sin(PI / 2.0 - theta) + (points[0].y + points[1].y) / 2.0;
      x2 = 0.16 * cos(PI / 2.0 + theta) + (points[0].x + points[1].x) / 2.0;
      y2 = 0.16 * sin(PI / 2.0 + theta) + (points[0].y + points[1].y) / 2.0;

      dis2 = sqrt((robot.x - x1) * (robot.x - x1) + (robot.y - y1) * (robot.y - y1));
      dis3 = sqrt((robot.x - x2) * (robot.x - x2) + (robot.y - y2) * (robot.y - y2));

      if (dis2 < dis3)
      {
        wagon.x = x1;
        wagon.y = y1;
        wagon.theta = theta;
      }
      else
      {
        wagon.x = x2;
        wagon.y = y2;
        wagon.theta = theta;
      }
    }

    else
    {
      if (points[0].x != points[1].x)
      {
        theta = atan2(points[1].y - points[0].y, points[1].x - points[0].x);

        x1 = 0.19 * cos(PI / 2.0 - theta) + (points[0].x + points[1].x) / 2.0;
        y1 = 0.19 * sin(PI / 2.0 - theta) + (points[0].y + points[1].y) / 2.0;
        x2 = 0.19 * cos(PI / 2.0 + theta) + (points[0].x + points[1].x) / 2.0;
        y2 = 0.19 * sin(PI / 2.0 + theta) + (points[0].y + points[1].y) / 2.0;

        dis2 = sqrt((robot.x - x1) * (robot.x - x1) + (robot.y - y1) * (robot.y - y1));
        dis3 = sqrt((robot.x - x2) * (robot.x - x2) + (robot.y - y2) * (robot.y - y2));

        if (dis2 < dis3)
        {
          wagon.x = x1;
          wagon.y = y1;
          wagon.theta = 90.0 + theta * 180.0 / PI;
        }
        else
        {
          wagon.x = x2;
          wagon.y = y2;
          wagon.theta = 90.0 + theta * 180.0 / PI;
        }
      }
      else
      {
        x1 = (points[0].x + points[1].x) / 2.0 + 0.19;
        y1 = (points[0].y + points[1].y) / 2.0;
        x2 = (points[0].x + points[1].x) / 2.0 - 0.19;
        y2 = (points[0].y + points[1].y) / 2.0;

        dis2 = sqrt((robot.x - x1) * (robot.x - x1) + (robot.y - y1) * (robot.y - y1));
        dis3 = sqrt((robot.x - x2) * (robot.x - x2) + (robot.y - y2) * (robot.y - y2));

        if (dis2 < dis3)
        {
          wagon.x = x1;
          wagon.y = y1;
          wagon.theta = 0.0;
        }
        else
        {
          wagon.x = x2;
          wagon.y = y2;
          wagon.theta = 0.0;
        }
      }
    }
  }

  else
    std::cout << "No find wagon gripper " << points.size() << std::endl;
}

bool ods_wagon(tms_msg_ss::ods_wagon::Request& req, tms_msg_ss::ods_wagon::Response& res)
{
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);

  tms_msg_ss::ods_pcd srv;
  srv.request.id = 3;
  if (commander_to_kinect_capture.call(srv))
    pcl::fromROSMsg(srv.response.cloud, *cloud);

  std::cout << "robot : (" << req.robot.x << " " << req.robot.y << " " << req.robot.theta << " )" << std::endl;

  robot.x = req.robot.x / 1000.0;
  robot.y = req.robot.y / 1000.0;
  robot.theta = req.robot.theta * PI / 180.0;
  C_OFFSET_X = 0.06;     // req.sensor.position.x;
  C_OFFSET_Y = 0.055;    // req.sensor.position.y;
  C_OFFSET_Z = 0.97;     // req.sensor.position.z;
  C_OFFSET_PITCH = 0.0;  // req.sensor.orientation.x;

  // pcl::io::loadPCDFile("src/ods_wagon/data/ods_wagon/input.pcd", *cloud);

  pcl::copyPointCloud(*cloud, *tmp_cloud);

  pcl::io::savePCDFile("src/ods_wagon/data/ods_wagon/input_xyz.pcd", *tmp_cloud, false);

  passfilter(*cloud);

  // plane_detect(*cloud);

  pcl::io::savePCDFile("src/ods_wagon/data/ods_wagon/filter.pcd", *cloud, false);

  std::vector< pcl::PointXYZ > g_cloud;

  clustering(*cloud, g_cloud);

  *tmp_cloud += *cloud;

  std::cout << "quick_sort" << std::endl;

  quick_sort(g_cloud, 0, g_cloud.size() - 1);

  for (int i = 0; i < g_cloud.size(); i++)
  {
    std::cout << g_cloud[i].x << " " << g_cloud[i].y << " " << g_cloud[i].z << std::endl;
  }

  pcl::io::savePCDFile("src/ods_wagon/data/ods_wagon/gripper.pcd", *tmp_cloud, false);

  wagon_position(g_cloud);

  res.wagon.x = wagon.x * 1000.0;
  res.wagon.y = wagon.y * 1000.0;

  while (1)
  {
    if (wagon.theta < -90.0)
      wagon.theta += 180.0;
    else if (wagon.theta >= 90.0)
      wagon.theta -= 180.0;
    else
      break;
  }
  res.wagon.theta = wagon.theta;

  std::cout << res.wagon.x << " " << res.wagon.y << " " << res.wagon.theta << std::endl;

  g_cloud.clear();

  return true;
}

int main(int argc, char** argv)
{
  std::cout << "init" << std::endl;
  ros::init(argc, argv, "ods_wagon");
  ros::NodeHandle n;

  service = n.advertiseService("ods_wagon", ods_wagon);

  commander_to_kinect_capture = n.serviceClient< tms_msg_ss::ods_pcd >("ods_capture");

  ros::spin();

  return 0;
}
