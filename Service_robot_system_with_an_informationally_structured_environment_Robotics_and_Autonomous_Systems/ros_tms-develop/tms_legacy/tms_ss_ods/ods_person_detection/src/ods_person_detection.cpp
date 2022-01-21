#include <ods_person_detection/person_detection.h>

//*******************************
//**Y軸フィルター
//*******************************
void passfilter(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZRGB >& cloud_out,
                std::vector< int >& index)
{
  std::cout << "passfilter" << std::endl;

  //変数宣言
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  std::vector< int > tmp_index;

  // NaN値を除去
  pcl::removeNaNFromPointCloud(cloud, *tmp_cloud, tmp_index);

  //フィルタリング
  for (int i = 0; i < tmp_cloud->size(); i++)
  {
    if ((-0.20 < tmp_cloud->points[i].y) && (tmp_cloud->points[i].y < 0.30))
    {
      cloud_out.points.push_back(tmp_cloud->points[i]);
      index.push_back(tmp_index[i]);
    }
  }

  return;
}

//*******************************
//**セグメントの端点を求める
//*******************************
Endpoints endpoint(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointIndices& inlier)
{
  //変数宣言
  Endpoints e;
  pcl::PointXYZRGB p1, p2;  // p1:左側　p2：右側
  int inlier1 = 0, inlier2 = 0, count = 0;
  double n_x = 0.0, n_y = 0.0, n_z = 0.0, n_c = 0.0;

  p1.x = 100.0, p2.x = -100.0;
  e.cloud = cloud;

  //法線情報を付加
  /*pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::Normal n;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.03);
  ne.compute (*cloud_normals);

  for(int i=0;i<cloud_normals->points.size();i++){
      if(n_x != 0.0){
          n_x += cloud_normals->points[i].normal_x;
          n_y += cloud_normals->points[i].normal_y;
          n_z += cloud_normals->points[i].normal_z;
          n_c += cloud_normals->points[i].curvature;
          count++;
      }
      else{
          std::cout << "no normal vector" << std::endl;
      }
  }

  n.normal_x = n_x / (double)count;
  n.normal_y = n_y / (double)count;
  n.normal_z = n_z / (double)count;
  n.curvature = n_c / (double)count;

  e.normal = n;*/

  //端点を求める
  for (int i = 0; i < cloud.points.size(); i++)
  {
    if (cloud.points[i].x < p1.x)
    {
      p1 = cloud.points[i];
      inlier1 = i;
    }
    if (p2.x < cloud.points[i].x)
    {
      p2 = cloud.points[i];
      inlier2 = i;
    }
  }
  e.inlier1 = inlier.indices[inlier1];
  e.inlier2 = inlier.indices[inlier2];
  e.p1 = p1;
  e.p2 = p2;

  return e;
}

//*******************************
//**オクルージョンの判定
//*******************************
int check_occlusion(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointXYZRGB p1, pcl::PointXYZRGB p2, int inlier1,
                    int inlier2)
{
  //変数宣言
  double a, b;
  int x1, y1, x2, y2;
  int count = 0, check = 0;  // count：端点間の画素距離，check：端点間の画素で端点より距離値が小さい画素数

  //オクルージョン判定
  //入力された端点のX(水平方向)が同値の場合
  if (p1.x == p2.x)
  {
    //入力された端点が一致している場合
    if (p1.y == p2.y)
    {
      return 1;
    }
    //端点間の画素を調べる
    else if (p1.y > p2.y)
    {
      x1 = inlier1 % IMAGE_WIDTH;
      y1 = inlier1 / IMAGE_WIDTH;
      x2 = inlier2 % IMAGE_WIDTH;
      y2 = inlier2 / IMAGE_WIDTH;
      while (y1 > y2)
      {
        count++;
        if ((cloud.points[y1 * IMAGE_WIDTH + x1].z <= p1.z) || (cloud.points[y1 * IMAGE_WIDTH + x1].z <= p2.z))
        {
          check++;
        }
        y1++;
      }
    }
    else
    {
      x1 = inlier1 % IMAGE_WIDTH;
      y1 = inlier1 / IMAGE_WIDTH;
      x2 = inlier2 % IMAGE_WIDTH;
      y2 = inlier2 / IMAGE_WIDTH;
      while (y1 < y2)
      {
        count++;
        if ((cloud.points[y1 * IMAGE_WIDTH + x1].z <= p1.z) || (cloud.points[y1 * IMAGE_WIDTH + x1].z <= p2.z))
        {
          check++;
        }
        y1--;
      }
    }
  }

  else
  {
    a = (p2.y - p1.y) / (p2.x - p1.x);
    b = (p2.x * p1.y - p1.x * p2.y) / (p2.x - p1.x);
    x1 = inlier1 % IMAGE_WIDTH;
    y1 = inlier1 / IMAGE_WIDTH;
    x2 = inlier2 % IMAGE_WIDTH;
    y2 = inlier2 / IMAGE_WIDTH;

    while (x1 <= x2)
    {
      if ((int)cloud.points[y1 * IMAGE_WIDTH + x1].z >= 0)
      {
        count++;
        if ((cloud.points[y1 * IMAGE_WIDTH + x1].z <= p1.z) || (cloud.points[y1 * IMAGE_WIDTH + x1].z <= p2.z))
        {
          check++;
        }
        else
        {
          std::cout << (double)cloud.points[y1 * IMAGE_WIDTH + x1].z << " " << p1.z << " " << p2.z << std::endl;
        }
      }
      x1++;
      y1 += a;
    }
  }

  std::cout << "count : " << count << " check : " << check << std::endl;
  if (!count)
  {
    std::cout << "count is " << count << std::endl;
    return 0;
  }

  else if ((100 * check / count) >= 95)
  {
    return 1;
  }

  return 0;
}

std::vector< Endpoints > group_endpoints(std::vector< Endpoints > endpoints_vec,
                                         pcl::PointCloud< pcl::PointXYZRGB >& cloud)
{
  std::vector< Endpoints > tmp, fin_e;
  std::vector< int > group_number;

  return endpoints_vec;

  for (int i = 0; i < endpoints_vec.size(); i++)
  {
    group_number.push_back(i);
    tmp.push_back(endpoints_vec[i]);
  }

  std::cout << "group number : " << group_number.size() << std::endl;

  for (int i = 0; i < endpoints_vec.size(); i++)
  {
    for (int j = 0; j < endpoints_vec.size(); j++)
    {
      if (i >= j)
        continue;
      double dis1, dis2;
      dis1 = sqrt((tmp[group_number[i]].p1.x - tmp[group_number[j]].p2.x) *
                      (tmp[group_number[i]].p1.x - tmp[group_number[j]].p2.x) +
                  (tmp[group_number[i]].p1.z - tmp[group_number[j]].p2.z) *
                      (tmp[group_number[i]].p1.z - tmp[group_number[j]].p2.z));

      dis2 = sqrt((tmp[group_number[i]].p2.x - tmp[group_number[j]].p1.x) *
                      (tmp[group_number[i]].p2.x - tmp[group_number[j]].p1.x) +
                  (tmp[group_number[i]].p2.z - tmp[group_number[j]].p1.z) *
                      (tmp[group_number[i]].p2.z - tmp[group_number[j]].p1.z));

      std::cout << "dis1 : " << dis1 << " dis2 : " << dis2 << std::endl;
      if ((dis1 < 0.5) || (dis2 < 0.5))
      {
        if ((dis1 <= dis2))
        {
          if (check_occlusion(cloud, tmp[group_number[j]].p2, tmp[group_number[i]].p1, tmp[group_number[j]].inlier2,
                              tmp[group_number[i]].inlier1))
          {
            tmp[group_number[i]].cloud += tmp[group_number[j]].cloud;
            tmp[group_number[i]].inlier1 = tmp[group_number[j]].inlier1;
            tmp[group_number[i]].p1 = tmp[group_number[j]].p1;
            group_number[j] = group_number[i];
          }
        }

        else if ((dis1 > dis2))
        {
          if (check_occlusion(cloud, tmp[group_number[i]].p2, tmp[group_number[j]].p1, tmp[group_number[i]].inlier2,
                              tmp[group_number[j]].inlier1))
          {
            tmp[group_number[i]].cloud += tmp[group_number[j]].cloud;
            tmp[group_number[i]].inlier2 = tmp[group_number[j]].inlier2;
            tmp[group_number[i]].p2 = tmp[group_number[j]].p2;
            group_number[j] = group_number[i];
          }
        }
      }
    }
  }

  std::cout << "group : ";
  for (int j = 0; j < group_number.size(); j++)
  {
    std::cout << group_number[j] << " ";
  }
  std::cout << std::endl;

  for (int i = 0; i < tmp.size(); i++)
  {
    for (int j = 0; j < group_number.size(); j++)
    {
      if (group_number[j] == i)
      {
        fin_e.push_back(tmp[i]);
        break;
      }
    }
  }

  return fin_e;
}

//クラスタリング
Endpoints clustering(pcl::PointCloud< pcl::PointXYZRGB >& f_cloud, pcl::PointCloud< pcl::PointXYZRGB >& cloud,
                     std::vector< int >& index)
{
  std::cout << "clustering" << std::endl;

  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud2(new pcl::PointCloud< pcl::PointXYZRGB >);
  std::vector< Endpoints > endpoints_vec;
  std::vector< Endpoints > fin_endpoints_vec;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree< pcl::PointXYZRGB >::Ptr tree(new pcl::search::KdTree< pcl::PointXYZRGB >);
  tree->setInputCloud(f_cloud.makeShared());

  std::vector< pcl::PointIndices > cluster_indices;
  pcl::EuclideanClusterExtraction< pcl::PointXYZRGB > ec;
  ec.setClusterTolerance(0.03);
  ec.setMinClusterSize(3000);
  ec.setMaxClusterSize(100000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(f_cloud.makeShared());
  ec.extract(cluster_indices);

  std::cout << "クラスタリング開始" << std::endl;
  int n = 0;
  float colors[6][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
  for (std::vector< pcl::PointIndices >::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_cluster(new pcl::PointCloud< pcl::PointXYZRGB >);
    pcl::PointIndices::Ptr inlier(new pcl::PointIndices);

    for (std::vector< int >::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      cloud_cluster->points.push_back(f_cloud.points[*pit]);
      inlier->indices.push_back(index[*pit]);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    std::cout << "クラスタリングaaa" << std::endl;

    for (int i = 0; i < cloud_cluster->points.size(); i++)
    {
      cloud_cluster->points[i].r = colors[n % 6][0];
      cloud_cluster->points[i].g = colors[n % 6][1];
      cloud_cluster->points[i].b = colors[n % 6][2];
    }
    *tmp_cloud2 += *cloud_cluster;

    std::cout << "part1" << std::endl;
    endpoints_vec.push_back(endpoint(*cloud_cluster, *inlier));
    n++;
  }

  std::cout << "part2" << std::endl;

  fin_endpoints_vec = group_endpoints(endpoints_vec, cloud);

  std::cout << "part3" << std::endl;

  double len = 0.0;
  int id;
  for (int i = 0; i < fin_endpoints_vec.size(); i++)
  {
    for (int j = 0; j < fin_endpoints_vec[i].cloud.points.size(); j++)
    {
      fin_endpoints_vec[i].cloud.points[j].r = colors[i % 6][0];
      fin_endpoints_vec[i].cloud.points[j].g = colors[i % 6][1];
      fin_endpoints_vec[i].cloud.points[j].b = colors[i % 6][2];
    }
    *tmp_cloud += fin_endpoints_vec[i].cloud;

    double l = sqrt((fin_endpoints_vec[i].p1.x - fin_endpoints_vec[i].p2.x) *
                        (fin_endpoints_vec[i].p1.x - fin_endpoints_vec[i].p2.x) +
                    (fin_endpoints_vec[i].p1.y - fin_endpoints_vec[i].p2.y) *
                        (fin_endpoints_vec[i].p1.y - fin_endpoints_vec[i].p2.y) +
                    (fin_endpoints_vec[i].p1.z - fin_endpoints_vec[i].p2.z) *
                        (fin_endpoints_vec[i].p1.z - fin_endpoints_vec[i].p2.z));
    if (l > len)
    {
      len = l;
      id = i;
    }

    // std::cout << "\ngroup " << i << std::endl;
    // std::cout << "p1 " << fin_endpoints_vec[i].p1.x << " " << fin_endpoints_vec[i].p1.y << " " <<
    // fin_endpoints_vec[i].p1.z << std::endl;
    // std::cout << "p2 " << fin_endpoints_vec[i].p2.x << " " << fin_endpoints_vec[i].p2.y << " " <<
    // fin_endpoints_vec[i].p2.z << std::endl;
  }

  pcl::io::savePCDFile("src/ods_person_detection/data/person_detection/group_cloud.pcd", *tmp_cloud);
  pcl::io::savePCDFile("src/ods_person_detection/data/person_detection/cluster_cloud.pcd", *tmp_cloud2);

  return fin_endpoints_vec[id];
}

bool person_detection(tms_msg_ss::ods_person_detection::Request& req, tms_msg_ss::ods_person_detection::Response& res)
{
  std::cout << "person_detection" << std::endl;

  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  Endpoints person;
  cv_bridge::CvImagePtr cv_ptr;

  tms_msg_ss::ods_pcd srv, srv2;
  srv.request.id = 3;
  if (commander_to_kinect_capture.call(srv))
  {
    pcl::fromROSMsg(srv.response.cloud, *cloud);
  }

  srv2.request.id = 4;
  if (commander_to_kinect_capture.call(srv2))
  {
    cv_ptr = cv_bridge::toCvCopy(srv2.response.image, sensor_msgs::image_encodings::BGR8);
    cv::imwrite("src/ods_person_detection/data/person_detection/rgb_image.png", cv_ptr->image);
    cv_ptr->toImageMsg(res.image);
    std::cout << srv2.response.image.encoding << std::endl;
    // cv::imwrite("src/ods_person_detection/data/person_detection/rgb_image.png", rgb_ptr->image);
  }

  // pcl::io::loadPCDFile("src/ods_person_detection/data/person_detection/input.pcd", *cloud);
  pcl::io::savePCDFile("src/ods_person_detection/data/person_detection/input.pcd", *cloud);

  pcl::PointCloud< pcl::PointXYZRGB >::Ptr f_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  std::vector< int > index;

  passfilter(*cloud, *f_cloud, index);

  person = clustering(*f_cloud, *cloud, index);

  /*while(!viewer.wasStopped()){
      viewer.showCloud(cloud);
  }*/

  res.p1_x = person.p1.x * 1000;
  res.p1_y = person.p1.y * 1000;
  res.p1_z = person.p1.z * 1000;
  res.p2_x = person.p2.x * 1000;
  res.p2_y = person.p2.y * 1000;
  res.p2_z = person.p2.z * 1000;

  std::cout << "p1 : " << res.p1_x << " " << res.p1_y << " " << res.p1_z << std::endl;
  std::cout << "p2 : " << res.p2_x << " " << res.p2_y << " " << res.p2_z << std::endl;

  // pcl::io::savePCDFile("src/ods_person_detection/data/person_detection/filter.pcd", *cloud);

  return true;
}

int main(int argc, char** argv)
{
  printf("init\n");
  ros::init(argc, argv, "ods_person_detection");
  ros::NodeHandle n;

  service = n.advertiseService("ods_person_detection", person_detection);

  commander_to_kinect_capture = n.serviceClient< tms_msg_ss::ods_pcd >("ods_capture");

  ros::spin();

  return 0;
}
