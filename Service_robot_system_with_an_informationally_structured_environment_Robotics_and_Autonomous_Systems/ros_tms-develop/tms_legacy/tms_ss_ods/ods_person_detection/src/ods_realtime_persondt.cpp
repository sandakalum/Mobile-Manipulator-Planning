#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <sensor_msgs/PointCloud2.h>
#include <tms_msg_ss/ods_person_dt.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#define PI 3.1415926
#define IMAGE_WIDTH 640

ros::ServiceServer service;
ros::ServiceClient commander_to_kinect_capture;

pcl::PointCloud< pcl::PointXYZRGB >::Ptr viewer_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);

typedef struct
{
  pcl::PointCloud< pcl::PointXYZRGB > cloud;
  int inlier1;
  int inlier2;
  pcl::PointXYZRGB p1;
  pcl::PointXYZRGB p2;
} Endpoints;

Endpoints person;

void passfilter(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZRGB >& cloud_out,
                std::vector< int >& index)
{
  // std::cout << "passfilter" << std::endl;
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  std::vector< int > tmp_index;

  pcl::removeNaNFromPointCloud(cloud, *tmp_cloud, tmp_index);

  for (int i = 0; i < tmp_cloud->size(); i++)
  {
    if ((-0.20 < tmp_cloud->points[i].y) && (tmp_cloud->points[i].y < 0.20))
    {
      cloud_out.points.push_back(tmp_cloud->points[i]);
      index.push_back(tmp_index[i]);
    }
  }
  // Create the filtering object
  /*pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud.makeShared());
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.2, 0.2);
  pass.filter (cloud);*/

  return;
}

Endpoints endpoint(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointIndices& inlier)
{
  Endpoints e;
  pcl::PointXYZRGB p1, p2;
  p1.x = 100.0, p2.x = -100.0;
  int inlier1 = 0, inlier2 = 0;

  e.cloud = cloud;

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

int check_occlusion(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointXYZRGB p1, pcl::PointXYZRGB p2, int inlier1,
                    int inlier2)
{
  double a, b;
  int x1, y1, x2, y2;
  int count = 0, check = 0;

  if (p1.x == p2.x)
  {
    if (p1.y == p2.y)
    {
      return 1;
    }
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
          // std::cout << (double)cloud.points[y1*IMAGE_WIDTH+x1].z << " " << p1.z << " " << p2.z << std::endl;
        }
      }
      x1++;
      y1 += a;
    }
  }

  // std::cout << "count : " << count << " check : " << check << std::endl;
  if (!count)
  {
    // std::cout << "count is " << count << std::endl;
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

  for (int i = 0; i < endpoints_vec.size(); i++)
  {
    group_number.push_back(i);
    tmp.push_back(endpoints_vec[i]);
  }

  std::cout << "group number : " << group_number.size() << std::endl;
  // int m = group_number.size();
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

      // std::cout << "dis1 : " << dis1 << " dis2 : " << dis2 << std::endl;
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
  // std::cout << "clustering" << std::endl;

  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud2(new pcl::PointCloud< pcl::PointXYZRGB >);
  std::vector< Endpoints > endpoints_vec;
  std::vector< Endpoints > fin_endpoints_vec;
  Endpoints final;

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

  // std::cout << "クラスタリング開始" << std::endl;
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

    for (int i = 0; i < cloud_cluster->points.size(); i++)
    {
      cloud_cluster->points[i].r = colors[n % 6][0];
      cloud_cluster->points[i].g = colors[n % 6][1];
      cloud_cluster->points[i].b = colors[n % 6][2];
    }
    *tmp_cloud2 += *cloud_cluster;

    // std::cout << "part1" << std::endl;
    endpoints_vec.push_back(endpoint(*cloud_cluster, *inlier));
    n++;
  }

  // std::cout << "part2" << std::endl;

  fin_endpoints_vec = group_endpoints(endpoints_vec, cloud);

  // std::cout << "part3" << std::endl;

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

  char home_dir[255];
  strcpy(home_dir, getenv("HOME"));
  std::string path = home_dir;
  std::string file_name1 = path + "/catkin_ws/src/ros_tms/tms_ss/tms_ss_ods/ods_person_detection/data/group_cloud.pcd";
  std::string file_name2 = path + "/catkin_ws/src/ros_tms/tms_ss/tms_ss_ods/ods_person_detection/data/"
                                  "cluster_cloud.pcd";

  pcl::io::savePCDFile(file_name1, *tmp_cloud);
  pcl::io::savePCDFile(file_name2, *tmp_cloud2);

  cloud = *tmp_cloud;

  final = fin_endpoints_vec[id];

  std::vector< Endpoints >().swap(fin_endpoints_vec);
  std::vector< Endpoints >().swap(endpoints_vec);

  return final;
}

void person_detection(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZRGB >& out_cloud)
{
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr f_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  std::vector< int > index;
  passfilter(cloud, *f_cloud, index);

  person = clustering(*f_cloud, cloud, index);

  pcl::copyPointCloud(cloud, out_cloud);

  return;
}

void callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr out_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);

  pcl::fromROSMsg(*input, *cloud);
  pcl::copyPointCloud(*cloud, *tmp_cloud);

  person_detection(*cloud, *out_cloud);

  *viewer_cloud = *tmp_cloud + *out_cloud;

  return;
}

int main(int argc, char** argv)
{
  // INITIALIZE ROS
  ros::init(argc, argv, "realtime_persondt");
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  ros::Publisher chatter_pub = nh.advertise< tms_msg_ss::ods_person_dt >("ods_realtime_persondt", 1);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    tms_msg_ss::ods_person_dt msg;

    pcl_sub = nh.subscribe("/camera/depth_registered/points", 1, callback);

    while (1)
    {
      if (viewer_cloud->points.size() != 0)
      {
        msg.p1_x = person.p1.x;
        msg.p1_y = person.p1.y;
        msg.p1_z = person.p1.z;
        msg.p2_x = person.p2.x;
        msg.p2_y = person.p2.y;
        msg.p2_z = person.p2.z;
        viewer.showCloud(viewer_cloud);

        ROS_INFO("p1(%f, %f, %f)\np2(%f, %f, %f)\n\n", msg.p1_x, msg.p1_y, msg.p1_z, msg.p2_x, msg.p2_y, msg.p2_z);

        chatter_pub.publish(msg);
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  return 0;
}
