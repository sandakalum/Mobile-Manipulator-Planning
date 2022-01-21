#include <ods_capture.h>

pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
std::vector< float > check;

/*void downsampling(float th)
{
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (th, th, th);
    sor.filter (*cloud);

    return;
}*/

// depth or depth_registered points
void callback(const sensor_msgs::PointCloud2::ConstPtr &input)
{
  if (n < 5)
  {
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp(new pcl::PointCloud< pcl::PointXYZRGB >);
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp2(new pcl::PointCloud< pcl::PointXYZRGB >);
    std::vector< int > index;
    pcl::fromROSMsg(*input, *tmp);
    n++;

    if (n == 1)
    {
      *cloud += *tmp;
    }

    pcl::removeNaNFromPointCloud(*tmp, *tmp2, index);

    int m = 0;
    for (int i = 0; i < tmp->points.size(); i++)
    {
      if ((index[m] == i) && (check[i] != 1))
      {
        cloud->points[i].x += tmp->points[i].x;
        cloud->points[i].y += tmp->points[i].y;
        cloud->points[i].z += tmp->points[i].z;
        m++;
        check[i] += 1;
      }
    }

    std::stringstream ss;
    ss << "src/ods_capture/data/smoothing_capture/input_" << n << ".pcd";
    pcl::io::savePCDFile(ss.str(), *tmp);
  }

  return;
}

bool capture_pcd(tms_msg_ss::ods_pcd::Request &req, tms_msg_ss::ods_pcd::Response &res)
{
  std::cout << "capture_cloud" << std::endl;
  ros::Rate loop_late(0.2);
  ros::Time::init();
  ros::NodeHandle nh;

  n = 0;

  check.resize(307200);

  for (int i = 0; i < check.size(); i++)
  {
    check[i] = 0.0;
  }

  //******************************
  // id == 1 なら　3次元点群のみ取得
  //******************************
  if (req.id == 1)
  {
    std::cout << "Depth points" << std::endl;
    pcl_sub = nh.subscribe("/camera/depth_registered/points", 1, callback);

    while (n < 5)
    {
      ros::spinOnce();
      loop_late.sleep();
    }

    int k = 0;
    for (int i = 0; i < check.size(); i++)
    {
      if (check[i] != 0)
      {
        k++;
        cloud->points[i].x /= check[i];
        cloud->points[i].y /= check[i];
        cloud->points[i].z /= check[i];
      }
      else
      {
        cloud->points[i].x = NAN;
        cloud->points[i].y = NAN;
        cloud->points[i].z = NAN;
      }
    }

    std::cout << k << std::endl;
    pcl::io::savePCDFile("src/ods_capture/data/smoothing_capture/input.pcd", *cloud);
    pcl::toROSMsg(*cloud, *client);
    res.cloud = *client;
  }

  return true;
}

int main(int argc, char **argv)
{
  // INITIALIZE ROS
  ros::init(argc, argv, "kinect_capture");
  ros::NodeHandle n;

  service = n.advertiseService("capture_cloud", capture_pcd);

  ros::spin();

  return 0;
}
