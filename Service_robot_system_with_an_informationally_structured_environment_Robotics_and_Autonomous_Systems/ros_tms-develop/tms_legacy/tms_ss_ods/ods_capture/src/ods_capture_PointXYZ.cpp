#include <ods_capture.h>

void callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  std::cout << "Callback " << n << std::endl;
  if (n <= 1)
  {
    pcl::PointCloud< PointType1 >::Ptr cloud(new pcl::PointCloud< PointType1 >);

    pcl::fromROSMsg(*input, *cloud);

    pcl::io::savePCDFile("src/capture/data/xyz/xyz.pcd", *cloud, false);

    n++;
  }

  return;
}

int main(int argc, char** argv)
{
  printf("capture_PointXYZ\n");
  ros::init(argc, argv, "ods_capture_PointXYZ");
  ros::NodeHandle nh;
  ros::Rate loop_late(1);
  ros::Time::init();

  pcl_sub = nh.subscribe("/openni2_camera/depth/points", 1, callback);

  while (n <= 1)
  {
    ros::spinOnce();
    loop_late.sleep();
  }

  return 0;
}
