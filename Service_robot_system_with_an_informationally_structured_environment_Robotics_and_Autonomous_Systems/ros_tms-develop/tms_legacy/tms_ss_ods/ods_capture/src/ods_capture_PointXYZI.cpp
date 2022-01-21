#include <ods_capture.h>

void callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  std::cout << "callback1 " << n << std::endl;
  if (n <= 3)
  {
    pcl::PointCloud< PointType1 >::Ptr cloud(new pcl::PointCloud< PointType1 >);
    pcl::fromROSMsg(*input, *cloud);

    pcl::io::savePCDFile("src/ods_capture/data/PointXYZI/input.pcd", *cloud, false);

    n++;
  }

  return;
}

void callback2(const sensor_msgs::Image::ConstPtr& input)
{
  std::cout << "callback2 " << n << std::endl;

  n++;
  pcl::PointCloud< PointType1 >::Ptr cloud_src(new pcl::PointCloud< PointType1 >);
  pcl::PointCloud< PointType2 >::Ptr cloud_cbr(new pcl::PointCloud< PointType2 >);
  pcl::io::loadPCDFile("src/ods_capture/data/PointXYZI/input.pcd", *cloud_src);
  pcl::copyPointCloud(*cloud_src, *cloud_cbr);

  for (int i = 0; i < input->data.size(); i += 2)
  {
    cloud_cbr->points[i / 2].intensity = input->data[i];
  }

  for (int i = 0; i < cloud_cbr->points.size(); i++)
  {
    if (cloud_cbr->points[i].intensity >= 230)
    {
      cloud_cbr->points[i].intensity = 255;
    }
    else
    {
      cloud_cbr->points[i].intensity = 0;
    }
  }

  pcl::io::savePCDFile("src/ods_capture/data/PointXYZI.pcd", *cloud_cbr, true);

  return;
}

void depth()
{
  ros::NodeHandle nh;
  ros::Rate loop_late(1);
  ros::Time::init();

  if (n <= 1)
  {
    pcl_sub = nh.subscribe("/camera/depth/points", 1, callback);

    while (n <= 1)
    {
      ros::spinOnce();
      loop_late.sleep();
    }
  }

  return;
}

void intensity()
{
  ros::NodeHandle nh;
  ros::Rate loop_late(1);
  ros::Time::init();

  if (n <= 3)
  {
    ir_sub = nh.subscribe("/camera/ir/image_raw", 1, callback2);

    while (n <= 3)
    {
      ros::spinOnce();
      loop_late.sleep();
    }
  }

  return;
}

int main(int argc, char** argv)
{
  printf("capture_PointXYZI\n");
  ros::init(argc, argv, "ods_capture_PointXYZI");

  depth();
  intensity();

  // calibration

  return true;
}
