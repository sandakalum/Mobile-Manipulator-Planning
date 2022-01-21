#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

// PCL specific includes
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZ PointType1;

ros::Subscriber pcl_sub;

#define LOOP 20
int n = 0;
int id;

void callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  std::cout << "Callback " << n << std::endl;
  if (n < LOOP)
  {
    pcl::PointCloud< PointType1 >::Ptr cloud(new pcl::PointCloud< PointType1 >);

    pcl::fromROSMsg(*input, *cloud);

    std::stringstream ss;
    ss << "src/ods_make_model/data/capture/input" << id << "_" << n << ".pcd";
    pcl::io::savePCDFile(ss.str(), *cloud, false);

    n++;
  }

  return;
}

int main(int argc, char** argv)
{
  printf("capture_data\nid = ");
  scanf("%d", &id);
  ros::init(argc, argv, "ods_makemodel_capture");
  ros::NodeHandle nh;
  ros::Rate loop_late(1);
  ros::Time::init();

  pcl_sub = nh.subscribe("/camera/depth_registered/points", 1, callback);

  while (n < LOOP)
  {
    ros::spinOnce();
    loop_late.sleep();
  }

  return 0;
}
