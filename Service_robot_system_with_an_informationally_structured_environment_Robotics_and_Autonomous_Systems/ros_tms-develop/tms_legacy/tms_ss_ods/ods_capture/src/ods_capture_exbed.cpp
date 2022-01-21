#include <ods_capture.h>

int main(int argc, char **argv)
{
  std::cout << "init" << std::endl;
  ros::init(argc, argv, "ods_capture_exbed");
  ros::NodeHandle n;

  commander_to_kinect_capture = n.serviceClient< tms_msg_ss::ods_pcd >("ods_capture");

  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  int id;
  std::stringstream ss, ss2;

  std::cout << "id : ";
  scanf("%d", &id);

  tms_msg_ss::ods_pcd srv, srv2;
  srv.request.id = 3;
  if (commander_to_kinect_capture.call(srv))
  {
    pcl::fromROSMsg(srv.response.cloud, *cloud);
  }

  ss << "src/ods_capture/data/exbed/" << id << "/bed" << id << "_1.pcd";
  pcl::io::savePCDFile(ss.str(), *cloud);

  std::cout << "capture memory data" << std::endl;
  std::cout << "id : ";
  scanf("%d", &id);

  srv2.request.id = 3;
  if (commander_to_kinect_capture.call(srv2))
  {
    pcl::fromROSMsg(srv2.response.cloud, *cloud);
  }

  ss2 << "src/ods_capture/data/exbed/" << id << "/bed" << id << "_2.pcd";
  pcl::io::savePCDFile(ss2.str(), *cloud);

  std::cout << "capture curent data" << std::endl;

  return 0;
}
