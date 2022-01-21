#include <ods_capture.h>

void callback(const sensor_msgs::ImageConstPtr& msg)
{
  std::cout << "callback" << n << std::endl;

  cv_bridge::CvImagePtr cv_ptr;
  if (n <= 2)
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::imwrite("src/ods_capture/data/DepthImage/DepthImage.png", cv_ptr->image);
    n++;
  }

  return;
}

int main(int argc, char** argv)
{
  std::cout << "capture depth_image" << std::endl;
  ros::init(argc, argv, "ods_capture_DepthImage");
  ros::NodeHandle nh;
  ros::Rate loop_late(1);
  ros::Time::init();

  if (n <= 2)
  {
    ir_sub = nh.subscribe("/camera/depth/image_raw", 1, callback);

    while (n <= 2)
    {
      ros::spinOnce();
      loop_late.sleep();
    }
  }

  return true;
}
