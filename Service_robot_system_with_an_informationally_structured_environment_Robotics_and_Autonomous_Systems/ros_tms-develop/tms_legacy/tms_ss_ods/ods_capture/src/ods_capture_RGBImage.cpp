#include <ods_capture.h>

void callback(const sensor_msgs::ImageConstPtr& msg)
{
  std::cout << "callback" << n << std::endl;

  cv_bridge::CvImagePtr cv_ptr;
  if (n <= 1)
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::imwrite("src/ods_capture/data/RGBImage/RGBImage.png", cv_ptr->image);
    n++;
  }

  return;
}

int main(int argc, char** argv)
{
  std::cout << "capture RGBImage" << std::endl;
  ros::init(argc, argv, "ods_capture_RGBImage");
  ros::NodeHandle nh;
  ros::Rate loop_late(1);
  ros::Time::init();

  if (n <= 1)
  {
    ir_sub = nh.subscribe("/camera/rgb/image_raw", 1, callback);

    while (n <= 1)
    {
      ros::spinOnce();
      loop_late.sleep();
    }
  }

  return true;
}
