#include <ods_capture.h>

void callback(const sensor_msgs::ImageConstPtr& msg)
{
  std::cout << "callback" << n << std::endl;

  if (n <= 2)
  {
    cv::Mat ir_ptr(msg->width, msg->height, CV_8UC1);
    ir_ptr.rows = msg->height;
    ir_ptr.cols = msg->width;
    ir_ptr.step = msg->width;
    int elemsize = ir_ptr.step / ir_ptr.cols;
    int elemsize2 = msg->step / msg->width;

    for (int y = 0; y < ir_ptr.rows; y++)
    {
      for (int x = 0; x < ir_ptr.cols; x++)
      {
        ir_ptr.data[y * ir_ptr.step + x * elemsize] = msg->data[y * msg->step + x * elemsize2];
      }
    }
    cv::imwrite("src/ods_capture/data/IRImage/IRImage.png", ir_ptr);
    n++;
  }

  return;
}

int main(int argc, char** argv)
{
  std::cout << "capture ir_image" << std::endl;
  ros::init(argc, argv, "ods_capture_IRImage");
  ros::NodeHandle nh;
  ros::Rate loop_late(1);
  ros::Time::init();

  if (n <= 2)
  {
    ir_sub = nh.subscribe("/camera/ir/image_raw", 1, callback);

    while (n <= 2)
    {
      ros::spinOnce();
      loop_late.sleep();
    }
  }

  return true;
}
