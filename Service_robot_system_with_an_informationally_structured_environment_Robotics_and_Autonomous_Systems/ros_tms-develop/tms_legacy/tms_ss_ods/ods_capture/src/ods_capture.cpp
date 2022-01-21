#include <ods_capture.h>

// depth or depth_registered points
void callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  if (n < 1)
  {
    *client = *input;

    n++;
  }

  return;
}

// depth points including intensity
void callback2(const sensor_msgs::Image::ConstPtr& input)
{
  if (n < 1)
  {
    pcl::PointCloud< PointType2 >::Ptr cloud_cbr(new pcl::PointCloud< PointType2 >);
    pcl::PCLPointCloud2 tmp_cloud;
    pcl_conversions::toPCL(*client, tmp_cloud);
    pcl::fromPCLPointCloud2(tmp_cloud, *cloud_cbr);

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

    pcl::toPCLPointCloud2(*cloud_cbr, tmp_cloud);
    pcl_conversions::fromPCL(tmp_cloud, *client);
  }

  return;
}

// rgb or depth or depth registered image
void callback3(const sensor_msgs::Image::ConstPtr& input)
{
  if (n < 2)
  {
    *image = *input;
    std::cout << image->encoding << std::endl;
    n++;
  }

  return;
}

bool capture_pcd(tms_msg_ss::ods_pcd::Request& req, tms_msg_ss::ods_pcd::Response& res)
{
  std::cout << "capture_cloud" << std::endl;
  ros::Rate loop_late(0.2);
  ros::Time::init();
  ros::NodeHandle nh;

  n = 0;

  //******************************
  // id == 1 なら　3次元点群のみ取得
  //******************************
  if (req.id == 1)
  {
    std::cout << "Depth points" << std::endl;
    pcl_sub = nh.subscribe("/camera/depth/points", 1, callback);

    while (n < 1)
    {
      ros::spinOnce();
      loop_late.sleep();
    }

    res.cloud = *client;
  }

  //******************************************
  // id == 2 なら　intensity情報付きの3次元点群取得
  //******************************************
  else if (req.id == 2)
  {
    std::cout << "Depth points including intensity" << std::endl;
    pcl_sub = nh.subscribe("/camera/depth/points", 1, callback);

    while (n < 1)
    {
      ros::spinOnce();
      loop_late.sleep();
    }
    n = 0;

    ir_sub = nh.subscribe("/camera/ir/image_raw", 1, callback2);

    while (n < 1)
    {
      ros::spinOnce();
      loop_late.sleep();
    }

    res.cloud = *client;
  }

  //**************************************
  // id == 3 なら　色情報付きの3次元点群情報取得
  //**************************************
  else if (req.id == 3)
  {
    std::cout << "Depth registered points" << std::endl;
    pcl_sub = nh.subscribe("/camera/depth_registered/points", 1, callback);

    while (n < 1)
    {
      ros::spinOnce();
      loop_late.sleep();
    }

    res.cloud = *client;
  }

  //**************************
  // id == 4 なら　カラー画像取得
  //**************************
  else if (req.id == 4)
  {
    std::cout << "RGB image" << std::endl;

    rgb_sub = nh.subscribe("/camera/rgb/image_raw", 1, callback3);

    while (n < 2)
    {
      ros::spinOnce();
      loop_late.sleep();
    }

    res.image = *image;
  }

  //****************************************************
  // id == 5 なら カラー画像とキャリブレーション済み距離画像を取得
  //****************************************************
  else if (req.id == 5)
  {
    std::cout << "Depth registered image" << std::endl;

    depth_sub = nh.subscribe("/camera/depth_registered/image_rect", 1, callback3);

    while (n < 1)
    {
      ros::spinOnce();
      loop_late.sleep();
    }

    res.image = *image;
  }

  return true;
}

int main(int argc, char** argv)
{
  // INITIALIZE ROS
  ros::init(argc, argv, "ods_capture");
  ros::NodeHandle n;

  service = n.advertiseService("ods_capture", capture_pcd);

  ros::spin();

  return 0;
}
