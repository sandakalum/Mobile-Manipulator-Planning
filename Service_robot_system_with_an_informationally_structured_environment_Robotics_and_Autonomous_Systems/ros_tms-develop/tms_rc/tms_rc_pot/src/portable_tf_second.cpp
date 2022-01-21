//---------------------------------------------------------------
// @file    : portable_tf_second.cpp 
// @author  : Watanabe Yuuta
// @version : Ver0.1.0 (since 2015.08.06)
// @date    : 2015
//---------------------------------------------------------------

#include "math.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

//main function
int main(int argc, char** argv)
{
  ros::init(argc, argv, "portable_tf_second");
  ros::NodeHandle n;

  std::string base_footprint;
  std::string base_link;
  std::string base_scan;

  //n.setParam("base_footprint_param", "base_footprint");
  //n.setParam("base_link_param", "base_link");
  //n.setParam("base_scan_param", "base_scan");

  ros::param::get("~base_footprint_param", base_footprint);
  ros::param::get("~base_link_param", base_link);
  ros::param::get("~base_scan_param", base_scan);

  ros::Rate r(50);

  tf::TransformBroadcaster broadcaster;

  while(n.ok())
  {
    //ros::param::get("~base_footprint_param", base_footprint);
    //ros::param::get("~base_link_param", base_link);
    //ros::param::get("~base_scan_param", base_scan);

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(
          tf::Quaternion(0, 0, 0, 1),
          tf::Vector3(0.0, 0.0, 0.01)),
          ros::Time::now(),
          base_footprint,
          base_link));

    tf::Quaternion q;
    //q.setRPY(M_PI, 0, 0);
    q.setRPY(0, 0, 0);

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(
          q,
          tf::Vector3(0.03, 0.0, 0.16)),
          ros::Time::now(),
          base_link,
          base_scan));
    r.sleep();
  }
}
