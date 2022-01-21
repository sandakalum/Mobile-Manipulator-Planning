#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kobuki_tf");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while (n.ok())
  {
    broadcaster.sendTransform(
        tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.01)), ros::Time::now(),
                             "base_footprint", "base_link"));

    broadcaster.sendTransform(
        tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.24)), ros::Time::now(),
                             "base_link", "base_scan"));

    r.sleep();
  }
}
