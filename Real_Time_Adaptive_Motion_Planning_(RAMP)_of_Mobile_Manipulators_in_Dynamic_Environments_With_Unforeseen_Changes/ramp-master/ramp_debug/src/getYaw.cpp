#include <iostream>
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

float value;

void odometryCallback(const nav_msgs::Odometry& msg) {
  float orientation_degrees = tf::getYaw(msg.pose.pose.orientation)*180/M_PI;
  std::cout<<"\nYaw: "<<orientation_degrees;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "getYaw");
  ros::NodeHandle handle;

  ros::Subscriber sub = handle.subscribe("odom", 1000, odometryCallback);

  ros::spin();
}
