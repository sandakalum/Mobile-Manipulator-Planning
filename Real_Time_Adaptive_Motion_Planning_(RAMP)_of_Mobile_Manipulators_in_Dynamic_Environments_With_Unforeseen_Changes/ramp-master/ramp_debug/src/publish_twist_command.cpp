#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

#define PI 3.14159f

float x, y, theta;

void odometryCallback(const nav_msgs::Odometry& msg) {
//  theta = tf::getYaw(msg.pose.pose.orientation);
//  std::cout<<"\nTheta: "<<theta;
//  x = msg.pose.pose.position.x;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_twist_command");
  ros::NodeHandle handle;

  ros::Publisher pub_twist = handle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber sub_odom = handle.subscribe("odom", 1000, odometryCallback); 

  geometry_msgs::Twist t;
  t.linear.x = 0.38f;
  t.linear.y = 0.f;
  t.linear.z = 0.f;
  t.angular.x = 0.f;
  t.angular.y = 0.f;
  t.angular.z = 0.f;

  std::cout<<"\nPress Enter to publish the twist message\n";
  std::cin.get();

  x = 0.f;
  theta = 0.f;
  ros::Rate r(50);
  ros::Time end = ros::Time::now() + ros::Duration(1);
  while(ros::ok() && ros::Time::now() < end) {
    pub_twist.publish(t);
    r.sleep();
    ros::spinOnce();
  }

  std::cout<<ros::Time::now();

  std::cout<<"\nExiting Normally\n";
  return 0;
}
