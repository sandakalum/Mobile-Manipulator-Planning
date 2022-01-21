//---------------------------------------------------------------
// @file    : portable_tf_first.cpp 
// @author  : Watanabe Yuuta
// @version : Ver0.1.0 (since 2015.08.06)
// @date    : 2015
//---------------------------------------------------------------
#include "ros/ros.h"
#include "math.h"
#include "iostream"
#include "tf/transform_broadcaster.h"
#include "tms_msg_rc/odom_rad.h"
#include "nav_msgs/Odometry.h"

//initialize
ros::Time current_time, last_time;
std::string odom_param;
std::string base_footprint_param;

//odom
ros::Publisher odom_pub;

//callback function
void plot_callback(const tms_msg_rc::odom_rad& plot)
{
  //param
  //ros::param::get("~odom_param", odom_param);
  //ros::param::get("~base_footprint_param", base_footprint_param);

  //time
  current_time = ros::Time::now();

  //tf broadcaster
  static tf::TransformBroadcaster broadcaster;

  //robot odom -> base_footprintf ------------------------------------------------------------------------------
  geometry_msgs::Quaternion robot_quat;
  robot_quat = tf::createQuaternionMsgFromYaw(plot.position_theta);
  geometry_msgs::TransformStamped state;
  state.header.stamp    = ros::Time::now();
  state.header.frame_id = odom_param;
  //state.header.frame_id = "odom7";
  state.child_frame_id  = base_footprint_param;
  //state.child_frame_id  = "base_footprint7";
  state.transform.translation.x = plot.position_x;
  state.transform.translation.y = plot.position_y;
  state.transform.translation.z = plot.position_z;
  state.transform.rotation      = robot_quat;
  broadcaster.sendTransform(state);

  //odometory message
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = odom_param;
  //odom.header.frame_id = "odom7";

  //set the position
  odom.pose.pose.position.x = plot.position_x;
  odom.pose.pose.position.y = plot.position_y;
  odom.pose.pose.position.z = plot.position_z;
  odom.pose.pose.orientation = robot_quat;
  odom.child_frame_id = base_footprint_param;
  odom.twist.twist.linear.x  = plot.velocity_x;
  odom.twist.twist.linear.y  = plot.velocity_y;
  odom.twist.twist.angular.z = plot.velocity_theta;

  odom_pub.publish(odom);

  //update
  last_time  = current_time;
}

//main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "portable_tf_first");

  ros::NodeHandle nh;

  //nh.setParam("odom_param", "odom");
  //nh.setParam("base_footprint_param", "base_footprint");

  //param
  ros::param::get("~odom_param", odom_param);
  ros::param::get("~base_footprint_param", base_footprint_param);

  ros::Subscriber sub = nh.subscribe("odom_rad", 100, &plot_callback);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);

  current_time = last_time = ros::Time::now();

  ros::spin();
  return 0;
}
