
#include "ros/ros.h"
#include "math.h"
#include "iostream"
#include "tf/transform_broadcaster.h"
#include "ninebot/odom_rad.h"
#include "nav_msgs/Odometry.h"

//initialize
ros::Time current_time;
std::string odom_frame;
std::string base_frame;

//odom
ros::Publisher odom_pub;

//callback function
void plot_callback(const ninebot::odom_rad& plot)
{
  //time
  current_time = ros::Time::now();

  //tf broadcaster
  static tf::TransformBroadcaster broadcaster;

  //robot odom -> base_footprintf
  geometry_msgs::Quaternion robot_quat;
  robot_quat = tf::createQuaternionMsgFromYaw(plot.position_theta);
  geometry_msgs::TransformStamped state;
  state.header.stamp    = current_time;
  state.header.frame_id = odom_frame;
  state.child_frame_id  = base_frame;
  state.transform.translation.x = plot.position_x;
  state.transform.translation.y = plot.position_y;
  state.transform.translation.z = plot.position_z;
  state.transform.rotation      = robot_quat;
  broadcaster.sendTransform(state);

  //odometory message
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = odom_frame;

  //set the position
  odom.pose.pose.position.x = plot.position_x;
  odom.pose.pose.position.y = plot.position_y;
  odom.pose.pose.position.z = plot.position_z;
  odom.pose.pose.orientation = robot_quat;
  odom.child_frame_id = base_frame;
  odom.twist.twist.linear.x  = plot.velocity_x;
  odom.twist.twist.linear.y  = plot.velocity_y;
  odom.twist.twist.angular.z = plot.velocity_theta;
  odom_pub.publish(odom);
}

//main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ninebot_tf");
  ros::NodeHandle nh;

  //param
  ros::param::get("~odom_frame_id", odom_frame);
  ros::param::get("~base_frame_id", base_frame);

  ros::Subscriber sub = nh.subscribe("odom_rad", 100, &plot_callback);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);

  current_time = ros::Time::now();

  ros::spin();
  return 0;
}
