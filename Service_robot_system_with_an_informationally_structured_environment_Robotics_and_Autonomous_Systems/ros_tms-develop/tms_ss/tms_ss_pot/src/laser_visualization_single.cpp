
#include "ros/ros.h"
#include "stdio.h"
#include "tms_msg_ss/tracking_points.h"
#include "tms_msg_ss/tracking_grid.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "iostream"
#include "math.h"
#include <sensor_msgs/LaserScan.h>
#include "opencv/cv.h"
#include "define.h"
#include <tf/transform_broadcaster.h>

ros::Publisher pub;
ros::Subscriber sub;

#define STEP 5

// float colorset[14][4] = {{1, 0, 0, 0},
//                          {0, 1, 0, 0},
//                          {0, 0, 1, 0},
//                          {1, 1, 0, 0},
//                          {0, 1, 1, 0},
//                          {1, 0, 1, 0},
//                          {1, 1, 1, 0},
//                          {0.5, 0, 0, 0},
//                          {0, 0.5, 0, 0},
//                          {0, 0, 0.5, 0},
//                          {0.5, 0.5, 0, 0},
//                          {0, 0.5, 0.5, 0},
//                          {0.5, 0, 0.5, 0},
//                          {0.5, 0.5, 0.5, 0}};

void laser_visualization_callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  visualization_msgs::MarkerArray markerArray;

  double laser_x = Config::is()->lrf1_pos[0]; 
  double laser_y = Config::is()->lrf1_pos[1]; 
  double laser_theta = Config::is()->lrf1_pos[2];
  double laser_point_x;
  double laser_point_y;

  CvMat *m_Rotate = cvCreateMat(2, 2, CV_64F);
  CvMat *Temp = cvCreateMat(2, 1, CV_64F);
  CvMat *Temp2 = cvCreateMat(2, 1, CV_64F);
  double theta = 0.0;

  cvmSet(m_Rotate, 0, 0,  cos(deg2rad(laser_theta)));
  cvmSet(m_Rotate, 0, 1, -sin(deg2rad(laser_theta)));
  cvmSet(m_Rotate, 1, 0,  sin(deg2rad(laser_theta)));
  cvmSet(m_Rotate, 1, 1,  cos(deg2rad(laser_theta)));

  int id = 0;

  uint32_t shape_arrow = visualization_msgs::Marker::ARROW;

  for (int i = 0; i < 3; i++){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world_link";
    marker.header.stamp = ros::Time::now();

    marker.id = id;
    id++;
    marker.type = shape_arrow;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 2.0;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.pose.position.x = laser_x;
    marker.pose.position.y = laser_y;
    marker.pose.position.z = 0;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    tf::Quaternion q;
    switch(i){
      case 0:
        q.setRPY(0, 0, deg2rad(laser_theta));
        marker.color.r = 1.0f;
        break;
      case 1:
        q.setRPY(0, 0, deg2rad(laser_theta) + M_PI / 2.0);
        marker.color.g = 1.0f;
        break;
      case 2:
        q.setRPY(0, -M_PI / 2.0, deg2rad(laser_theta));
        marker.color.b = 1.0f;
        break;
    }

    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    markerArray.markers.push_back(marker);
  }

  double prev_x, prev_y;

  for (unsigned int i = 0; i < scan->ranges.size(); i+=STEP)
  {
    theta = scan->angle_increment * i + scan->angle_min;
    cvmSet(Temp, 0, 0, scan->ranges[i] * cos(theta));
    cvmSet(Temp, 1, 0, scan->ranges[i] * sin(theta));
    cvMatMul(m_Rotate, Temp, Temp2);
    laser_point_x = cvmGet(Temp2, 0, 0);
    laser_point_y = cvmGet(Temp2, 1, 0);
    laser_point_x = laser_point_x + laser_x;
    laser_point_y = laser_point_y + laser_y;

    if( i == 0 ){ 
       prev_x = laser_point_x;
       prev_y = laser_point_y;
    }

    visualization_msgs::Marker marker;
    uint32_t laser_line = visualization_msgs::Marker::LINE_STRIP;

    marker.header.frame_id = "/world_link";
    marker.header.stamp = ros::Time::now();
    marker.id = id;
    id++;
    marker.type = laser_line;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.color.a = 0.5;
    geometry_msgs::Point ps;
/*
    ps.x = laser_x;
    ps.y = laser_y;
*/
    ps.x = prev_x;
    ps.y = prev_y;
    ps.z = 1.0;
    marker.points.push_back(ps);

    ps.x = laser_point_x;
    ps.y = laser_point_y;
    ps.z = 1.0;
    marker.points.push_back(ps);

    marker.lifetime = ros::Duration(1.1);
    markerArray.markers.push_back(marker);

    prev_x = laser_point_x;
    prev_y = laser_point_y;

  }

  pub.publish(markerArray);
}

// main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_visualization");
  ros::NodeHandle n;
  pub = n.advertise< visualization_msgs::MarkerArray >("laser_visualization", 1);
  sub = n.subscribe("LaserTracker", 1, laser_visualization_callback);
  ros::spin();
  return 0;
}
