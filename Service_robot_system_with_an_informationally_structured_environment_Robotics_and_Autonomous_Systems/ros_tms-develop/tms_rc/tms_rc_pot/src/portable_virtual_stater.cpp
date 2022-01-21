//----------------------------------------------------------
// @file   : virtual_por_test.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.4 (since 2015.08.05)
// @date   : 2015.08.05
//----------------------------------------------------------
#include "ros/ros.h"
#include "tms_msg_rc/odom_rad.h"
#include "math.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "virtual_portable_robot");
  ros::NodeHandle n;
  
  tms_msg_rc::odom_rad portable_odom;
  
  double x,y,z,yaw;
  double theta;
  
  ros::Publisher pub = n.advertise<tms_msg_rc::odom_rad>("odom_rad", 1000); 

  ros::Rate loop(10);
  while(ros::ok())
  {
    portable_odom.header.stamp = ros::Time::now();
    portable_odom.position_x = 0.0;
    portable_odom.position_y = 0.0;
    portable_odom.position_z = 0.0;
    portable_odom.position_theta = 0.0;

    portable_odom.velocity_x = 0.0;
    portable_odom.velocity_y = 0.0;
    portable_odom.velocity_z = 0.0;
    portable_odom.velocity_theta = 0.0;
    pub.publish(portable_odom);

   loop.sleep();
  }  
  return 0;
}

