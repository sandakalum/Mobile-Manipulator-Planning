//----------------------------------------------------------
// @file   : portable_virtual_laser.cpp
// @author : Watanabe Yuuta 
// @version: Ver0.0.1 (since 2014.05.02)
// @date   : 2014.11.21
//----------------------------------------------------------
#include "ros/ros.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher  pub;

int main( int argc, char **argv )
{
    ros::init(argc, argv, "pot_urg_scan1");
    ros::NodeHandle nh;
    pub   = nh.advertise<sensor_msgs::LaserScan>("/scan", 1000);
  
    ros::Rate loop(10);
    while(ros::ok())
    {
      sensor_msgs::LaserScan Laser;
      Laser.header.stamp    = ros::Time::now();
      Laser.header.frame_id = "base_scan";
      Laser.angle_min       = -1.83464097977;
      Laser.angle_max       = 1.83464097977;
      Laser.angle_increment = 0.00613592332229;
      Laser.time_increment  = 9.76562732831e-05;
      Laser.scan_time       = 0.10000000149;
      Laser.range_min       = 0.019999999553;
      Laser.range_max       = 5.59999990463;
      if (Laser.ranges.size() == 0 ) Laser.ranges.resize(726);
      for (int i = 0; i < 720; i++)
      {
        Laser.ranges.push_back(5.2);

        if(240 < i &&  480 < i)
        {
          Laser.ranges.push_back(3.0);
        }
      }

      pub.publish(Laser);
      Laser.ranges.clear();
      loop.sleep();
    }

    return 0;
}

