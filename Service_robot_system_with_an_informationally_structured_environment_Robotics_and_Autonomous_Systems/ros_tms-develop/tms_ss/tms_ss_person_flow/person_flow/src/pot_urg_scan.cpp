//----------------------------------------------------------
// @file   : pot_scan.cpp
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
#include <pthread.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <tms_ss_pot/define.h>

pthread_mutex_t mutex_laser  = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_target = PTHREAD_MUTEX_INITIALIZER;

ros::Publisher  pub;
ros::Subscriber sub;

void LaserSensingCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    pthread_mutex_lock(&mutex_laser);
    //std::cout << "\ESC[2J" ;
    ROS_INFO("start pot_urg_pot");

    sensor_msgs::LaserScan Laser;
    Laser.header.seq      = scan->header.seq;
    Laser.header.stamp    = scan->header.stamp;
    Laser.header.frame_id = scan->header.frame_id;
    Laser.angle_min       = scan->angle_min;
    Laser.angle_max       = scan->angle_max;
    Laser.angle_increment = scan->angle_increment;
    Laser.time_increment  = scan->time_increment;
    Laser.scan_time       = scan->scan_time;
    Laser.range_min       = scan->range_min;
    Laser.range_max       = scan->range_max;

    if (Laser.ranges.size() == 0 ) Laser.ranges.resize(scan->ranges.size());
    for (int i = 0; i < scan->ranges.size(); i++)
    {
        if (isnan(scan->ranges[scan->ranges.size() - i -1]) == 0)
        {
            Laser.ranges[i] = scan->ranges[scan->ranges.size() - i -1];
        }
        else if (isnan(scan->ranges[scan->ranges.size() - i -1]) != 0)
        {
            Laser.ranges[i] = 5.59999999999;
        }
    }
    Laser.intensities = scan->intensities;
    pub.publish(Laser);
    ROS_INFO("end   pot_urg_pot");

    pthread_mutex_unlock(&mutex_laser);
}

int main( int argc, char **argv )
{
    ros::MultiThreadedSpinner spinner(4);
    ros::init(argc, argv, "pot_urg_scan1");
    ros::NodeHandle nh;
    pub   = nh.advertise<sensor_msgs::LaserScan>("/LaserTracker1", 1000);
    sub   = nh.subscribe("/pot_urg1/scan", 1000, LaserSensingCallback);
    spinner.spin();
    ros::waitForShutdown();

    return 0;
}

