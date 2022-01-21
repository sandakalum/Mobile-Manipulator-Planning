//----------------------------------------------------------
// @file   : pot_scan.cpp
// @author : Watanabe Yuuta
// @version: Ver0.0.1 (since 2014.05.02)
// @date   : 2016.11.01
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
#include <tms_ss_pot/define.h>
#include <sensor_msgs/LaserScan.h>
#include <tms_msg_ss/fss_tf_data.h>

pthread_mutex_t mutex_laser  = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_target = PTHREAD_MUTEX_INITIALIZER;

ros::Subscriber rosSub1;
ros::Publisher  rosPub1;
ros::Subscriber rosSub2;
ros::Publisher  rosPub2;

void lrfCallback1(const tms_msg_ss::fss_pre_data::ConstPtr &scan)
{
    ROS_INFO_STREAM("completed receiving of fss_tf_data1 #s");
    double StartTime = ros::Time::now().toSec();

    tms_msg_ss::fss_tf_data fss_tf_data;

    float fAngle  = 0;
    float fLrf_set_x = 1500;
    float fLrf_set_y = 500;
    float scan_angle = (float)360 / 1024;

    vector<uint8_t> vstReflect;
    vector<uint8_t> vstIsForwardPoint;
    vector<float>   vstDistance;
    vector<float>   vstIntensity;
    vector<float>   vstIntrinsicIntensity;
    vector<float>   vstAcuteAngle;
    vector<float>   vstX1;
    vector<float>   vstY1;
    vector<float>   vstX2;
    vector<float>   vstY2;

    unsigned int iLength = 0;

    iLength = msg->fDistance.size();
    for (unsigned int i = 0; i < iLength ; i++)
    {
        fAngle = i * scan_angle * DEG2RAD;

        stPoint.fX = stLrfData.fDistance * cos(fAngle) + fLrf_set_x;
        stPoint.fY = stLrfData.fDistance * sin(fAngle) + fLrf_set_y;

        //----------------------------------------------------------------------
        vstReflect.push_back(false);           
        vstIsForwardPoint.push_back(true);     
        vstDistance.push_back(scan[i]->ranges); 
        vstIntensity.push_back(scan[i]->intensities); 
        vstX1.push_back(0);  
        vstY1.push_back(0);    
        vstX2.push_back(~~~~~~);  
        vstY2.push_back(~~~~~~);  
    }

    //data in message type -------------------------------------------------------
    fss_tf_data.header.frame_id     = "fss_tf_data1";
    fss_tf_data.header.stamp        = ros::Time::now() + ros::Duration(9 * 60 * 60); // GMT +9;
    fss_tf_data.tMeasuredTime       = msg->tMeasuredTime;

    fss_tf_data.bIsReflect          = vstReflect;                   //true
    fss_tf_data.bIsForwardPoint     = vstIsForwardPoint;            //false
    fss_tf_data.fDistance           = vstDistance;                  //stLrfData.fDistance
    fss_tf_data.fIntensity          = vstIntensity;                 //stLrfData.fIntensity
    fss_tf_data.fIntrinsicIntensity = vstIntrinsicIntensity;        //???
    fss_tf_data.fAcuteAngle         = vstAcuteAngle;                //???
    fss_tf_data.fX1                 = vstX1;                        //0
    fss_tf_data.fY1                 = vstY1;                        //0
    fss_tf_data.fX2                 = vstX2;                        //stPoint.fX;
    fss_tf_data.fY2                 = vstY2;                        //stPoint.fy;

    rosPub1.publish(fss_tf_data);

    vstLrfDataTemp.clear();
    vstIsForwardPoint.clear();
    vstReflect.clear();
    vstDistance.clear();
    vstIntensity.clear();
    vstIntrinsicIntensity.clear();
    vstAcuteAngle.clear();
    vstX1.clear();
    vstY1.clear();
    vstX2.clear();
    vstY2.clear();

    double ExeTime = ros::Time::now().toSec() - StartTime;
    printf("ExeTime = %f sec\n", ExeTime);
    ROS_INFO_STREAM("completed sending of fss_tf_data1 #e");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pot_viewer");
    ros::NodeHandle nh;

    rosSub1 = nh.subscribe("/LsrScan0", 1000, lrfCallback1);
    rosSub2 = nh.subscribe("/LsrScan1", 1000, lrfCallback2);
    rosPub1 = nh.advertise<tms_msg_ss::fss_tf_data>("fss_tf_data1", 10);
    rosPub2 = nh.advertise<tms_msg_ss::fss_tf_data>("fss_tf_data2", 10);
    ros::spin();
    return (0);
}
