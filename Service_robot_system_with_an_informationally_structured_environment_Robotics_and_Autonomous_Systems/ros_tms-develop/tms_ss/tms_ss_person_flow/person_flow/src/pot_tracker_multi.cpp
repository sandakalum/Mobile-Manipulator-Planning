//----------------------------------------------------------
// @file   : pot_tracker_multi.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.4 (since 2014.05.02)
// @date   : 2014.11.11
//----------------------------------------------------------

#include <ros/ros.h>
#include <pthread.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


pthread_mutex_t mutex_laser = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_target = PTHREAD_MUTEX_INITIALIZER;

#include "opencv/cv.h"
#include <sensor_msgs/LaserScan.h>
#include "tms_ss_pot/define.h"
#include "tms_ss_pot/Target.h"
#include "tms_ss_pot/Laser.h"
#include "tms_ss_pot/Particle_filter.h"
#include "tms_ss_pot/Multiple_particle_filter.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <iostream>
#include <ctime>
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>

std::vector<float> scanData0;
std::vector<float> scanData1;
std::vector<float> scanData2;
std::vector<float> scanData3;
std::vector<float> scanData4;
std::vector<float> scanData5;

std::vector<double> tmp_lrf_x;
std::vector<double> tmp_lrf_y;

CLaser laser;
bool CallbackCalled0 = false;
bool CallbackCalled1 = false;
bool CallbackCalled2 = false;
bool CallbackCalled3 = false;
bool CallbackCalled4 = false;
bool CallbackCalled5 = false;

std::ofstream ofs("laser_map.txt");

void laser_setting();
void Get_backrangedata();
void laser_mapping();
void laser_classfy(int n);
void laser_cluster(int n);

void *Visualization( void *ptr )
{
    int ID;
    float output_x;
    float output_y;
    ros::Rate r(10);
    while (ros::ok())
    {
        pthread_mutex_lock(&mutex_target);
        for (int i = 0; i < MAX_TRACKING_OBJECT; i++)
        {
            char str[20];
            if (laser.m_pTarget[i] != NULL)
            {
                //std::cout << "OK! " << std::endl;
                if (laser.m_pTarget[i]->cnt < 200)
                {
                    //cout << laser.m_pTarget[i]->cnt << endl;
                    continue;
                }
                ID = laser.m_pTarget[i]->id;
                output_x = - (laser.m_pTarget[i]->py);
                output_y =    laser.m_pTarget[i]->px ;
                std::cout << "ID" << ID << "output_x " << output_x << "output_y" << output_y << std::endl;
                sprintf(str, "txt/ID%d.txt", ID);
                std::cout << "str " << str << std::endl;
                std::ofstream strw;
                strw.open(str, std::ofstream::out | std::ofstream::app);
                strw << output_x << " " << output_y << std::endl;
            }
        }
        pthread_mutex_unlock(&mutex_target);
        r.sleep();
    }

    return 0;
}


void *Processing( void *ptr )
{
    CMultipleParticleFilter m_PF;
    ros::Rate r(30);
    laser.Init();
    laser_setting();

    if (laser.m_bNodeActive[0]) while (!CallbackCalled0) r.sleep();
    if (laser.m_bNodeActive[1]) while (!CallbackCalled1) r.sleep();
    if (laser.m_bNodeActive[2]) while (!CallbackCalled2) r.sleep();
    if (laser.m_bNodeActive[3]) while (!CallbackCalled3) r.sleep();
    if (laser.m_bNodeActive[4]) while (!CallbackCalled4) r.sleep();
    if (laser.m_bNodeActive[5]) while (!CallbackCalled5) r.sleep();

    if (laser.m_bResetBackRangeData == true)
    {
        for (int it = 0; it < laser.m_ring; it++)
        {
            Get_backrangedata();
            r.sleep();
        }
        laser.m_bResetBackRangeData = false;
    }
    std::cout << "---Back range data is stored---" << std::endl;
    std::cout << "---mapping start---" << std::endl;
    laser_mapping();
    std::cout << "---mapping end---" << std::endl;

    int iteration = 0;
    while (ros::ok())
    {
        if (laser.m_bResetBackRangeData == false)
        {
            for (int n = 0; n < laser.m_cnMaxConnect; n++)
            {
                if (laser.m_bNodeActive[n])
                {
                    pthread_mutex_lock(&mutex_laser);
                    laser.m_LRFData[n].clear();
                    laser_classfy(n);
                    pthread_mutex_unlock(&mutex_laser);
                    laser.GetDiffLRFCluster(n);
                    laser_cluster(n);
                }
                if (n == laser.m_cnMaxConnect - 1)
                {
                    laser.m_LRFClsPoints[0].clear();
                    laser.m_LRFClsPoints[0].resize(tmp_lrf_x.size());
                    for (int i = 0; i < tmp_lrf_x.size() ; i++)
                    {
                        laser.m_LRFClsPoints[0][i].x = tmp_lrf_x[i];
                        laser.m_LRFClsPoints[0][i].y = tmp_lrf_y[i];
                    }
                    tmp_lrf_x.clear();
                    tmp_lrf_y.clear();
                }
            }
        }

        pthread_mutex_lock(&mutex_target);
        m_PF.update(&laser);
        pthread_mutex_unlock(&mutex_target);

        ros::Time begin = ros::Time::now();
        if (m_PF.m_ParticleFilter.size() > 0) std::cout << "Time " << begin << " Number of PFs " << m_PF.m_ParticleFilter.size() << std::endl;

        if (!(iteration % 10000)) laser.GetBackLRFDataGaussian();

        r.sleep();
        iteration ++;
    }
    return 0;
}

void laser_setting()
{
    /***all_setting*******************/
    laser.m_bNodeActive[0] = true;
    laser.m_bNodeActive[1] = true;
    laser.m_bNodeActive[2] = false;
    laser.m_bNodeActive[3] = false;
    laser.m_bNodeActive[4] = false;
    laser.m_bNodeActive[5] = false;
    laser.m_nConnectNum = 6;
    laser.GetLRFParam();
    /**********************************/

    /***laser_setting0******************/
    laser.m_nStep[0] = laser.m_LRFParam[0].step;
    laser.m_StartAngle[0] = -laser.m_LRFParam[0].viewangle / 2.0;
    laser.m_DivAngle[0] = laser.m_LRFParam[0].divangle;
    /**********************************/

    /***laser_setting1******************/
    laser.m_nStep[1] = laser.m_LRFParam[1].step;
    laser.m_StartAngle[1] = -laser.m_LRFParam[1].viewangle / 2.0;
    laser.m_DivAngle[1] = laser.m_LRFParam[1].divangle;
    /**********************************/

    /***laser_setting2******************/
    laser.m_nStep[2] = laser.m_LRFParam[2].step;
    laser.m_StartAngle[2] = -laser.m_LRFParam[2].viewangle / 2.0;
    laser.m_DivAngle[2] = laser.m_LRFParam[2].divangle;
    /**********************************/

    /***laser_setting3******************/
    laser.m_nStep[3] = laser.m_LRFParam[3].step;
    laser.m_StartAngle[3] = -laser.m_LRFParam[3].viewangle / 2.0;
    laser.m_DivAngle[3] = laser.m_LRFParam[3].divangle;
    /**********************************/

    /***laser_setting4******************/
    laser.m_nStep[4] = laser.m_LRFParam[4].step;
    laser.m_StartAngle[4] = -laser.m_LRFParam[4].viewangle / 2.0;
    laser.m_DivAngle[4] = laser.m_LRFParam[4].divangle;
    /**********************************/

    /***laser_setting5******************/
    laser.m_nStep[5] = laser.m_LRFParam[5].step;
    laser.m_StartAngle[5] = -laser.m_LRFParam[5].viewangle / 2.0;
    laser.m_DivAngle[5] = laser.m_LRFParam[5].divangle;
    /**********************************/
}

void Get_backrangedata()
{
    for (int n = 0; n < laser.m_cnMaxConnect; n++)
    {
        if (laser.m_bNodeActive[n])
        {

            pthread_mutex_lock(&mutex_laser);

            laser.m_LRFData[n].clear();
            if (n == 0)
            {
                laser.m_LRFData[n].resize(scanData0.size());
                for (int i = 0; i < scanData0.size(); i++) laser.m_LRFData[n][i] = scanData0[i];
            }
            if (n == 1)
            {
                laser.m_LRFData[n].resize(scanData1.size());
                for (int i = 0; i < scanData1.size(); i++) laser.m_LRFData[n][i] = scanData1[i];
            }
            if (n == 2)
            {
                laser.m_LRFData[n].resize(scanData2.size());
                for (int i = 0; i < scanData2.size(); i++) laser.m_LRFData[n][i] = scanData2[i];
            }
            if (n == 3)
            {
                laser.m_LRFData[n].resize(scanData3.size());
                for (int i = 0; i < scanData3.size(); i++) laser.m_LRFData[n][i] = scanData3[i];
            }
            if (n == 4)
            {
                laser.m_LRFData[n].resize(scanData4.size());
                for (int i = 0; i < scanData4.size(); i++) laser.m_LRFData[n][i] = scanData4[i];
            }
            if (n == 5)
            {
                laser.m_LRFData[n].resize(scanData5.size());
                for (int i = 0; i < scanData5.size(); i++) laser.m_LRFData[n][i] = scanData5[i];
            }
            pthread_mutex_unlock(&mutex_laser);
        }
    }
    laser.GetBackLRFDataGaussian();
}

void laser_mapping()
{
    int count;
    double theta, range;
    CvMat *m_Rotate = cvCreateMat(2, 2, CV_64F);
    CvMat *m_Translate = cvCreateMat(2, 1, CV_64F);
    CvMat *Temp = cvCreateMat(2, 1, CV_64F);

    for (int n = 0; n < laser.m_cnMaxConnect; n++)
    {
        if (laser.m_bNodeActive[n])
        {
            cvmSet(m_Rotate, 0, 0, cos(deg2rad(laser.m_LRFParam[n].rz)));
            cvmSet(m_Rotate, 0, 1, -sin(deg2rad(laser.m_LRFParam[n].rz)));
            cvmSet(m_Rotate, 1, 0, sin(deg2rad(laser.m_LRFParam[n].rz)));
            cvmSet(m_Rotate, 1, 1, cos(deg2rad(laser.m_LRFParam[n].rz)));
            cvmSet(m_Translate, 0, 0, laser.m_LRFParam[n].tx);
            cvmSet(m_Translate, 1, 0, laser.m_LRFParam[n].ty);

            laser.m_LRFPoints[n].clear();
            laser.m_LRFPoints[n].resize(laser.m_LRFData[n].size());
            count = 0 ;
            for ( int i = 0; i < laser.m_LRFData[n].size(); i++)
            {
                if (laser.m_BackLRFDataAve[n][i] < 5.0)
                {
                    range = laser.m_BackLRFDataAve[n][i];
                }
                else
                {
                    range = 0;
                }

                theta = laser.m_DivAngle[n] * count + laser.m_StartAngle[n];
                cvmSet(laser.m_LRFPos[n][i], 0, 0, range * cos(deg2rad(theta)));
                cvmSet(laser.m_LRFPos[n][i], 1, 0, range * sin(deg2rad(theta)));
                cvmMul(m_Rotate, laser.m_LRFPos[n][i], Temp);    //
                cvmAdd(m_Translate, Temp, laser.m_LRFPos[n][i]);  //
                laser.m_LRFPoints[n][i].x = cvmGet(laser.m_LRFPos[n][i], 0, 0) * 1000.0;
                laser.m_LRFPoints[n][i].y = cvmGet(laser.m_LRFPos[n][i], 1, 0) * 1000.0;
                //double r = sqrt(pow(laser.m_LRFPoints[n][i].x - laser.m_LRFParam[n].tx, 2) + pow(laser.m_LRFPoints[n][i].y - laser.m_LRFParam[n].ty, 2));
                ofs << -(laser.m_LRFPoints[n][i].y) << " " << laser.m_LRFPoints[n][i].x << " " <<  std::endl;
                count++;
            }
        }
    }
}

void laser_classfy(int n)
{
    switch (n)
    {
    case 0:
        laser.m_LRFData[n].resize(scanData0.size());
        for (int i = 0; i < scanData0.size(); i++)
        {
            laser.m_LRFData[n][i] = scanData0[i];
        }
        break;
    case 1:
        laser.m_LRFData[n].resize(scanData1.size());
        for (int i = 0; i < scanData1.size(); i++)
        {
            laser.m_LRFData[n][i] = scanData1[i];
        }
        break;
    case 2:
        laser.m_LRFData[n].resize(scanData2.size());
        for (int i = 0; i < scanData2.size(); i++)
        {
            laser.m_LRFData[n][i] = scanData2[i];
        }
    case 3:
        laser.m_LRFData[n].resize(scanData3.size());
        for (int i = 0; i < scanData3.size(); i++)
        {
            laser.m_LRFData[n][i] = scanData3[i];
        }
        break;
    case 4:
        laser.m_LRFData[n].resize(scanData4.size());
        for (int i = 0; i < scanData4.size(); i++)
        {
            laser.m_LRFData[n][i] = scanData4[i];
        }
        break;
    case 5:
        laser.m_LRFData[n].resize(scanData5.size());
        for (int i = 0; i < scanData5.size(); i++)
        {
            laser.m_LRFData[n][i] = scanData5[i];
        }

        break;
    }
}

void laser_cluster(int n)
{
    int count;
    double theta, range;
    CvMat *m_Rotate = cvCreateMat(2, 2, CV_64F);
    CvMat *m_Translate = cvCreateMat(2, 1, CV_64F);
    CvMat *Temp = cvCreateMat(2, 1, CV_64F);
    laser.m_LRFClsPoints[n].clear();
    laser.m_LRFClsPoints[n].resize(laser.m_LRFClsData[n].size());

    cvmSet(m_Rotate, 0, 0, cos(deg2rad(laser.m_LRFParam[n].rz)));
    cvmSet(m_Rotate, 0, 1, -sin(deg2rad(laser.m_LRFParam[n].rz)));
    cvmSet(m_Rotate, 1, 0, sin(deg2rad(laser.m_LRFParam[n].rz)));
    cvmSet(m_Rotate, 1, 1, cos(deg2rad(laser.m_LRFParam[n].rz)));
    cvmSet(m_Translate, 0, 0, laser.m_LRFParam[n].tx);
    cvmSet(m_Translate, 1, 0, laser.m_LRFParam[n].ty);

    for (int i = 0; i < laser.m_LRFClsData[n].size(); i++)
    {
        count = laser.m_LRFClsData[n][i].n;
        range = laser.m_LRFClsData[n][i].range;
        theta = laser.m_DivAngle[0] * count + laser.m_StartAngle[0];
        theta = laser.m_DivAngle[1] * count + laser.m_StartAngle[1];

        cvmSet(laser.m_LRFClsPos[n][i], 0, 0, range * cos(deg2rad(theta)));
        cvmSet(laser.m_LRFClsPos[n][i], 1, 0, range * sin(deg2rad(theta)));
        cvmMul(m_Rotate, laser.m_LRFClsPos[n][i], Temp);    //
        cvmAdd(m_Translate, Temp, laser.m_LRFClsPos[n][i]);  //

        laser.m_LRFClsPoints[n][i].x = cvmGet(laser.m_LRFClsPos[n][i], 0, 0) * 1000.0;
        laser.m_LRFClsPoints[n][i].y = cvmGet(laser.m_LRFClsPos[n][i], 1, 0) * 1000.0;
        tmp_lrf_x.push_back(laser.m_LRFClsPoints[n][i].x);
        tmp_lrf_y.push_back(laser.m_LRFClsPoints[n][i].y);
    }
    cvReleaseMat(&Temp);
    cvReleaseMat(&m_Rotate);
    cvReleaseMat(&m_Translate);
}

void LaserSensingCallback0(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //std::cout << "size0" << scan->ranges.size() << std::endl;
    if (scan->ranges.size() > 0)
    {
        pthread_mutex_lock(&mutex_laser);
        int steps = 683;
        if ( scanData0.size() == 0 ) scanData0.resize(steps);

        for (int i = 0, j = scan->ranges.size() - steps; i < steps ; i++, j++)
        {
            scanData0[i] = scan->ranges[j];
        }
        pthread_mutex_unlock(&mutex_laser);
        CallbackCalled0 = true;
    }
}

void LaserSensingCallback1(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //std::cout << "size1" << scan->ranges.size() << std::endl;
    if (scan->ranges.size() > 0)
    {
        pthread_mutex_lock(&mutex_laser);
        int steps = 683;
        if ( scanData1.size() == 0 ) scanData1.resize(steps);

        for (int i = 0, j = scan->ranges.size() - steps; i < steps ; i++, j++)
        {
            scanData1[i] = scan->ranges[j];
        }
        pthread_mutex_unlock(&mutex_laser);
        CallbackCalled1 = true;
    }
}

void LaserSensingCallback2(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    std::cout << "size2" << scan->ranges.size() << std::endl;
    if (scan->ranges.size() > 0)
    {
        pthread_mutex_lock(&mutex_laser);
        int steps = 683;
        if ( scanData2.size() == 0 ) scanData2.resize(steps);

        for (int i = 0, j = scan->ranges.size() - steps; i < steps ; i++, j++)
        {
            scanData2[i] = scan->ranges[j];
        }
        pthread_mutex_unlock(&mutex_laser);
        CallbackCalled2 = true;
    }
}

void LaserSensingCallback3(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    std::cout << "size3" << scan->ranges.size() << std::endl;
    if (scan->ranges.size() > 0)
    {
        pthread_mutex_lock(&mutex_laser);
        int steps = 683;
        if ( scanData3.size() == 0 ) scanData3.resize(steps);

        for (int i = 0, j = scan->ranges.size() - steps; i < steps ; i++, j++)
        {
            scanData3[i] = scan->ranges[j];
        }
        pthread_mutex_unlock(&mutex_laser);
        CallbackCalled3 = true;
    }
}

void LaserSensingCallback4(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    std::cout << "size4" << scan->ranges.size() << std::endl;
    if (scan->ranges.size() > 0)
    {
        pthread_mutex_lock(&mutex_laser);
        int steps = 683;
        if ( scanData4.size() == 0 ) scanData4.resize(steps);

        for (int i = 0, j = scan->ranges.size() - steps; i < steps ; i++, j++)
        {
            scanData4[i] = scan->ranges[j];
        }
        pthread_mutex_unlock(&mutex_laser);
        CallbackCalled4 = true;
    }
}

void LaserSensingCallback5(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    std::cout << "size5" << scan->ranges.size() << std::endl;
    if (scan->ranges.size() > 0)
    {
        pthread_mutex_lock(&mutex_laser);
        int steps = 683;
        if ( scanData5.size() == 0 ) scanData5.resize(steps);

        for (int i = 0, j = scan->ranges.size() - steps; i < steps ; i++, j++)
        {
            scanData5[i] = scan->ranges[j];
        }
        pthread_mutex_unlock(&mutex_laser);
        CallbackCalled5 = true;
    }
}

int main( int argc, char **argv )
{
    std::cout << "---pot_tracking_multi start---" << std::endl;
    pthread_t thread_p;
    pthread_t thread_v;
    ros::MultiThreadedSpinner spinner(4);

    ros::init(argc, argv, "pot_tracker_multi");
    ros::NodeHandle n;
    ros::Publisher  pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    ros::Subscriber sub0 = n.subscribe("/LaserTracker0" , 1000, LaserSensingCallback0);
    ros::Subscriber sub1 = n.subscribe("/LaserTracker1" , 1000, LaserSensingCallback1);
    ros::Subscriber sub2 = n.subscribe("/LaserTracker2" , 1000, LaserSensingCallback2);
    ros::Subscriber sub3 = n.subscribe("/LaserTracker3" , 1000, LaserSensingCallback3);
    ros::Subscriber sub4 = n.subscribe("/LaserTracker4" , 1000, LaserSensingCallback4);
    ros::Subscriber sub5 = n.subscribe("/LaserTracker5" , 1000, LaserSensingCallback5);

    if ( pthread_create( &thread_v, NULL, Visualization, (void *)&pub) )
    {
        printf("error creating thread.");
        abort();
    }

    if ( pthread_create( &thread_p, NULL, Processing, NULL) )
    {
        printf("error creating thread.");
        abort();
    }

    spinner.spin(); // spin() will not return until the node has been shutdown

    ros::waitForShutdown();

    if ( pthread_join( thread_p, NULL) )
    {
        printf("error joining thread.");
        abort();
    }

    if ( pthread_join( thread_v, NULL) )
    {
        printf("error joining thread.");
        abort();
    }


    scanData0.clear();
    scanData1.clear();
    scanData2.clear();
    scanData3.clear();
    scanData4.clear();
    scanData5.clear();

    return 0;
}

