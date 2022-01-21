//----------------------------------------------------------
// @file   : pot_mapping.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.4 (since 2014.05.02)
// @date   : 2014.11.12
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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <iostream>
#include <ctime>
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>

std::vector<float> scanData;
CLaser laser;
bool CallbackCalled = false;
std::ofstream ofs("map.txt");

void *Processing( void *ptr )
{
    ros::Rate r(30);
    laser.Init();

    /**********************************/
    laser.m_bNodeActive[0] = true;
    laser.m_nConnectNum = 1;
    laser.GetLRFParam();
    laser.m_nStep[0] = laser.m_LRFParam[0].step;
    laser.m_StartAngle[0] = -laser.m_LRFParam[0].viewangle / 2.0;
    laser.m_DivAngle[0] = laser.m_LRFParam[0].divangle;
    /**********************************/
    CvMat *m_Rotate = cvCreateMat(2, 2, CV_64F);
    CvMat *m_Translate = cvCreateMat(2, 1, CV_64F);
    CvMat *Temp = cvCreateMat(2, 1, CV_64F);

    int count;
    double theta, range;

    if (laser.m_bNodeActive[0])
        while (!CallbackCalled) r.sleep();

    if (laser.m_bResetBackRangeData == true)
    {
        for (int it = 0; it < laser.m_ring; it++)
        {
            for (int n = 0; n < 1; n++)
            {
                if (laser.m_bNodeActive[n])
                {
                    pthread_mutex_lock(&mutex_laser);

                    laser.m_LRFData[n].clear();
                    laser.m_LRFData[n].resize(scanData.size());
                    for (int i = 0; i < scanData.size(); i++) laser.m_LRFData[n][i] = scanData[i];

                    pthread_mutex_unlock(&mutex_laser);
                }
            }
            //      laser.GetBackLRFData();
            laser.GetBackLRFDataGaussian();
            r.sleep();
        }
        laser.m_bResetBackRangeData = false;
        std::cout << "Back range data is stored" << std::endl;
    }
    /*
        for (int n = 0; n < 1; n++)
        {
            for (int i = 0; i < laser.m_LRFData[n].size(); i++)
            {
                std::cout << "laser.m_LRFData_Ave " << i << " " <<   laser.m_BackLRFDataAve[n][i] << std::endl;
            }
        }
    */


    for (int n = 0; n < 1; n++)
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
            std::cout << "laser.m_DivAngle " <<  laser.m_DivAngle[0] << std::endl;
            for ( int i = 0; i < laser.m_LRFData[n].size(); i++)
            {
                range = laser.m_BackLRFDataAve[n][i];
                theta = laser.m_DivAngle[0] * count + laser.m_StartAngle[0];
                //laser.m_LRFPoints[n][i].x = range * cos(deg2rad(theta));
                //laser.m_LRFPoints[n][i].y = range * sin(deg2rad(theta));
                // std::cout << "theta " <<  theta << std::endl;
                cvmSet(laser.m_LRFPos[n][i], 0, 0, range * cos(deg2rad(theta)));
                cvmSet(laser.m_LRFPos[n][i], 1, 0, range * sin(deg2rad(theta)));
                cvmMul(m_Rotate, laser.m_LRFPos[n][i], Temp);    //
                cvmAdd(m_Translate, Temp, laser.m_LRFPos[n][i]);  //
                laser.m_LRFPoints[n][i].x = cvmGet(laser.m_LRFPos[n][i], 0, 0) * 1000.0;
                laser.m_LRFPoints[n][i].y = cvmGet(laser.m_LRFPos[n][i], 1, 0) * 1000.0;
                ofs << -(laser.m_LRFPoints[n][i].y) << " " << laser.m_LRFPoints[n][i].x << " " <<  std::endl;
                count++;
            }
        }
    }

    std::cout << "mapping end" << std::endl;
    // for (int n = 0; n < 1; n++)
    // {
    //     for (int i = 0; i < laser.m_LRFData[n].size(); i++)
    //     {
    //         std::cout << "laser.m_LRFData_Ave " << i << " " <<   laser.m_BackLRFDataAve[n][i] << std::endl;
    //     }
    // }



    cvReleaseMat(&Temp);
    cvReleaseMat(&m_Rotate);
    cvReleaseMat(&m_Translate);

    return 0;

}

void LaserSensingCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    pthread_mutex_lock(&mutex_laser);
    std::cout << "scan start" << scan->ranges.size() << std::endl;
    int steps = 683;
    if ( scanData.size() == 0 ) scanData.resize(steps);

    for (int i = 0, j = scan->ranges.size() - steps; i < steps ; i++, j++)
    {
        scanData[i] = scan->ranges[j];
        std::cout << "j " << j << "scan->ranges" <<  scan->ranges[j] << std::endl;
    }
    std::cout << "scanData" << scanData.size() << std::endl;
    pthread_mutex_unlock(&mutex_laser);
    CallbackCalled = true;
}

int main( int argc, char **argv )
{
    std::cout << "mapping start" << std::endl;
    pthread_t thread_p;
    pthread_t thread_v;
    ros::MultiThreadedSpinner spinner(1);

    ros::init(argc, argv, "pot_tmapping");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/LaserTracker0", 1000, LaserSensingCallback);

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
    scanData.clear();

    return 0;
}

