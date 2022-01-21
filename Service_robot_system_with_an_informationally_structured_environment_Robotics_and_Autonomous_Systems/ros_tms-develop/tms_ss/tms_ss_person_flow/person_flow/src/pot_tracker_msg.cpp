//----------------------------------------------------------
// @file   : pot_tracker_double.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.4 (since 2014.05.02)
// @date   : 2016.06.09
//----------------------------------------------------------

#include <ros/ros.h>
#include <pthread.h>
#include <std_msgs/String.h>
#include <tms_msg_ss/pot_tracking_points.h>
#include <tms_msg_ss/pot_tracking_grid.h>

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

std::vector<float> scanData;
CLaser laser;
bool CallbackCalled = false;
IDparam tmp_param;
std::vector< IDparam > idparam;
std::vector< IDparam >::iterator v;

void *Visualization( void *ptr )
{
    int count = 0;
    int   ID;
    float X;
    float Y;
    ros::Rate r(10);
    ros::Publisher *pub = (ros::Publisher *)ptr;

    tmp_param.id = 0;
    tmp_param.flag = 0;
    tmp_param.count = 0;
    tmp_param.start_pos.x = 0.0;
    tmp_param.start_pos.y = 0.0;
    tmp_param.tmp_pos.x = 0.0;
    tmp_param.tmp_pos.y = 0.0;
    tmp_param.end_pos.x = 0.0;
    tmp_param.end_pos.y = 0.0;
    tmp_param.vector.x = 0.0;
    tmp_param.vector.y = 0.0;
    idparam.push_back(tmp_param);

    while (ros::ok())
    {
        pthread_mutex_lock(&mutex_target);
        //std::cout << "visual_start"<<std::endl;
        tms_msg_ss::pot_tracking_grid grid;
        tms_msg_ss::pot_tracking_points points;

        for (int i = 0; i < MAX_TRACKING_OBJECT; i++)
        {
            if (laser.m_pTarget[i] != NULL)
            {
                if (laser.m_pTarget[i]->cnt < 200)
                {
                    //cout << laser.m_pTarget[i]->cnt << endl;
                    continue;
                }
                //std::cout << "laser.m_pTarget " << laser.m_pTarget[i]->cnt << std::endl;
                ID =  (laser.m_pTarget[i]->id);
                X  = -(laser.m_pTarget[i]->py);
                Y  =  (laser.m_pTarget[i]->px);
                //std::cout << "ID " << ID << "X " << X << "Y " << Y << std::endl;

                for (v = idparam.begin(); v != idparam.end(); ++v)
                {
                    v -> flag = 0;
                }


                for (v = idparam.begin(); v != idparam.end(); ++v)
                {
                    if ( v -> id == ID)
                    {
                        v -> flag = 1;
                        v -> count++;
                        if ((v -> count) % 10 == 0)
                        {
                            v -> vector.x = X - (v -> tmp_pos.x);
                            v -> vector.y = Y - (v -> tmp_pos.y);
                            v -> tmp_pos.x = X;
                            v -> tmp_pos.y = Y;
                        }
                        v -> end_pos.x = X;
                        v -> end_pos.y = Y;
                        //std::cout << "v = id " << std::endl;
                        break;
                    }
                    else if (v == idparam.end() - 1 )
                    {
                        tmp_param.id = ID;
                        tmp_param.flag = 1;
                        tmp_param.count = 0;
                        tmp_param.start_pos.x = X;
                        tmp_param.start_pos.y = Y;
                        tmp_param.tmp_pos.x = X;
                        tmp_param.tmp_pos.y = Y;
                        tmp_param.end_pos.x = X;
                        tmp_param.end_pos.y = Y;
                        tmp_param.vector.x = X;
                        tmp_param.vector.y = Y;
                        idparam.insert(idparam.begin(), tmp_param);
                        break;
                    }
                }

                for (v = idparam.begin(); v != idparam.end(); ++v)
                {
                    grid.id = v -> id;
                    grid.flag = v -> flag;
                    grid.count = v -> count;
                    grid.start_x = v -> start_pos.x;
                    grid.start_y = v -> start_pos.y;
                    grid.tmp_x = v -> tmp_pos.x;
                    grid.tmp_y = v -> tmp_pos.y;
                    grid.end_x = v -> end_pos.x;
                    grid.end_y = v -> end_pos.y;
                    grid.vector_x = v -> vector.x;
                    grid.vector_y = v -> vector.y;

                    //std::cout << "id " << grid.id << "flag " << grid.flag << "x " << grid.end_x << "y " << grid.end_y << std::endl;
                    points.pot_tracking_grid.push_back(grid);
                }

                    for (v = idparam.begin(); v != idparam.end(); ++v)
                    {
                        //std::cout << "v -> flag" << v -> flag << std::endl;
                        if (v -> flag == 0 && v -> id != 0)
                        {
                            idparam.erase (v);
                        }
                    }
            }

        }

        pthread_mutex_unlock(&mutex_target);
        pub->publish(points);
        //std::cout << "visual_end"<<std::endl;
        count++;
        r.sleep();
    }
    return 0;
}


void *Processing( void *ptr )
{
    //  CParticleFilter m_PF;
    CMultipleParticleFilter m_PF;
    //ros::Rate r(30);
    ros::Rate r(30);
    laser.Init();

    //  int area[2]={STAGE_X, STAGE_Y};
    //  m_PF.initialize(area, MAX_PARTICLE_NUM, MAX_PARTICLE_NUM_MCMC);

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
            for (int n = 0; n < laser.m_cnMaxConnect; n++)
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
                    laser.m_LRFData[n].resize(scanData.size());
                    for (int i = 0; i < scanData.size(); i++)
                    {
                        laser.m_LRFData[n][i] = scanData[i];
                    }

                    pthread_mutex_unlock(&mutex_laser);

                    laser.GetDiffLRFCluster(n);

                    cvmSet(m_Rotate, 0, 0, cos(deg2rad(laser.m_LRFParam[n].rz)));
                    cvmSet(m_Rotate, 0, 1, -sin(deg2rad(laser.m_LRFParam[n].rz)));
                    cvmSet(m_Rotate, 1, 0, sin(deg2rad(laser.m_LRFParam[n].rz)));
                    cvmSet(m_Rotate, 1, 1, cos(deg2rad(laser.m_LRFParam[n].rz)));
                    cvmSet(m_Translate, 0, 0, laser.m_LRFParam[n].tx);
                    cvmSet(m_Translate, 1, 0, laser.m_LRFParam[n].ty);

                    laser.m_LRFPoints[n].clear();
                    laser.m_LRFPoints[n].resize(laser.m_DiffLRFData[n].size());

                    for ( int i = 0; i < laser.m_DiffLRFData[n].size(); i++)
                    {
                        count = laser.m_DiffLRFData[n][i].n;
                        range = laser.m_DiffLRFData[n][i].range;
                        theta = laser.m_DivAngle[0] * count + laser.m_StartAngle[0];

                        cvmSet(laser.m_LRFPos[n][i], 0, 0, range * cos(deg2rad(theta)));
                        cvmSet(laser.m_LRFPos[n][i], 1, 0, range * sin(deg2rad(theta)));
                        cvmMul(m_Rotate, laser.m_LRFPos[n][i], Temp);    //
                        cvmAdd(m_Translate, Temp, laser.m_LRFPos[n][i]);  //
                        laser.m_LRFPoints[n][i].x = cvmGet(laser.m_LRFPos[n][i], 0, 0) * 1000.0;
                        laser.m_LRFPoints[n][i].y = cvmGet(laser.m_LRFPos[n][i], 1, 0) * 1000.0;
                    }

                    // Number of clusters
                    // std::cout << "Number of clusters " << laser.m_LRFClsData[n].size() << std::endl;

                    laser.m_LRFClsPoints[n].clear();
                    laser.m_LRFClsPoints[n].resize(laser.m_LRFClsData[n].size());
                    for (int i = 0; i < laser.m_LRFClsData[n].size(); i++)
                    {
                        count = laser.m_LRFClsData[n][i].n;
                        range = laser.m_LRFClsData[n][i].range;
                        theta = laser.m_DivAngle[0] * count + laser.m_StartAngle[0];

                        cvmSet(laser.m_LRFClsPos[n][i], 0, 0, range * cos(deg2rad(theta)));
                        cvmSet(laser.m_LRFClsPos[n][i], 1, 0, range * sin(deg2rad(theta)));
                        cvmMul(m_Rotate, laser.m_LRFClsPos[n][i], Temp);    //
                        cvmAdd(m_Translate, Temp, laser.m_LRFClsPos[n][i]);  //
                        laser.m_LRFClsPoints[n][i].x = cvmGet(laser.m_LRFClsPos[n][i], 0, 0) * 1000.0;
                        laser.m_LRFClsPoints[n][i].y = cvmGet(laser.m_LRFClsPos[n][i], 1, 0) * 1000.0;

                        //std::cout << laser.m_LRFClsPoints[n][i].x << " " << laser.m_LRFClsPoints[n][i].y << std::endl;

                    }
                }
            }
        }
        pthread_mutex_lock(&mutex_target);
        m_PF.update(&laser);
        pthread_mutex_unlock(&mutex_target);

        ros::Time begin = ros::Time::now();
        if (m_PF.m_ParticleFilter.size() > 0) std::cout << "Time " << begin << " Number of PFs " << m_PF.m_ParticleFilter.size() << std::endl;

        if (!(iteration % 100)) laser.GetBackLRFDataGaussian();

        r.sleep();
        iteration ++;
    }

    cvReleaseMat(&Temp);
    cvReleaseMat(&m_Rotate);
    cvReleaseMat(&m_Translate);

}

void LaserSensingCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    pthread_mutex_lock(&mutex_laser);
    int num = floor((scan->angle_max - scan->angle_min) / scan->angle_increment);

    if ( scanData.size() == 0 ) scanData.resize(num);

    for (int i = 0; i < num ; i++)
    {
        if (isnan(scan->ranges[i]) == 0)
        {
            scanData[i] = scan->ranges[i];
        }
        else
        {
            scanData[i] = 5.6;
        }
    }
    pthread_mutex_unlock(&mutex_laser);
    CallbackCalled = true;
}

int main( int argc, char **argv )
{
    std::cout << "tracker_start" << std::endl;
    pthread_t thread_p;
    pthread_t thread_v;
    ros::MultiThreadedSpinner spinner(0);

    ros::init(argc, argv, "pot_tracker_msg");
    ros::NodeHandle n;
    ros::Publisher  pub = n.advertise<tms_msg_ss::pot_tracking_points>("tracking_point_inf", 10);
    ros::Subscriber sub = n.subscribe("/LaserTracker0", 1000, LaserSensingCallback);
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

    scanData.clear();

    return 0;
}

