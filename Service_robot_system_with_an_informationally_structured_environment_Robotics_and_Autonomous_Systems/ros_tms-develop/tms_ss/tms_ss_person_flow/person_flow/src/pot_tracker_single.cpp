//----------------------------------------------------------
// @file   : pot_tracker_single.cpp
// @author : Watanabe Yuuta 
// @version: Ver0.1.4 (since 2014.05.02)
// @date   : 2016.06.09
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

#include <tms_msg_db/TmsdbGetData.h>
#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_db/Tmsdb.h>

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
std::ofstream ofs("plot2.txt");

void *Visualization( void *ptr )
{
    uint32_t shape = visualization_msgs::Marker::CYLINDER;
    uint32_t shape_arrow = visualization_msgs::Marker::ARROW;
    float output_x[10];
    float output_y[10];
    float b_output_x[10];
    float b_output_y[10];
    int rlcount = 0;
    ros::Rate r(10);
    ros::Publisher *pub = (ros::Publisher *)ptr;
    visualization_msgs::MarkerArray latest_markerArray;
    int latest_id = 0;
    float colorset[14][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {1, 1, 0, 0}, {0, 1, 1, 0}, {1, 0, 1, 0}, {1, 1, 1, 0},
        {0.5, 0, 0, 0}, {0, 0.5, 0, 0}, {0, 0, 0.5, 0}, {0.5, 0.5, 0, 0}, {0, 0.5, 0.5, 0}, {0.5, 0, 0.5, 0}, {0.5, 0.5, 0.5, 0}
    };

    while (ros::ok())
    {
        visualization_msgs::MarkerArray markerArray;
        //    markerArray.markers.clear();

        int id = 0;
        int flag = 0;
        for (int i = 0; i < 3; i++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/laser";
            marker.header.stamp = ros::Time::now();

            //    marker.ns = "tracker";
            marker.id = id;
            marker.type = shape_arrow;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 1.0;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;


            switch (i)
            {
            case 0:
                marker.pose.orientation.x = sin(90.0 * PI / 180.0 / 2.0);
                marker.pose.orientation.w = cos(90.0 * PI / 180.0 / 2.0);
                marker.color.r = 1.0f;
                break;
            case 1:
                marker.pose.orientation.z = sin(90.0 * PI / 180.0 / 2.0);
                marker.pose.orientation.w = cos(90.0 * PI / 180.0 / 2.0);
                marker.color.g = 1.0f;
                break;
            case 2:
                marker.pose.orientation.y = sin(-90.0 * PI / 180.0 / 2.0);
                marker.pose.orientation.w = cos(-90.0 * PI / 180.0 / 2.0);
                marker.color.b = 1.0f;
                break;
            }

            //    marker.lifetime = ros::Duration();

            markerArray.markers.push_back(marker);
            id ++;
        }

        pthread_mutex_lock(&mutex_target);

        for (int i = 0; i < MAX_TRACKING_OBJECT; i++)
        {
            if (laser.m_pTarget[i] != NULL)
            {

                if (laser.m_pTarget[i]->cnt < 200)
                {
                    //std::cout << "laser.m_pTarget->cnt" << laser.m_pTarget[i]->cnt << std::endl;
                    continue;
                }
                //cout << laser.m_pTarget[i]->cnt << endl;

                visualization_msgs::Marker marker;
                marker.header.frame_id = "/laser";
                marker.header.stamp = ros::Time::now();

                //         marker.ns = "tracker";
                marker.id = id;
                marker.type = shape;
                marker.action = visualization_msgs::Marker::ADD;

                marker.pose.position.x = laser.m_pTarget[i]->px * 0.001;
                marker.pose.position.y = laser.m_pTarget[i]->py * 0.001;
                std::cout << "x y " << laser.m_pTarget[i]->px << " " << laser.m_pTarget[i]->py;
                marker.pose.position.z = 0.5;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 1.0;

                int color = laser.m_pTarget[i]->id % 14;
                std::cout << "laser.m_pTarget->id " << laser.m_pTarget[i]->id << std::endl;
                marker.color.r = colorset[color][0];
                marker.color.g = colorset[color][1];
                marker.color.b = colorset[color][2];
                marker.color.a = 1.0;

                //         marker.lifetime = ros::Duration();

                markerArray.markers.push_back(marker);
                id ++;
                output_x[i] = - (laser.m_pTarget[i]->py);
                output_y[i] =    laser.m_pTarget[i]->px;

                ofs << output_x[i] << " " << output_y[i] << " " ;
            }
            else
            {
                ofs << "\t\t" ;
            }
        }
        ofs << std::endl ;
        if (id - 3 > 0) std::cout << "Number of Markers " << id - 3 << std::endl;

        pthread_mutex_unlock(&mutex_target);

        //std::cout << "Number of Markers " << markerArray.markers.size() << std::endl;

        if (latest_id > id)
        {
            for (int i = 0; i < latest_markerArray.markers.size(); i++) latest_markerArray.markers[i].action = visualization_msgs::Marker::DELETE;
            pub->publish(latest_markerArray);
        }

        latest_id = id;
        latest_markerArray = markerArray;

        pub->publish(markerArray);
        //std::cout << "pub"<<std::endl;

        r.sleep();
    }

    return 0;
}

void *Processing( void *ptr )
{
    //  CParticleFilter m_PF;
    CMultipleParticleFilter m_PF;
    //ros::Rate r(30);
    ros::Rate r(20);
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
    ros::MultiThreadedSpinner spinner(4);

    ros::init(argc, argv, "pot_tracker_single");
    ros::NodeHandle n;
    ros::Publisher  pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
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

