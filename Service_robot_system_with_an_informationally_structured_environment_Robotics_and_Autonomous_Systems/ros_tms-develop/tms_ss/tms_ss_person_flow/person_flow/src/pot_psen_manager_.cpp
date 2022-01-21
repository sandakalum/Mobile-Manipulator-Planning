//----------------------------------------------------------
// @file   : pot_psen_manager.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.4 (since 2014.05.02)
// @date   : 2014.11.18
//----------------------------------------------------------

#include <ros/ros.h>
#include <pthread.h>
#include <std_msgs/String.h>
#include <tms_msg_ss/pot_tracking_psen.h>
#include <tms_msg_ss/pot_tracking_psens.h>

pthread_mutex_t mutex_laser  = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_target = PTHREAD_MUTEX_INITIALIZER;

#include "tms_ss_pot/define.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <iostream>
#include <ctime>
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

#define MODE_START        0
#define MODE_END          5
#define LRF_NUMBER_START  0
#define LRF_NUMBER_END    11
#define INIT_VALUE_START  0
#define INIT_VALUE_END    7
#define FEATURE_START     0
#define FEATURE_END       8

#define PORTABLE_DISCRIMINATION_AVERAGE   85.56
#define PORTABLE_DISCRIMINATION_VARIANCE   6.14
#define FIXED_SIGMA_NUMBER                 3.0
#define RECONFIGURRABLE_START_THRESHHOLD   20
#define REMOVE_THRESHHOLD                  2
#define TIMER_MINUTE                       0.1//15.0//0.1//1.0//0.1//15.0
#define TIMER_MINUTE_SUB                   0.2//25.0//0.1//1.0//0.1//15.0

std::ofstream ofs1("reconf/1_inout.txt");
std::ofstream ofs2("reconf/2_inout.txt");
std::ofstream ofs3("reconf/3_inout.txt");
std::ofstream ofs4("reconf/4_inout.txt");


//----------------
//msg to all_cnt
//----------------

typedef struct
{
    int righter;
    int lefter;
} counter;


//-------------------
//all_cnt to all_cmp
//-------------------

typedef struct
{
    int mode;
    std::vector<int>  LRF_number;
    std::vector<int>  init_value;
    int   feature;
    int   inner;
    int   outer;
    float calc_inout;
    float calc_outin;
    float calc_righter_inout;
    float calc_righter_outin;
    float calc_lefter_inout;
    float calc_lefter_outin;

    int   f_righter;
    int   f_lefter;
    int   s_righter;
    int   s_lefter;
    int   t_righter;
    int   t_lefter;
    int   fr_righter;
    int   fr_lefter;

    double time;
} composition;

std::vector<counter> all_cnt;
std::vector<composition> all_cmp;
std::vector<composition> all_cmp_sub;

std::vector<int> righter_adjustment;
std::vector<int> lefter_adjustment;
composition tmp_comp;

std::vector<std::string> comment_data;
std::vector<std::string>::iterator comment_data_it;

int    g_max_feature;
double g_StartTime;
namespace fs = boost::filesystem;

void  lrf_display();
void  under_lrf_display();
void  Reconfigurable_threshhold(float calc_inout,
                                float calc_outin,
                                float calc_righter_inout,
                                float calc_righter_outin,
                                float calc_lefter_inout,
                                float calc_lefter_outin,
                                int mode,
                                int f_feature,
                                double exe_time);

void *getcss( void *ptr )
{
  char number;
  ros::Rate r(2);
  while (ros::ok())
  {
      number = getchar();
      pthread_mutex_lock(&mutex_target);
      switch (number) {
       case ' ': if(comment_data.size() >0){
          comment_data_it = comment_data.begin();
          comment_data.erase(comment_data_it);
          break;
            }
       default:                                                                              break;
       }
        pthread_mutex_unlock(&mutex_target);
      r.sleep();
  }
}
void *Management( void *ptr )
{
    ros::Rate r(30);
    while (ros::ok())
    {
        //------------------------------------------------------------------------------------------------------
        //txt data to all_cmp
        //------------------------------------------------------------------------------------------------------
        const fs::path path("/home/kurt/catkin_ws/position");
        all_cmp.clear();
        BOOST_FOREACH(const fs::path & p, std::make_pair(fs::directory_iterator(path), fs::directory_iterator()))
        {
            if (!fs::is_directory(p) && boost::algorithm::iends_with(p.string(), ".txt"))
            {
                std::ifstream ifs(p.c_str());
                std::string line;
                std::string tmp_line;
                int line_number = 0;
                while (getline(ifs, line))
                {
                    line_number++;
                    //std::cout << line << std::endl;
                    if (line.substr(MODE_START, MODE_END) == "mode:")                           //
                    {
                        int fs;
                        fs = line.find(",");
                        fs = fs - MODE_END;
                        //std::cout << "fs " << fs << std::endl;
                        tmp_line = line.substr(MODE_END, fs);
                        //std::cout << tmp_line << std::endl;
                        tmp_comp.mode = atoi(tmp_line.c_str());
                    }
                    else if (line.substr(LRF_NUMBER_START, LRF_NUMBER_END) == "LRF_number:")    //
                    {
                        //LRFの呼び出し
                        //mode == 1 or mode == 2;
                        if (tmp_comp.mode == 1 || tmp_comp.mode == 2)
                        {
                            int fp = LRF_NUMBER_END;
                            int fn;
                            for (int i = 0; i < 2; i++)
                            {
                                fn = line.find("," , fp);
                                fn = fn - fp;
                                //std::cout << line.substr(fp, fn) << " ";
                                tmp_comp.LRF_number.push_back(atoi(line.substr(fp, fn).c_str()));
                                fp = fp + fn + 1;
                            }
                            //std::cout << std::endl;
                        }
                        else if (tmp_comp.mode == 3 || tmp_comp.mode == 4 || tmp_comp.mode == 5 || tmp_comp.mode == 6)
                        {
                            int fp = LRF_NUMBER_END;
                            int fn;
                            for (int i = 0; i < 3; i++)
                            {
                                fn = line.find("," , fp);
                                fn = fn - fp;
                                //std::cout << line.substr(fp, fn) << " ";
                                tmp_comp.LRF_number.push_back(atoi(line.substr(fp, fn).c_str()));
                                fp = fp + fn + 1;
                            }
                            //std::cout << std::endl;
                        }
                        else if (tmp_comp.mode == 7 || tmp_comp.mode == 8 || tmp_comp.mode == 9 || tmp_comp.mode == 10 || tmp_comp.mode == 11 || tmp_comp.mode == 12 || tmp_comp.mode == 13 || tmp_comp.mode == 14)
                        {
                            int fp = LRF_NUMBER_END;
                            int fn;
                            for (int i = 0; i < 4; i++)
                            {
                                fn = line.find("," , fp);
                                fn = fn - fp;
                                //std::cout << line.substr(fp, fn) << " ";
                                tmp_comp.LRF_number.push_back(atoi(line.substr(fp, fn).c_str()));
                                fp = fp + fn + 1;
                            }
                            //std::cout << std::endl;
                        }

                    }
                    else if (line.substr(INIT_VALUE_START, INIT_VALUE_END) == "inival:")                    //
                    {
                        //mode == 1 or mode == 2;
                        if (tmp_comp.mode == 1 || tmp_comp.mode == 2)
                        {
                            int fp = INIT_VALUE_END;
                            int fn;
                            for (int i = 0; i < 4; i++)
                            {
                                fn = line.find("," , fp);
                                fn = fn - fp;
                                //std::cout << line.substr(fp, fn) << " ";
                                tmp_comp.init_value.push_back(atoi(line.substr(fp, fn).c_str()));
                                fp = fp + fn + 1;
                            }
                            //std::cout << std::endl;
                        }
                        else if (tmp_comp.mode == 3 || tmp_comp.mode == 4 || tmp_comp.mode == 5 || tmp_comp.mode == 6)
                        {
                            int fp = INIT_VALUE_END;
                            int fn;
                            for (int i = 0; i < 6; i++)
                            {
                                fn = line.find("," , fp);
                                fn = fn - fp;
                                //std::cout << line.substr(fp, fn) << " ";
                                tmp_comp.init_value.push_back(atoi(line.substr(fp, fn).c_str()));
                                fp = fp + fn + 1;
                            }
                            //std::cout << std::endl;
                        }
                        else if (tmp_comp.mode == 7 || tmp_comp.mode == 8 || tmp_comp.mode == 9 || tmp_comp.mode == 10 || tmp_comp.mode == 11 || tmp_comp.mode == 12 || tmp_comp.mode == 13 || tmp_comp.mode == 14)
                        {
                            int fp = INIT_VALUE_END;
                            int fn;
                            for (int i = 0; i < 8; i++)
                            {
                                fn = line.find("," , fp);
                                fn = fn - fp;
                                //std::cout << line.substr(fp, fn) << " ";
                                tmp_comp.init_value.push_back(atoi(line.substr(fp, fn).c_str()));
                                fp = fp + fn + 1;
                            }
                            //std::cout << std::endl;
                        }

                    }
                    else if (line.substr(0, 11) == "start_time:")                                              //初期時間
                    {
                        int fp = 11;
                        int fn;
                        fn = line.find("," , fp);
                        fn = fn - fp;
                        tmp_comp.time = atof(line.substr(fp, fn).c_str());
                    }
                    else if (line.substr(FEATURE_START, FEATURE_END) == "feature:")                            //特徴点
                    {
                        int fp = FEATURE_END;
                        int fn;
                        fn = line.find("," , fp);
                        fn = fn - fp;
                        tmp_comp.feature = atoi(line.substr(fp, fn).c_str());

                        //----------------------------------------
                        //g_max_feature setting
                        //----------------------------------------

                        if(g_max_feature < tmp_comp.feature)
                        {
                           g_max_feature = tmp_comp.feature;
                        }
                        all_cmp.push_back(tmp_comp);
                        tmp_comp.mode == 0;
                        tmp_comp.LRF_number.clear();
                        tmp_comp.init_value.clear();
                        tmp_comp.feature == 0;
                    }


                    if (line_number == 5)break;
                }
            }
        }

        //-------------------------------------------------------------------------------------------------------


        //------------------------------------------------------------------------------------------------------
        //all_cnt to all_cmp
        //------------------------------------------------------------------------------------------------------
        int clsn = 0;
        for (clsn = 0 ; clsn < all_cmp.size() ; clsn++)
        {
            if (all_cmp[clsn].mode == 1 || all_cmp[clsn].mode == 2)
            {
                int f_righter = all_cnt[all_cmp[clsn].LRF_number[0] - 1].righter;
                int f_lefter  = all_cnt[all_cmp[clsn].LRF_number[0] - 1].lefter;
                int s_righter = all_cnt[all_cmp[clsn].LRF_number[1] - 1].righter;
                int s_lefter  = all_cnt[all_cmp[clsn].LRF_number[1] - 1].lefter;
                int f_righter_dose = all_cmp[clsn].init_value[0];
                int f_lefter_dose  = all_cmp[clsn].init_value[1];
                int s_righter_dose = all_cmp[clsn].init_value[2];
                int s_lefter_dose  = all_cmp[clsn].init_value[3];
                int calc_in;
                int calc_out;
                float calc_inout;
                float calc_outin;
                float calc_righter_inout;
                float calc_righter_outin;
                float calc_lefter_outin;
                float calc_lefter_inout;
                if (all_cmp[clsn].mode == 1)
                {
                    calc_in    =  f_lefter + s_righter - (f_lefter_dose  + s_righter_dose);
                    calc_out   =  f_righter + s_lefter - (f_righter_dose +  s_lefter_dose);
                    calc_righter_inout = (float)(f_righter - f_righter_dose)/(float)(s_righter - s_righter_dose);
                    calc_righter_outin = (float)(s_righter - s_righter_dose)/(float)(f_righter - f_righter_dose);
                    calc_lefter_outin  = (float)(f_lefter  - f_lefter_dose) /(float)(s_lefter - s_lefter_dose);
                    calc_lefter_inout  = (float)(s_lefter  - s_lefter_dose) /(float)(f_lefter  - f_lefter_dose);
                }

                else if (all_cmp[clsn].mode == 2)
                {
                    calc_in    =  f_righter + s_righter - (f_righter_dose +  s_righter_dose); 
                    calc_out   =  f_lefter + s_lefter - (f_lefter_dose + s_lefter_dose);
                    calc_righter_inout = (float)(f_righter - f_righter_dose)/(float)(s_lefter - s_lefter_dose);
                    calc_righter_outin = (float)(s_lefter  - s_lefter_dose) /(float)(f_righter - f_righter_dose);
                    calc_lefter_outin  = (float)(f_lefter  - f_lefter_dose) /(float)(s_righter - s_righter_dose);
                    calc_lefter_inout  = (float)(s_righter - s_righter_dose)/(float)(f_lefter  - f_lefter_dose);
                }
                //-------------------------------------
                //for remove judgement
                //-------------------------------------
                all_cmp[clsn].f_righter  = f_righter - f_righter_dose;
                all_cmp[clsn].f_lefter   = f_lefter  - f_lefter_dose;
                all_cmp[clsn].s_righter  = s_righter - s_righter_dose;
                all_cmp[clsn].s_lefter   = s_lefter  - s_lefter_dose;
                all_cmp[clsn].t_righter  = 100;
                all_cmp[clsn].t_lefter   = 100;
                all_cmp[clsn].fr_righter = 100;
                all_cmp[clsn].fr_lefter  = 100;

                calc_inout =  (float)calc_in/(float)calc_out;
                calc_outin =  (float)calc_out/(float)calc_in;
                all_cmp[clsn].inner = calc_in;
                all_cmp[clsn].outer = calc_out;
                all_cmp[clsn].calc_inout = calc_inout;
                all_cmp[clsn].calc_outin = calc_outin;
                all_cmp[clsn].calc_righter_inout = calc_righter_inout;
                all_cmp[clsn].calc_righter_outin = calc_righter_outin;
                all_cmp[clsn].calc_lefter_inout =  calc_lefter_inout;
                all_cmp[clsn].calc_lefter_outin =  calc_lefter_outin;
            }
            else if (all_cmp[clsn].mode == 3 || all_cmp[clsn].mode == 4 || all_cmp[clsn].mode == 5 || all_cmp[clsn].mode == 6)
            {
                int f_righter = all_cnt[all_cmp[clsn].LRF_number[0] - 1].righter;
                int f_lefter  = all_cnt[all_cmp[clsn].LRF_number[0] - 1].lefter;
                int s_righter = all_cnt[all_cmp[clsn].LRF_number[1] - 1].righter;
                int s_lefter  = all_cnt[all_cmp[clsn].LRF_number[1] - 1].lefter;
                int t_righter = all_cnt[all_cmp[clsn].LRF_number[2] - 1].righter;
                int t_lefter  = all_cnt[all_cmp[clsn].LRF_number[2] - 1].lefter;
                int f_righter_dose = all_cmp[clsn].init_value[0];
                int f_lefter_dose  = all_cmp[clsn].init_value[1];
                int s_righter_dose = all_cmp[clsn].init_value[2];
                int s_lefter_dose  = all_cmp[clsn].init_value[3];
                int t_righter_dose = all_cmp[clsn].init_value[4];
                int t_lefter_dose  = all_cmp[clsn].init_value[5];
                int calc_in;
                int calc_out;
                float calc_inout;
                float calc_outin;
                if (all_cmp[clsn].mode == 3)
                {
                    calc_in = f_lefter + s_lefter + t_righter - (f_lefter_dose + s_lefter_dose + t_righter_dose);
                    calc_out= f_righter + s_righter + t_lefter - (f_righter_dose + s_righter_dose + t_lefter_dose);
                }
                else if (all_cmp[clsn].mode == 4)
                {
                    calc_in = f_lefter + s_lefter + t_lefter - (f_lefter_dose + s_lefter_dose + t_lefter_dose);
                    calc_out= f_righter + s_righter + t_righter - (f_righter_dose + s_righter_dose + t_righter_dose);
                }
                else if (all_cmp[clsn].mode == 5)
                {
                    calc_in = f_lefter + s_righter + t_righter - (f_lefter_dose + s_righter_dose + t_righter_dose);
                    calc_out= f_righter + s_lefter + t_lefter - (f_righter_dose + s_lefter_dose + t_lefter_dose);
                }
                else if (all_cmp[clsn].mode == 6)
                {
                    calc_in = f_lefter + s_righter + t_lefter - (f_lefter_dose + s_righter_dose + t_lefter_dose);
                    calc_out= f_righter + s_lefter + t_righter - (f_righter_dose + s_lefter_dose + t_righter_dose);
                }
                //-------------------------------------
                //for remove judgement
                //-------------------------------------
                all_cmp[clsn].f_righter  = f_righter - f_righter_dose;
                all_cmp[clsn].f_lefter   = f_lefter  - f_lefter_dose;
                all_cmp[clsn].s_righter  = s_righter - s_righter_dose;
                all_cmp[clsn].s_lefter   = s_lefter  - s_lefter_dose;
                all_cmp[clsn].t_righter  = t_righter - t_righter_dose;
                all_cmp[clsn].t_lefter   = t_lefter  - t_lefter_dose;
                all_cmp[clsn].fr_righter = 100;
                all_cmp[clsn].fr_lefter  = 100;

                calc_inout = (float)calc_in/(float)calc_out;
                calc_outin = (float)calc_out/(float)calc_in;
                all_cmp[clsn].inner = calc_in;
                all_cmp[clsn].outer = calc_out;
                all_cmp[clsn].calc_inout = calc_inout;
                all_cmp[clsn].calc_outin = calc_outin;
                all_cmp[clsn].calc_righter_inout = 1000.0;
                all_cmp[clsn].calc_righter_outin = 1000.0;
                all_cmp[clsn].calc_lefter_inout  = 1000.0;
                all_cmp[clsn].calc_lefter_outin  = 1000.0;

            }
            else if (all_cmp[clsn].mode == 7  || all_cmp[clsn].mode == 8  || all_cmp[clsn].mode == 9  ||
                     all_cmp[clsn].mode == 10 || all_cmp[clsn].mode == 11 || all_cmp[clsn].mode == 12 ||
                     all_cmp[clsn].mode == 13 || all_cmp[clsn].mode == 14 )
            {
                int f_righter = all_cnt[all_cmp[clsn].LRF_number[0] - 1].righter;
                int f_lefter  = all_cnt[all_cmp[clsn].LRF_number[0] - 1].lefter;
                int s_righter = all_cnt[all_cmp[clsn].LRF_number[1] - 1].righter;
                int s_lefter  = all_cnt[all_cmp[clsn].LRF_number[1] - 1].lefter;
                int t_righter = all_cnt[all_cmp[clsn].LRF_number[2] - 1].righter;
                int t_lefter  = all_cnt[all_cmp[clsn].LRF_number[2] - 1].lefter;
                int tf_righter= all_cnt[all_cmp[clsn].LRF_number[3] - 1].righter;
                int tf_lefter = all_cnt[all_cmp[clsn].LRF_number[3] - 1].lefter;
                int f_righter_dose = all_cmp[clsn].init_value[0];
                int f_lefter_dose  = all_cmp[clsn].init_value[1];
                int s_righter_dose = all_cmp[clsn].init_value[2];
                int s_lefter_dose  = all_cmp[clsn].init_value[3];
                int t_righter_dose = all_cmp[clsn].init_value[4];
                int t_lefter_dose  = all_cmp[clsn].init_value[5];
                int tf_righter_dose= all_cmp[clsn].init_value[6];
                int tf_lefter_dose = all_cmp[clsn].init_value[7];
                int calc_in;
                int calc_out;
                float calc_inout;
                float calc_outin;
                if (all_cmp[clsn].mode == 7)
                {
                    calc_in = f_lefter + s_lefter + t_righter + tf_righter - (f_lefter_dose + s_lefter_dose + t_righter_dose + tf_righter_dose);
                    calc_out= f_righter + s_righter + t_lefter + tf_lefter - (f_righter_dose + s_righter_dose + t_lefter_dose + tf_lefter_dose);
                }
                else if (all_cmp[clsn].mode == 8)
                {
                    calc_in = f_lefter  + s_lefter + t_righter + tf_lefter  - (f_lefter_dose + s_lefter_dose + t_righter_dose + tf_lefter_dose);
                    calc_out= f_righter + s_righter + t_lefter + tf_righter - (f_righter_dose + s_righter_dose + t_lefter_dose + tf_righter_dose);
                }
                else if (all_cmp[clsn].mode == 9)
                {
                    calc_in = f_lefter + s_lefter + t_lefter + tf_righter - (f_lefter_dose + s_lefter_dose + t_lefter_dose + tf_righter_dose);
                    calc_out= f_righter + s_righter + t_righter + tf_lefter - (f_righter_dose + s_righter_dose + t_righter_dose + tf_lefter_dose);
                }
                else if (all_cmp[clsn].mode == 10)
                {
                    calc_in = f_lefter + s_lefter + t_lefter + tf_lefter - (f_lefter_dose + s_lefter_dose + t_lefter_dose + tf_lefter_dose);
                    calc_out= f_righter + s_righter + t_righter + tf_righter - (f_righter_dose + s_righter_dose + t_righter_dose + tf_righter_dose);
                }
                else if (all_cmp[clsn].mode == 11)
                {
                    calc_in = f_lefter + s_righter + t_righter + tf_righter - (f_lefter_dose + s_righter_dose + t_righter_dose + tf_righter_dose);
                    calc_out= f_righter + s_lefter + t_lefter + tf_lefter - (f_righter_dose + s_lefter_dose + t_lefter_dose + tf_lefter_dose);
                }
                else if (all_cmp[clsn].mode == 12)
                {
                    calc_in = f_lefter + s_righter + t_righter + tf_lefter - (f_lefter_dose + s_righter_dose + t_righter_dose + tf_lefter_dose);
                    calc_out= f_righter + s_lefter + t_lefter + tf_righter - (f_righter_dose + s_lefter_dose + t_lefter_dose + tf_righter_dose);
                }
                else if (all_cmp[clsn].mode == 13)
                {
                    calc_in = f_lefter + s_righter + t_lefter + tf_righter - (f_lefter_dose + s_righter_dose + t_lefter_dose + tf_righter_dose);
                    calc_out= f_righter + s_lefter + t_righter + tf_lefter - (f_righter_dose + s_lefter_dose + t_righter_dose + tf_lefter_dose);
                }
                else if (all_cmp[clsn].mode == 14)
                {
                    calc_in = f_lefter + s_righter + t_lefter + tf_lefter - (f_lefter_dose + s_righter_dose + t_lefter_dose + tf_lefter_dose);
                    calc_out= f_righter + s_lefter + t_righter + tf_righter - (f_righter_dose + s_lefter_dose + t_righter_dose + tf_righter_dose);
                }


                //-------------------------------------
                //for remove judgement
                //-------------------------------------
                all_cmp[clsn].f_righter  = f_righter  - f_righter_dose;
                all_cmp[clsn].f_lefter   = f_lefter   - f_lefter_dose;
                all_cmp[clsn].s_righter  = s_righter  - s_righter_dose;
                all_cmp[clsn].s_lefter   = s_lefter   - s_lefter_dose;
                all_cmp[clsn].t_righter  = t_righter  - t_righter_dose;
                all_cmp[clsn].t_lefter   = t_lefter   - t_lefter_dose;
                all_cmp[clsn].fr_righter = tf_righter - tf_righter_dose;
                all_cmp[clsn].fr_lefter  = tf_lefter  - tf_lefter_dose;

                calc_inout = (float)calc_in/(float)calc_out;
                calc_outin = (float)calc_out/(float)calc_in;
                all_cmp[clsn].inner = calc_in;
                all_cmp[clsn].outer = calc_out;
                all_cmp[clsn].calc_inout = calc_inout;
                all_cmp[clsn].calc_outin = calc_outin;
                all_cmp[clsn].calc_righter_inout = 1000.0;
                all_cmp[clsn].calc_righter_outin = 1000.0;
                all_cmp[clsn].calc_lefter_inout  = 1000.0;
                all_cmp[clsn].calc_lefter_outin  = 1000.0;
            }
        }

        //------------------------------
        //Copy all_cmp to all_cmp_sub
        //------------------------------
        std::copy(all_cmp.begin(),all_cmp.end(),std::back_inserter(all_cmp_sub));

        //-------------------------------------------------------------------------------------------------------

        //------------------------------
        //lrf_display
        //------------------------------
        lrf_display();
        under_lrf_display();

        //-------------------------------------------------------------------------------------------------------
        all_cmp.clear();
        all_cmp_sub.clear();
        r.sleep();

    }
}

//------------------------------------------------------------------------------------------------------
//dhisplay to terminal
//------------------------------------------------------------------------------------------------------

void  lrf_display()
{
    pthread_mutex_lock(&mutex_target);

    std::vector<int> new_put_lrf_number;
    std::vector<int> replace_lrf_number;
    std::vector<int> predominance;
    predominance.resize(6);
    int cnt = 0;
    char str[20];

    //----------------------------------------------------
    //predominance initialize
    //----------------------------------------------------
    for (int clsn = 0 ; clsn < all_cmp.size() ; clsn++)
    {
        for (int i = 0; i < all_cmp[clsn].LRF_number.size(); i++)
        {
                predominance[all_cmp[clsn].LRF_number[i] - 1] = 0;
        }
    }


    //----------------------------------------------------
    //predominance setting
    //----------------------------------------------------
    for (int clsn = 0 ; clsn < all_cmp.size() ; clsn++)
    {
        for (int i = 0; i < all_cmp[clsn].LRF_number.size(); i++)
        {
                predominance[all_cmp[clsn].LRF_number[i] - 1] = 1;
        }
    }


    //----------------------------------------------------

    //----------------------------------------------------
    //replaceable lrf number
    //----------------------------------------------------
    for (int clsn = 0; clsn < all_cnt.size(); clsn++)
    {
        if (predominance[clsn] == 0)
        {
            new_put_lrf_number.push_back(clsn + 1);

        }
        else if (clsn == predominance.size() - 1 )
        {
            new_put_lrf_number.push_back(10000);
        }
    }

    //----------------------------------------------------

    system("clear");

    //----------------------------------------------------
    //dhisplay setting and see lrf_status
    //----------------------------------------------------

    std::cout << "---Portable Status--- ";

    for(int clsn = 0 ; clsn < new_put_lrf_number.size(); clsn++)
    {
        std::cout << new_put_lrf_number[clsn] << " ";
    }
    std::cout << std::endl;
    std::cout << "Portable:";
    cnt = 17;
    for (int clsn = 0 ; clsn < all_cnt.size() ; clsn++)
    {
        std::cout << clsn + 1 << "[" << predominance[clsn] << "]" << "\r";
        sprintf(str, "\033[%dC", cnt);
        std::cout << str;
        cnt = cnt + 8;
    }

    std::cout << std::endl;
    std::cout << "righter :";
    cnt = 17;
    for (int clsn = 0 ; clsn < all_cnt.size() ; clsn++)
    {
        std::cout << all_cnt[clsn].righter << "\r";
        sprintf(str, "\033[%dC", cnt);
        std::cout << str;
        cnt = cnt + 8;
    }
    std::cout << std::endl;
    std::cout << "lefter  :";
    cnt = 17;
    for (int clsn = 0 ; clsn < all_cnt.size() ; clsn++)
    {
        std::cout << all_cnt[clsn].lefter << "\r";
        sprintf(str, "\033[%dC", cnt);
        std::cout << str;
        cnt = cnt + 8;
    }
    std::cout << std::endl;
    double ExeTime = ros::Time::now().toSec() - g_StartTime;
    std::cout << "---Mode Setting--- Time " << ExeTime << std::endl;

    //----------------------------------------------------

     //----------------------------------------------------
     //txt data dhisplay
     //----------------------------------------------------

    std::cout << "--------------------------------------------------------------------" << std::endl;

    for (int clsn = 0 ; clsn < all_cmp.size() ; clsn++)
    {
        std::cout << clsn << ". "  << "Portable_Number ";

        replace_lrf_number.resize(all_cmp[clsn].LRF_number.size());
        for (int i = 0; i < all_cmp[clsn].LRF_number.size(); i++)
        {
            std::cout <<  all_cmp[clsn].LRF_number[i];
            if (i !=  all_cmp[clsn].LRF_number.size() - 1)
            {
                std::cout << ",";
            }
                replace_lrf_number[i] = all_cmp[clsn].LRF_number[i];
        }
        std::cout << "\r" << "\033[35C"<< "measurement_space " << all_cmp[clsn].feature;
        std::cout << std::endl;
        std::cout << "\r" << "\033[3C"<< "inner " << all_cmp[clsn].inner << "\r" << "\033[35C"<< "calc_inout " << all_cmp[clsn].calc_inout;
        std::cout << std::endl;
        std::cout << "\r" << "\033[3C"<< "outer " << all_cmp[clsn].outer << "\r" << "\033[35C"<< "calc_outin " << all_cmp[clsn].calc_outin;
        std::cout << std::endl;
        std::cout << "\r" << "\033[3C"<< "EXE_TIME " <<ExeTime - all_cmp[clsn].time;
        std::cout << "\r" << "\033[23C" << "mode:" << all_cmp[clsn].mode;

        //----------------------------------------------------------------

        if (all_cmp[clsn].mode == 1 || all_cmp[clsn].mode == 2)
        {
            std::cout << "\r" << "\033[35C";
            std::cout << "righter_inout " << all_cmp[clsn].calc_righter_inout;
            std::cout << std::endl;
            std::cout << "\r" << "\033[35C";
            std::cout << "righter_outin " << all_cmp[clsn].calc_righter_outin;
            std::cout << std::endl;
            std::cout << "\r" << "\033[35C";
            std::cout << "lefter_inout " << all_cmp[clsn].calc_lefter_inout;
            std::cout << std::endl;
            std::cout << "\r" << "\033[35C";
            std::cout << "lefter_outin " << all_cmp[clsn].calc_lefter_outin;
            std::cout << std::endl;
            if((all_cmp[clsn].inner > RECONFIGURRABLE_START_THRESHHOLD || all_cmp[clsn].outer > RECONFIGURRABLE_START_THRESHHOLD) && ExeTime - all_cmp[clsn].time > TIMER_MINUTE * 60.0){
            Reconfigurable_threshhold(all_cmp[clsn].calc_inout,
                                      all_cmp[clsn].calc_inout,
                                      all_cmp[clsn].calc_righter_inout,
                                      all_cmp[clsn].calc_righter_outin,
                                      all_cmp[clsn].calc_lefter_inout,
                                      all_cmp[clsn].calc_lefter_outin,
                                      all_cmp[clsn].mode,
                                      all_cmp[clsn].feature,
                                      ExeTime
                                      );
              }else{
                std::cout << "\r" << "\033[35C" << "[wait a little]" << std::endl;
              }
        }else{
            if((all_cmp[clsn].inner > RECONFIGURRABLE_START_THRESHHOLD || all_cmp[clsn].outer > RECONFIGURRABLE_START_THRESHHOLD)){
            Reconfigurable_threshhold(all_cmp[clsn].calc_inout,
                                      all_cmp[clsn].calc_inout,
                                      PORTABLE_DISCRIMINATION_AVERAGE/100.0,
                                      PORTABLE_DISCRIMINATION_AVERAGE/100.0,
                                      PORTABLE_DISCRIMINATION_AVERAGE/100.0,
                                      PORTABLE_DISCRIMINATION_AVERAGE/100.0,
                                      all_cmp[clsn].mode,
                                      all_cmp[clsn].feature,
                                      ExeTime
                                      );
           }else{
                std::cout << "\r" << "\033[35C" << "[wait a little]" << std::endl;
              }
        }
      std::cout << "--------------------------------------------------------------------" << std::endl;
    }

    //----------------------------------------------------


    pthread_mutex_unlock(&mutex_target);
    predominance.clear();
}


//------------------------------------------------------------------------------------------------------
//dhisplay to terminal
//------------------------------------------------------------------------------------------------------
void  under_lrf_display()
{
 pthread_mutex_lock(&mutex_target);
 std::cout << "<<< Comment >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
  for(int ji=0;ji< comment_data.size();ji++)
 {
  std::cout << " " << comment_data[ji] << std::endl;
 }
 std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
 pthread_mutex_unlock(&mutex_target);
}







//------------------------------------------------------------------------------------------------------
//Reconfigurable threshhold setting
//------------------------------------------------------------------------------------------------------
void Reconfigurable_threshhold(float calc_inout,
                               float calc_outin,
                               float calc_righter_inout,
                               float calc_righter_outin,
                               float calc_lefter_inout,
                               float calc_lefter_outin,
                               int   mode,
                               int   f_feature,
                               double exe_time)
{
  float average_minus;
  float average_plus;
  float lower_bound;
  float upper_bound;

  //-------------------------------
  //FOR move portable variable
  //-------------------------------
  int mode_t = 0;
  int replace_a;
  int replace_b;
  int not_open_key = 0;
  std::vector<int> init_val_sub;
  char move_file[50];

  init_val_sub.resize(4);

  //P - 3 * σ
  average_minus = PORTABLE_DISCRIMINATION_AVERAGE - (FIXED_SIGMA_NUMBER * PORTABLE_DISCRIMINATION_VARIANCE);
  //P ＋3 * σ
  average_plus  = PORTABLE_DISCRIMINATION_AVERAGE + (FIXED_SIGMA_NUMBER * PORTABLE_DISCRIMINATION_VARIANCE);
  lower_bound =  average_minus/average_plus;
  upper_bound =  average_plus/average_minus;

  //------------------------------------------------
  //　閾値計算
  //------------------------------------------------
  //   P - 3 * σ           M        P + 3 * σ
  // -------------   <   -----  < -------------
  //   P + 3 * σ           N        P - 3 * σ

  if((lower_bound < calc_inout && calc_inout < upper_bound) || (lower_bound < calc_outin && calc_outin < upper_bound))
  {
   std::cout << "\r" << "\033[35C"<< "[no comment]" << std::endl;
  }else{
   std::cout << "\r" << "\033[35C"<< "[put a new portable]"  << std::endl;
  }


  //------------------------------------------------
  //層流型　乱流型判断
  //------------------------------------------------
  //   P - 3 * σ           R1        P + 3 * σ
  // -------------   <   -----  < -------------
  //   P + 3 * σ           R2        P - 3 * σ

  //   P - 3 * σ           L1        P + 3 * σ
  // -------------   <   -----  < -------------
  //   P + 3 * σ           L2        P - 3 * σ

  if(mode == 1 ||mode == 2){
  //------------------------------------------------
  //層流型判断と判断されたら
  //------------------------------------------------
  if(((lower_bound < calc_righter_inout && calc_righter_inout < upper_bound) ||
     (lower_bound < calc_righter_outin && calc_righter_outin < upper_bound)) &&
     ((lower_bound < calc_lefter_inout && calc_lefter_inout < upper_bound)   ||
     (lower_bound < calc_lefter_outin && calc_lefter_outin < upper_bound))
     )
  {
      //----------------------------------------------------
      //FIRST CHANGING  　all_cmp to FOR move portable variable
      //----------------------------------------------------
      std::vector<composition>::iterator it;
      for (it =all_cmp_sub.begin(); it < all_cmp_sub.end(); ++it)
      {
        if( it->feature== f_feature){
           //---------------------------------
           //Mode
           //---------------------------------
           

           //---------------------------------
           //First
           //---------------------------------
           replace_a = it->LRF_number[0];
           init_val_sub[0] = it->init_value[0];
           init_val_sub[1] = it->init_value[1];

           //---------------------------------
           //Second
           //---------------------------------
           replace_b = it->LRF_number[1];
           init_val_sub[2] = it->init_value[2];
           init_val_sub[3] = it->init_value[3];


           //---------------------------------
           //Erase
           //---------------------------------
           all_cmp_sub.erase(it);

           sprintf(move_file, "rm -r /home/kurt/catkin_ws/position/class%d.txt",f_feature);

           system(move_file);

           break;
          }
      }

      //----------------------------------------------------
      //SECOND CHANGING　直接的なポータブルの除去
      //----------------------------------------------------

          if(mode_t == 1)
          {
          //-------------------------------------
          //if mode_t == 1 and not change a mode
          //-------------------------------------
          std::vector<composition>::iterator it;
          for (it =all_cmp_sub.begin(); it < all_cmp_sub.end(); ++it)
           {
              for(int iu = 0; iu< it->LRF_number.size();iu++)
               {
                  if(it->LRF_number[iu] == replace_a)
                  {
                      //------------------------------------------------
                      //number -> replace_b   and change an init_value
                      //------------------------------------------------

                       int sub_feature;
                       sub_feature  = it->feature;
                       it->LRF_number[iu]   = replace_b;
                       it->init_value[(iu*2)  ] = it->init_value[2] - it->init_value[0] + it->init_value[(iu*2)  ];
                       it->init_value[(iu*2)+1] = it->init_value[3] - it->init_value[1] + it->init_value[(iu*2)+1];
                       g_max_feature++;
                       it->feature = g_max_feature;
                       not_open_key = 1;  //close

                       sprintf(move_file, "/home/kurt/catkin_ws/position/class%d.txt",it->feature);
                       std::ofstream make_a_txt_file(move_file);

                       make_a_txt_file << "mode:" << it->mode << "," << std::endl;
                       make_a_txt_file << "LRF_number:";
                       for(int ie=0;ie<it->LRF_number.size();ie++)
                       {
                         make_a_txt_file << it->LRF_number[ie] << ",";
                       }
                       make_a_txt_file << std::endl;
                       make_a_txt_file << "inival:";
                       for(int ie=0;ie<it->init_value.size();ie++)
                       {
                         make_a_txt_file << it->init_value[ie] << ",";
                       }
                       make_a_txt_file << std::endl;
                       make_a_txt_file << "start_time:" << exe_time << ","<< std::endl;
                       make_a_txt_file << "feature:"    << it->feature << "," << std::endl;

                       sprintf(move_file, "rm -r /home/kurt/catkin_ws/position/class%d.txt",sub_feature);
                       system(move_file);

                       //--------------
                       //alarm
                       //--------------
                       char move_file2[50];
                       sprintf(move_file2, "removed measurement_space %d",sub_feature);
                       comment_data.push_back(move_file2);
                   }
                }
             }

            if(not_open_key == 1)
            {
              //--------------
              //alarm
              //--------------
              char move_file3[50];
              sprintf(move_file3, "removed Portable%d",replace_a);
              comment_data.push_back(move_file3);
            }

                if(not_open_key == 0)
                {
                  //------------------------------------------------
                  //not open a door to replace A
                  //------------------------------------------------
                  for (it =all_cmp_sub.begin(); it < all_cmp_sub.end(); ++it)
                  {
                    for(int iu = 0; iu< it->LRF_number.size();iu++)
                     {
                        if(it->LRF_number[iu] == replace_b)
                        {
                            //------------------------------------------------
                            //number -> replace_b   and change an init_value
                            //------------------------------------------------

                             int sub_feature;
                             sub_feature  = it->feature;
                             it->LRF_number[iu]   = replace_a;
                             it->init_value[(iu*2)  ] = it->init_value[2] - it->init_value[0] + it->init_value[(iu*2)  ];
                             it->init_value[(iu*2)+1] = it->init_value[3] - it->init_value[1] + it->init_value[(iu*2)+1];
                             g_max_feature++;
                             it->feature = g_max_feature;
                             not_open_key = 1;  //close

                             sprintf(move_file, "/home/kurt/catkin_ws/position/class%d.txt",it->feature);
                             std::ofstream make_a_txt_file(move_file);

                             make_a_txt_file << "mode:" << it->mode  << ","<< std::endl;
                             make_a_txt_file << "LRF_number:";
                             for(int ie=0;ie<it->LRF_number.size();ie++)
                             {
                               make_a_txt_file << it->LRF_number[ie] << ",";
                             }
                             make_a_txt_file << std::endl;
                             make_a_txt_file << "inival:";
                             for(int ie=0;ie<it->init_value.size();ie++)
                             {
                               make_a_txt_file << it->init_value[ie] << ",";
                             }
                             make_a_txt_file << std::endl;
                             make_a_txt_file << "start_time:" << exe_time << ","<< std::endl;
                             make_a_txt_file << "feature:"    << it->feature << "," << std::endl;


                             sprintf(move_file, "rm -r /home/kurt/catkin_ws/position/class%d.txt",sub_feature);
                             system(move_file);

                             //--------------
                             //alarm
                             //--------------
                             char move_file2[50];
                             sprintf(move_file2, "removed measurement_space %d",sub_feature);
                             comment_data.push_back(move_file2);
                         }
                     }
                }
                  //--------------
                  //alarm
                  //--------------
                  char move_file3[50];
                  sprintf(move_file3, "removed Portable%d",replace_b);
                  comment_data.push_back(move_file3);
             }
          }
          else if(mode_t == 2)
         {
              //-------------------------------------
              //if mode_t == 2 and change a mode
              //-------------------------------------
              std::vector<composition>::iterator it;
              for (it =all_cmp_sub.begin(); it < all_cmp_sub.end(); ++it)
               {
                  for(int iu = 0; iu< it->LRF_number.size();iu++)
                   {
                      if(it->LRF_number[iu] == replace_a)
                      {
                          //------------------------------------------------
                          //number -> replace_b   and change a mode an init_value
                          //------------------------------------------------

                           int sub_feature;
                           sub_feature  = it->feature;
                           it->LRF_number[iu]   = replace_b;
                           it->init_value[(iu*2)  ] = it->init_value[2] - it->init_value[0] + it->init_value[(iu*2)  ];
                           it->init_value[(iu*2)+1] = it->init_value[3] - it->init_value[1] + it->init_value[(iu*2)+1];
                           g_max_feature++;
                           it->feature = g_max_feature;
                           not_open_key = 1;  //close

                           sprintf(move_file, "/home/kurt/catkin_ws/position/class%d.txt",it->feature);
                           std::ofstream make_a_txt_file(move_file);
                           make_a_txt_file << "mode:";

                           //------------------------------------------------
                           //mode change place
                           //------------------------------------------------
                           if(it->mode == 1)
                           {
                             make_a_txt_file << "2," << std::endl;
                             it->mode = 2;
                           }
                           else if(it->mode == 2)
                           {
                             make_a_txt_file << "1," << std::endl;
                             it->mode = 1;
                           }
                           else if(it->mode == 3)
                           {
                               if(iu ==0)
                                 {
                                   make_a_txt_file << "6," << std::endl;
                                   it->mode = 6;
                                 }
                               else if(iu ==1)
                                 {
                                   make_a_txt_file << "5," << std::endl;
                                   it->mode = 5;
                                 }
                               else if(iu ==2)
                                 {
                                   make_a_txt_file << "4," << std::endl;
                                   it->mode = 4;
                                 }
                           }
                           else if(it->mode == 4)
                           {
                               if(iu ==0)
                                 {
                                   make_a_txt_file << "5," << std::endl;
                                  it->mode = 5;
                                 }
                               else if(iu ==1)
                                 {
                                   make_a_txt_file << "6," << std::endl;
                                   it->mode = 6;
                                 }
                               else if(iu ==2)
                                 {
                                   make_a_txt_file << "3," << std::endl;
                                   it->mode = 3;
                                 }
                           }
                           else if(it->mode == 5)
                           {
                               if(iu ==0)
                                 {
                                   make_a_txt_file << "4," << std::endl;
                                   it->mode = 4;
                                 }
                               else if(iu ==1)
                                 {
                                   make_a_txt_file << "3," << std::endl;
                                   it->mode = 3;
                                 }
                               else if(iu ==2)
                                 {
                                   make_a_txt_file << "6," << std::endl;
                                   it->mode = 6;
                                 }
                           }
                           else if(it->mode == 6)
                           {
                               if(iu ==0)
                                 {
                                   make_a_txt_file << "3," << std::endl;
                                   it->mode = 3;
                                 }
                               else if(iu ==1)
                                 {
                                   make_a_txt_file << "4," << std::endl;
                                   it->mode = 4;
                                 }
                               else if(iu ==2)
                                 {
                                   make_a_txt_file << "5," << std::endl;
                                 it->mode = 5;
                                 }
                           }

                           make_a_txt_file << "LRF_number:";
                           for(int ie=0;ie<it->LRF_number.size();ie++)
                           {
                             make_a_txt_file << it->LRF_number[ie] << ",";
                           }
                           make_a_txt_file << std::endl;
                           make_a_txt_file << "inival:";
                           for(int ie=0;ie<it->init_value.size();ie++)
                           {
                             make_a_txt_file << it->init_value[ie] << ",";
                           }
                           make_a_txt_file << std::endl;
                           make_a_txt_file << "start_time:" << exe_time << ","<< std::endl;
                           make_a_txt_file << "feature:"    << it->feature << "," << std::endl;

                           sprintf(move_file, "rm -r /home/kurt/catkin_ws/position/class%d.txt",sub_feature);
                           system(move_file);
                           //--------------
                           //alarm
                           //--------------
                           char move_file2[50];
                           sprintf(move_file2, "removed measurement_space %d",sub_feature);
                           comment_data.push_back(move_file2);
                       }
                    }
                }
              if(not_open_key == 1)
              {
                //--------------
                //alarm
                //--------------
                char move_file3[50];
                sprintf(move_file3, "removed Portable%d",replace_a);
                comment_data.push_back(move_file3);
              }
              //------------------------------------------------
              //not open a door to replace A
              //------------------------------------------------
              if(not_open_key == 0)
                 {
                    for (it =all_cmp_sub.begin(); it < all_cmp_sub.end(); ++it)
                    {
                        for (it =all_cmp_sub.begin(); it < all_cmp_sub.end(); ++it)
                         {
                            for(int iu = 0; iu< it->LRF_number.size();iu++)
                             {
                                if(it->LRF_number[iu] == replace_b)
                                {
                                    //------------------------------------------------
                                    //number -> replace_a   and change a mode an init_value
                                    //------------------------------------------------

                                     int sub_feature;
                                     sub_feature  = it->feature;
                                     it->LRF_number[iu]   = replace_a;
                                     it->init_value[(iu*2)  ] = it->init_value[2] - it->init_value[0] + it->init_value[(iu*2)  ];
                                     it->init_value[(iu*2)+1] = it->init_value[3] - it->init_value[1] + it->init_value[(iu*2)+1];
                                     g_max_feature++;
                                     it->feature = g_max_feature;
                                     not_open_key = 1;  //close

                                     sprintf(move_file, "/home/kurt/catkin_ws/position/class%d.txt",it->feature);
                                     std::ofstream make_a_txt_file(move_file);
                                     make_a_txt_file << "mode:";

                                     //------------------------------------------------
                                     //mode change place
                                     //------------------------------------------------
                                     if(it->mode == 1)
                                     {
                                       make_a_txt_file << "2," << std::endl;
                                       it->mode = 2;
                                     }
                                     else if(it->mode == 2)
                                     {
                                       make_a_txt_file << "1," << std::endl;
                                       it->mode = 1;
                                     }
                                     else if(it->mode == 3)
                                     {
                                         if(iu ==0)
                                           {
                                             make_a_txt_file << "6," << std::endl;
                                             it->mode = 6;
                                           }
                                         else if(iu ==1)
                                           {
                                             make_a_txt_file << "5," << std::endl;
                                             it->mode = 5;
                                           }
                                         else if(iu ==2)
                                           {
                                             make_a_txt_file << "4," << std::endl;
                                             it->mode = 4;
                                           }
                                     }
                                     else if(it->mode == 4)
                                     {
                                         if(iu ==0)
                                           {
                                             make_a_txt_file << "5," << std::endl;
                                            it->mode = 5;
                                           }
                                         else if(iu ==1)
                                           {
                                             make_a_txt_file << "6," << std::endl;
                                             it->mode = 6;
                                           }
                                         else if(iu ==2)
                                           {
                                             make_a_txt_file << "3," << std::endl;
                                             it->mode = 3;
                                           }
                                     }
                                     else if(it->mode == 5)
                                     {
                                         if(iu ==0)
                                           {
                                             make_a_txt_file << "4," << std::endl;
                                             it->mode = 4;
                                           }
                                         else if(iu ==1)
                                           {
                                             make_a_txt_file << "3," << std::endl;
                                             it->mode = 3;
                                           }
                                         else if(iu ==2)
                                           {
                                             make_a_txt_file << "6," << std::endl;
                                             it->mode = 6;
                                           }
                                     }
                                     else if(it->mode == 6)
                                     {
                                         if(iu ==0)
                                           {
                                             make_a_txt_file << "3," << std::endl;
                                             it->mode = 3;
                                           }
                                         else if(iu ==1)
                                           {
                                             make_a_txt_file << "4," << std::endl;
                                             it->mode = 4;
                                           }
                                         else if(iu ==2)
                                           {
                                             make_a_txt_file << "5," << std::endl;
                                           it->mode = 5;
                                           }
                                     }

                                     make_a_txt_file << "LRF_number:";
                                     for(int ie=0;ie<it->LRF_number.size();ie++)
                                     {
                                       make_a_txt_file << it->LRF_number[ie] << ",";
                                     }
                                     make_a_txt_file << std::endl;
                                     make_a_txt_file << "inival:";
                                     for(int ie=0;ie<it->init_value.size();ie++)
                                     {
                                       make_a_txt_file << it->init_value[ie] << ",";
                                     }
                                     make_a_txt_file << std::endl;
                                     make_a_txt_file << "start_time:" << exe_time << ","<< std::endl;
                                     make_a_txt_file << "feature:"    << it->feature << "," << std::endl;

                                     sprintf(move_file, "rm -r /home/kurt/catkin_ws/position/class%d.txt",sub_feature);
                                     system(move_file);
                                     //--------------
                                     //alarm
                                     //--------------
                                     char move_file2[50];
                                     sprintf(move_file2, "removed measurement_space %d",sub_feature);
                                     comment_data.push_back(move_file2);
                                 }
                              }
                          }

                    }
                    //--------------
                    //alarm
                    //--------------
                    char move_file3[50];
                    sprintf(move_file3, "removed Portable%d",replace_b);
                    comment_data.push_back(move_file3);
                 }
               }

  }else{
      //------------------------------------------------
      //乱流型と判断されたら…見守る
      //------------------------------------------------
      std::cout << "\r" << "\033[35C"<< "[no comment]"  << std::endl;
  }
 }

  //------------------------------------------------
  //　dust in shoot 間接的なポータブルの除去
  //------------------------------------------------

   std::vector<composition>::iterator it;
    for (it =all_cmp_sub.begin(); it < all_cmp_sub.end(); ++it)
    {
      if((it->inner > RECONFIGURRABLE_START_THRESHHOLD || it->outer > RECONFIGURRABLE_START_THRESHHOLD) && exe_time - it->time > TIMER_MINUTE_SUB * 60.0)
       {
        if(it->mode == 3)
        {
            if(it->f_righter< REMOVE_THRESHHOLD && it->f_lefter < REMOVE_THRESHHOLD)
             {
                int sub_feature;
                sub_feature  = it->feature;
                g_max_feature++;
                it->feature = g_max_feature;

                //----------------------
                //make a file
                //----------------------
                sprintf(move_file, "/home/kurt/catkin_ws/position/class%d.txt",it->feature);
                std::ofstream make_a_txt_file(move_file);
                make_a_txt_file << "mode:1," << std::endl;
                make_a_txt_file << "LRF_number:" << it->LRF_number[1] << "," << it->LRF_number[2]<< "," << std::endl;
                make_a_txt_file << "inival:" << it->init_value[2] << "," << it->init_value[3] << "," << it->init_value[4] << "," << it->init_value[5] << "," << std::endl;
                make_a_txt_file << "start_time:" << exe_time << ","<< std::endl;
                make_a_txt_file << "feature:"    << it->feature << "," << std::endl;

                //----------------------
                //remove a file
                //----------------------
                sprintf(move_file, "rm -r /home/kurt/catkin_ws/position/class%d.txt",sub_feature);
                system(move_file);

                //--------------
                //kansetu_alarm
                //--------------
                char move_file2[50];
                sprintf(move_file2, "removed measurement_space %d",sub_feature);
                comment_data.push_back(move_file2);
                //--------------
                //kansetu_alarm
                //--------------
                char move_file3[50];
                sprintf(move_file3, "removed Portable%d",it->LRF_number[0]);
                comment_data.push_back(move_file3);
             }
             else if(it->s_righter < REMOVE_THRESHHOLD && it->s_lefter < REMOVE_THRESHHOLD)
             {
                int sub_feature;
                sub_feature  = it->feature;
                g_max_feature++;
                it->feature = g_max_feature;

                //----------------------
                //make a file
                //----------------------
                sprintf(move_file, "/home/kurt/catkin_ws/position/class%d.txt",it->feature);
                std::ofstream make_a_txt_file(move_file);
                make_a_txt_file << "mode:1," << std::endl;
                make_a_txt_file << "LRF_number:" << it->LRF_number[0] << "," << it->LRF_number[2]<< "," << std::endl;
                make_a_txt_file << "inival:" << it->init_value[0] << "," << it->init_value[1] << "," << it->init_value[4] << "," << it->init_value[5] << "," << std::endl;
                make_a_txt_file << "start_time:" << exe_time << ","<< std::endl;
                make_a_txt_file << "feature:"    << it->feature << "," << std::endl;

                //----------------------
                //remove a file
                //----------------------
                sprintf(move_file, "rm -r /home/kurt/catkin_ws/position/class%d.txt",sub_feature);
                system(move_file);

                //--------------
                //kansetu_alarm
                //--------------
                char move_file2[50];
                sprintf(move_file2, "removed measurement_space %d",sub_feature);
                comment_data.push_back(move_file2);
                //--------------
                //kansetu_alarm
                //--------------
                char move_file3[50];
                sprintf(move_file3, "removed Portable%d",it->LRF_number[1]);
                comment_data.push_back(move_file3);

             }
             else if(it->t_righter < REMOVE_THRESHHOLD && it->t_lefter < REMOVE_THRESHHOLD)
             {
                int sub_feature;
                sub_feature  = it->feature;
                g_max_feature++;
                it->feature = g_max_feature;

                //----------------------
                //make a file
                //----------------------
                sprintf(move_file, "/home/kurt/catkin_ws/position/class%d.txt",it->feature);
                std::ofstream make_a_txt_file(move_file);
                make_a_txt_file << "mode:2," << std::endl;
                make_a_txt_file << "LRF_number:" << it->LRF_number[0] << "," << it->LRF_number[1]<< "," << std::endl;
                make_a_txt_file << "inival:" << it->init_value[0] << "," << it->init_value[1] << "," << it->init_value[2] << "," << it->init_value[3] << "," << std::endl;
                make_a_txt_file << "start_time:" << exe_time << ","<< std::endl;
                make_a_txt_file << "feature:"    << it->feature << "," << std::endl;

                //----------------------
                //remove a file
                //----------------------
                sprintf(move_file, "rm -r /home/kurt/catkin_ws/position/class%d.txt",sub_feature);
                system(move_file);

                //--------------
                //kansetu_alarm
                //--------------
                char move_file2[50];
                sprintf(move_file2, "removed measurement_space %d",sub_feature);
                comment_data.push_back(move_file2);
                //--------------
                //kansetu_alarm
                //--------------
                char move_file3[50];
                sprintf(move_file3, "removed Portable%d",it->LRF_number[2]);
                comment_data.push_back(move_file3);

             }
        }
        else if(it->mode == 4)
        {
            if(it->f_righter< REMOVE_THRESHHOLD && it->f_lefter < REMOVE_THRESHHOLD)
             {
                int sub_feature;
                sub_feature  = it->feature;
                g_max_feature++;
                it->feature = g_max_feature;

                //----------------------
                //make a file
                //----------------------
                sprintf(move_file, "/home/kurt/catkin_ws/position/class%d.txt",it->feature);
                std::ofstream make_a_txt_file(move_file);
                make_a_txt_file << "mode:2," << std::endl;
                make_a_txt_file << "LRF_number:" << it->LRF_number[1] << "," << it->LRF_number[2]<< "," << std::endl;
                make_a_txt_file << "inival:" << it->init_value[2] << "," << it->init_value[3] << "," << it->init_value[4] << "," << it->init_value[5] << "," << std::endl;
                make_a_txt_file << "start_time:" << exe_time << ","<< std::endl;
                make_a_txt_file << "feature:"    << it->feature << "," << std::endl;

                //----------------------
                //remove a file
                //----------------------
                sprintf(move_file, "rm -r /home/kurt/catkin_ws/position/class%d.txt",sub_feature);
                system(move_file);

                //--------------
                //kansetu_alarm
                //--------------
                char move_file2[50];
                sprintf(move_file2, "removed measurement_space %d",sub_feature);
                comment_data.push_back(move_file2);
                //--------------
                //kansetu_alarm
                //--------------
                char move_file3[50];
                sprintf(move_file3, "removed Portable%d",it->LRF_number[0]);
                comment_data.push_back(move_file3);

             }
             else if(it->s_righter < REMOVE_THRESHHOLD && it->s_lefter < REMOVE_THRESHHOLD)
             {
                int sub_feature;
                sub_feature  = it->feature;
                g_max_feature++;
                it->feature = g_max_feature;

                //----------------------
                //make a file
                //----------------------
                sprintf(move_file, "/home/kurt/catkin_ws/position/class%d.txt",it->feature);
                std::ofstream make_a_txt_file(move_file);
                make_a_txt_file << "mode:2," << std::endl;
                make_a_txt_file << "LRF_number:" << it->LRF_number[0] << "," << it->LRF_number[2]<< "," << std::endl;
                make_a_txt_file << "inival:" << it->init_value[0] << "," << it->init_value[1] << "," << it->init_value[4] << "," << it->init_value[5] << "," << std::endl;
                make_a_txt_file << "start_time:" << exe_time << ","<< std::endl;
                make_a_txt_file << "feature:"    << it->feature << "," << std::endl;

                //----------------------
                //remove a file
                //----------------------
                sprintf(move_file, "rm -r /home/kurt/catkin_ws/position/class%d.txt",sub_feature);
                system(move_file);

                //--------------
                //kansetu_alarm
                //--------------
                char move_file2[50];
                sprintf(move_file2, "removed measurement_space %d",sub_feature);
                comment_data.push_back(move_file2);
                //--------------
                //kansetu_alarm
                //--------------
                char move_file3[50];
                sprintf(move_file3, "removed Portable%d",it->LRF_number[1]);
                comment_data.push_back(move_file3);


             }
             else if(it->t_righter < REMOVE_THRESHHOLD && it->t_lefter < REMOVE_THRESHHOLD)
             {
                int sub_feature;
                sub_feature  = it->feature;
                g_max_feature++;
                it->feature = g_max_feature;

                //----------------------
                //make a file
                //----------------------
                sprintf(move_file, "/home/kurt/catkin_ws/position/class%d.txt",it->feature);
                std::ofstream make_a_txt_file(move_file);
                make_a_txt_file << "mode:2," << std::endl;
                make_a_txt_file << "LRF_number:" << it->LRF_number[0] << "," << it->LRF_number[1]<< "," << std::endl;
                make_a_txt_file << "inival:" << it->init_value[0] << "," << it->init_value[1] << "," << it->init_value[2] << "," << it->init_value[3] << "," << std::endl;
                make_a_txt_file << "start_time:" << exe_time << ","<< std::endl;
                make_a_txt_file << "feature:"    << it->feature << "," << std::endl;

                //----------------------
                //remove a file
                //----------------------
                sprintf(move_file, "rm -r /home/kurt/catkin_ws/position/class%d.txt",sub_feature);
                system(move_file);

                //--------------
                //kansetu_alarm
                //--------------
                char move_file2[50];
                sprintf(move_file2, "removed measurement_space %d",sub_feature);
                comment_data.push_back(move_file2);
                //--------------
                //kansetu_alarm
                //--------------
                char move_file3[50];
                sprintf(move_file3, "removed Portable%d",it->LRF_number[2]);
                comment_data.push_back(move_file3);
             }
        }
        else if(it->mode == 5)
        {
            if(it->f_righter< REMOVE_THRESHHOLD && it->f_lefter < REMOVE_THRESHHOLD)
             {
                int sub_feature;
                sub_feature  = it->feature;
                g_max_feature++;
                it->feature = g_max_feature;

                //----------------------
                //make a file
                //----------------------
                sprintf(move_file, "/home/kurt/catkin_ws/position/class%d.txt",it->feature);
                std::ofstream make_a_txt_file(move_file);
                make_a_txt_file << "mode:2," << std::endl;
                make_a_txt_file << "LRF_number:" << it->LRF_number[1] << "," << it->LRF_number[2]<< "," << std::endl;
                make_a_txt_file << "inival:" << it->init_value[2] << "," << it->init_value[3] << "," << it->init_value[4] << "," << it->init_value[5] << "," << std::endl;
                make_a_txt_file << "start_time:" << exe_time << ","<< std::endl;
                make_a_txt_file << "feature:"    << it->feature << "," << std::endl;

                //----------------------
                //remove a file
                //----------------------
                sprintf(move_file, "rm -r /home/kurt/catkin_ws/position/class%d.txt",sub_feature);
                system(move_file);

                //--------------
                //kansetu_alarm
                //--------------
                char move_file2[50];
                sprintf(move_file2, "removed measurement_space %d",sub_feature);
                comment_data.push_back(move_file2);
                //--------------
                //kansetu_alarm
                //--------------
                char move_file3[50];
                sprintf(move_file3, "removed Portable%d",it->LRF_number[0]);
                comment_data.push_back(move_file3);

             }
             else if(it->s_righter < REMOVE_THRESHHOLD && it->s_lefter < REMOVE_THRESHHOLD)
             {
                int sub_feature;
                sub_feature  = it->feature;
                g_max_feature++;
                it->feature = g_max_feature;

                //----------------------
                //make a file
                //----------------------
                sprintf(move_file, "/home/kurt/catkin_ws/position/class%d.txt",it->feature);
                std::ofstream make_a_txt_file(move_file);
                make_a_txt_file << "mode:1," << std::endl;
                make_a_txt_file << "LRF_number:" << it->LRF_number[0] << "," << it->LRF_number[2]<< "," << std::endl;
                make_a_txt_file << "inival:" << it->init_value[0] << "," << it->init_value[1] << "," << it->init_value[4] << "," << it->init_value[5] << "," << std::endl;
                make_a_txt_file << "start_time:" << exe_time << ","<< std::endl;
                make_a_txt_file << "feature:"    << it->feature << "," << std::endl;

                //----------------------
                //remove a file
                //----------------------
                sprintf(move_file, "rm -r /home/kurt/catkin_ws/position/class%d.txt",sub_feature);
                system(move_file);

                //--------------
                //kansetu_alarm
                //--------------
                char move_file2[50];
                sprintf(move_file2, "removed measurement_space %d",sub_feature);
                comment_data.push_back(move_file2);
                //--------------
                //kansetu_alarm
                //--------------
                char move_file3[50];
                sprintf(move_file3, "removed Portable%d",it->LRF_number[1]);
                comment_data.push_back(move_file3);


             }
             else if(it->t_righter < REMOVE_THRESHHOLD && it->t_lefter < REMOVE_THRESHHOLD)
             {
                int sub_feature;
                sub_feature  = it->feature;
                g_max_feature++;
                it->feature = g_max_feature;

                //----------------------
                //make a file
                //----------------------
                sprintf(move_file, "/home/kurt/catkin_ws/position/class%d.txt",it->feature);
                std::ofstream make_a_txt_file(move_file);
                make_a_txt_file << "mode:1," << std::endl;
                make_a_txt_file << "LRF_number:" << it->LRF_number[0] << "," << it->LRF_number[1]<< "," << std::endl;
                make_a_txt_file << "inival:" << it->init_value[0] << "," << it->init_value[1] << "," << it->init_value[2] << "," << it->init_value[3] << "," << std::endl;
                make_a_txt_file << "start_time:" << exe_time << ","<< std::endl;
                make_a_txt_file << "feature:"    << it->feature << "," << std::endl;

                //----------------------
                //remove a file
                //----------------------
                sprintf(move_file, "rm -r /home/kurt/catkin_ws/position/class%d.txt",sub_feature);
                system(move_file);

                //--------------
                //kansetu_alarm
                //--------------
                char move_file2[50];
                sprintf(move_file2, "removed measurement_space %d",sub_feature);
                comment_data.push_back(move_file2);
                //--------------
                //kansetu_alarm
                //--------------
                char move_file3[50];
                sprintf(move_file3, "removed Portable%d",it->LRF_number[2]);
                comment_data.push_back(move_file3);
             }
        }
        else if(it->mode == 6)
        {
            if(it->f_righter< REMOVE_THRESHHOLD && it->f_lefter < REMOVE_THRESHHOLD)
             {
                int sub_feature;
                sub_feature  = it->feature;
                g_max_feature++;
                it->feature = g_max_feature;

                //----------------------
                //make a file
                //----------------------
                sprintf(move_file, "/home/kurt/catkin_ws/position/class%d.txt",it->feature);
                std::ofstream make_a_txt_file(move_file);
                make_a_txt_file << "mode:1," << std::endl;
                make_a_txt_file << "LRF_number:" << it->LRF_number[1] << "," << it->LRF_number[2]<< "," << std::endl;
                make_a_txt_file << "inival:" << it->init_value[2] << "," << it->init_value[3] << "," << it->init_value[4] << "," << it->init_value[5] << "," << std::endl;
                make_a_txt_file << "start_time:" << exe_time << ","<< std::endl;
                make_a_txt_file << "feature:"    << it->feature << "," << std::endl;

                //----------------------
                //remove a file
                //----------------------
                sprintf(move_file, "rm -r /home/kurt/catkin_ws/position/class%d.txt",sub_feature);
                system(move_file);

                //--------------
                //kansetu_alarm
                //--------------
                char move_file2[50];
                sprintf(move_file2, "removed measurement_space %d",sub_feature);
                comment_data.push_back(move_file2);
                //--------------
                //kansetu_alarm
                //--------------
                char move_file3[50];
                sprintf(move_file3, "removed Portable%d",it->LRF_number[0]);
                comment_data.push_back(move_file3);

             }
             else if(it->s_righter < REMOVE_THRESHHOLD && it->s_lefter < REMOVE_THRESHHOLD)
             {
                int sub_feature;
                sub_feature  = it->feature;
                g_max_feature++;
                it->feature = g_max_feature;

                //----------------------
                //make a file
                //----------------------
                sprintf(move_file, "/home/kurt/catkin_ws/position/class%d.txt",it->feature);
                std::ofstream make_a_txt_file(move_file);
                make_a_txt_file << "mode:2," << std::endl;
                make_a_txt_file << "LRF_number:" << it->LRF_number[0] << "," << it->LRF_number[2]<< "," << std::endl;
                make_a_txt_file << "inival:" << it->init_value[0] << "," << it->init_value[1] << "," << it->init_value[4] << "," << it->init_value[5] << "," << std::endl;
                make_a_txt_file << "start_time:" << exe_time << ","<< std::endl;
                make_a_txt_file << "feature:"    << it->feature << "," << std::endl;

                //----------------------
                //remove a file
                //----------------------
                sprintf(move_file, "rm -r /home/kurt/catkin_ws/position/class%d.txt",sub_feature);
                system(move_file);

                //--------------
                //kansetu_alarm
                //--------------
                char move_file2[50];
                sprintf(move_file2, "removed measurement_space %d",sub_feature);
                comment_data.push_back(move_file2);
                //--------------
                //kansetu_alarm
                //--------------
                char move_file3[50];
                sprintf(move_file3, "removed Portable%d",it->LRF_number[1]);
                comment_data.push_back(move_file3);


             }
             else if(it->t_righter < REMOVE_THRESHHOLD && it->t_lefter < REMOVE_THRESHHOLD)
             {
                int sub_feature;
                sub_feature  = it->feature;
                g_max_feature++;
                it->feature = g_max_feature;

                //----------------------
                //make a file
                //----------------------
                sprintf(move_file, "/home/kurt/catkin_ws/position/class%d.txt",it->feature);
                std::ofstream make_a_txt_file(move_file);
                make_a_txt_file << "mode:1," << std::endl;
                make_a_txt_file << "LRF_number:" << it->LRF_number[0] << "," << it->LRF_number[1]<< "," << std::endl;
                make_a_txt_file << "inival:" << it->init_value[0] << "," << it->init_value[1] << "," << it->init_value[2] << "," << it->init_value[3] << "," << std::endl;
                make_a_txt_file << "start_time:" << exe_time << ","<< std::endl;
                make_a_txt_file << "feature:"    << it->feature << "," << std::endl;

                //----------------------
                //remove a file
                //----------------------
                sprintf(move_file, "rm -r /home/kurt/catkin_ws/position/class%d.txt",sub_feature);
                system(move_file);

                //--------------
                //kansetu_alarm
                //--------------
                char move_file2[50];
                sprintf(move_file2, "removed measurement_space %d",sub_feature);
                comment_data.push_back(move_file2);
                //--------------
                //kansetu_alarm
                //--------------
                char move_file3[50];
                sprintf(move_file3, "removed Portable%d",it->LRF_number[2]);
                comment_data.push_back(move_file3);
             }
        }
    }
  }
}


void PsenCallback0(const tms_msg_ss::pot_tracking_psens::ConstPtr &psens)
{
    int ID;
    int flag;
    float output_x;
    float output_y;
    pthread_mutex_lock(&mutex_target);
    for (int i = 0; i < psens->pot_tracking_psen.size(); i++)
    {
        char str[20];
        ID = psens->pot_tracking_psen[i].id;
        flag = psens->pot_tracking_psen[i].flag;
        output_x = psens->pot_tracking_psen[i].x;
        output_y = psens->pot_tracking_psen[i].y;
        all_cnt[0].righter = psens->pot_tracking_psen[i].righter + righter_adjustment[0];
        all_cnt[0].lefter = psens->pot_tracking_psen[i].lefter   + lefter_adjustment[0];
        sprintf(str, "psen/psen_txt0/ID%d.txt", ID);
        std::ofstream strw;
        strw.open(str, std::ofstream::out | std::ofstream::app);
        strw << output_x << " " << output_y  << " " << flag << std::endl;
    }

    pthread_mutex_unlock(&mutex_target);
}

void PsenCallback1(const tms_msg_ss::pot_tracking_psens::ConstPtr &psens)
{
    int ID;
    float output_x;
    float output_y;
    pthread_mutex_lock(&mutex_target);
    for (int i = 0; i < psens->pot_tracking_psen.size(); i++)
    {
        char str[20];
        ID = psens->pot_tracking_psen[i].id;
        output_x = psens->pot_tracking_psen[i].x;
        output_y = psens->pot_tracking_psen[i].y;
        all_cnt[1].righter = psens->pot_tracking_psen[i].righter  + righter_adjustment[1];
        all_cnt[1].lefter  = psens->pot_tracking_psen[i].lefter   + lefter_adjustment[1];
        sprintf(str, "psen/psen_txt1/ID%d.txt", ID);
        std::ofstream strw;
        strw.open(str, std::ofstream::out | std::ofstream::app);
        strw << output_x << " " << output_y << std::endl;
    }

    pthread_mutex_unlock(&mutex_target);
}

void PsenCallback2(const tms_msg_ss::pot_tracking_psens::ConstPtr &psens)
{
    int ID;
    float output_x;
    float output_y;
    pthread_mutex_lock(&mutex_target);
    for (int i = 0; i < psens->pot_tracking_psen.size(); i++)
    {
        char str[20];
        ID = psens->pot_tracking_psen[i].id;
        output_x = psens->pot_tracking_psen[i].x;
        output_y = psens->pot_tracking_psen[i].y;
        all_cnt[2].righter = psens->pot_tracking_psen[i].righter + righter_adjustment[2];
        all_cnt[2].lefter  = psens->pot_tracking_psen[i].lefter  + lefter_adjustment[2];
        sprintf(str, "psen/psen_txt2/ID%d.txt", ID);
        std::ofstream strw;
        strw.open(str, std::ofstream::out | std::ofstream::app);
        strw << output_x << " " << output_y << std::endl;
    }

    pthread_mutex_unlock(&mutex_target);
}

void PsenCallback3(const tms_msg_ss::pot_tracking_psens::ConstPtr &psens)
{
    int ID;
    float output_x;
    float output_y;
    pthread_mutex_lock(&mutex_target);
    for (int i = 0; i < psens->pot_tracking_psen.size(); i++)
    {
        char str[20];
        ID = psens->pot_tracking_psen[i].id;
        output_x = psens->pot_tracking_psen[i].x;
        output_y = psens->pot_tracking_psen[i].y;
        all_cnt[3].righter = psens->pot_tracking_psen[i].righter  + righter_adjustment[3];
        all_cnt[3].lefter  = psens->pot_tracking_psen[i].lefter   + lefter_adjustment[3];
        sprintf(str, "psen/psen_txt3/ID%d.txt", ID);
        std::ofstream strw;
        strw.open(str, std::ofstream::out | std::ofstream::app);
        strw << output_x << " " << output_y << std::endl;
    }

    pthread_mutex_unlock(&mutex_target);
}

void PsenCallback4(const tms_msg_ss::pot_tracking_psens::ConstPtr &psens)
{
    int ID;
    float output_x;
    float output_y;
    pthread_mutex_lock(&mutex_target);
    for (int i = 0; i < psens->pot_tracking_psen.size(); i++)
    {
        char str[20];
        ID = psens->pot_tracking_psen[i].id;
        output_x = psens->pot_tracking_psen[i].x;
        output_y = psens->pot_tracking_psen[i].y;
        all_cnt[4].righter = psens->pot_tracking_psen[i].righter + righter_adjustment[4];
        all_cnt[4].lefter  = psens->pot_tracking_psen[i].lefter + lefter_adjustment[4];
        sprintf(str, "psen/psen_txt4/ID%d.txt", ID);
        std::ofstream strw;
        strw.open(str, std::ofstream::out | std::ofstream::app);
        strw << output_x << " " << output_y << std::endl;
    }

    pthread_mutex_unlock(&mutex_target);
}

void PsenCallback5(const tms_msg_ss::pot_tracking_psens::ConstPtr &psens)
{
    int ID;
    float output_x;
    float output_y;
    pthread_mutex_lock(&mutex_target);
    for (int i = 0; i < psens->pot_tracking_psen.size(); i++)
    {
        char str[20];
        ID = psens->pot_tracking_psen[i].id;
        output_x = psens->pot_tracking_psen[i].x;
        output_y = psens->pot_tracking_psen[i].y;
        all_cnt[5].righter = psens->pot_tracking_psen[i].righter + righter_adjustment[5];
        all_cnt[5].lefter  = psens->pot_tracking_psen[i].lefter + lefter_adjustment[5];
        sprintf(str, "psen/psen_txt/ID%d.txt", ID);
        std::ofstream strw;
        strw.open(str, std::ofstream::out | std::ofstream::app);
        strw << output_x << " " << output_y << std::endl;
    }

    pthread_mutex_unlock(&mutex_target);
}

int main( int argc, char **argv )
{ 
    std::cout << "---pot_psen_manager start---" << std::endl;
    all_cnt.resize(6);
    righter_adjustment.resize(6);
    lefter_adjustment.resize(6);
    g_max_feature = 0;
    pthread_t thread_p;
    pthread_t thread_v;
    ros::MultiThreadedSpinner spinner(4);

    //test--------------------------
    all_cnt[0].righter = 20;
    all_cnt[0].lefter  = 20;
    all_cnt[1].righter = 0;
    all_cnt[1].lefter  = 0;
    all_cnt[2].righter = 0;
    all_cnt[2].lefter  = 0;
    all_cnt[3].righter = 0;
    all_cnt[3].lefter  = 0;
    all_cnt[4].righter = 0;
    all_cnt[4].lefter  = 0;
    all_cnt[5].righter = 20;
    all_cnt[5].lefter  = 20;
    //------------------------------

    for(int i=0;i<6;i++)
    {
     righter_adjustment[i] = 0;
     lefter_adjustment[i]  = 0;
    }

    ros::init(argc, argv, "pot_psen_manager");
    ros::NodeHandle n;
    ros::Subscriber sub0 = n.subscribe("/pot_urg0/tracking_psen0" , 1000, PsenCallback0);
    ros::Subscriber sub1 = n.subscribe("/pot_urg1/tracking_psen1" , 1000, PsenCallback1);
    ros::Subscriber sub2 = n.subscribe("/pot_urg2/tracking_psen2" , 1000, PsenCallback2);
    ros::Subscriber sub3 = n.subscribe("/pot_urg3/tracking_psen3" , 1000, PsenCallback3);
    ros::Subscriber sub4 = n.subscribe("/pot_urg4/tracking_psen4" , 1000, PsenCallback4);
    ros::Subscriber sub5 = n.subscribe("/pot_urg5/tracking_psen5" , 1000, PsenCallback5);

    if ( pthread_create( &thread_p, NULL, Management, NULL) )
    {
        printf("error creating thread.");
        abort();
    }

    if ( pthread_create( &thread_v, NULL, getcss, NULL) )
    {
        printf("error creating thread.");
        abort();
    }
    g_StartTime = ros::Time::now().toSec();
    spinner.spin(); // spin() will not return until the node has been shutdown

    ros::waitForShutdown();

    if ( pthread_join( thread_v, NULL) )
    {
        printf("error joining thread.");
        abort();
    }

    if ( pthread_join( thread_p, NULL) )
    {
        printf("error joining thread.");
        abort();
    }

    return 0;
}
