//----------------------------------------------------------
// @file   : pot_psen_generalizar.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.4 (since 2014.05.02)
// @date   : 2016.11.18
//----------------------------------------------------------

#include <ros/ros.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <time.h>
#include <vector>
#include <math.h>
#include <pthread.h>
#include <tms_ss_pot/Particle_filter.h>
#include <tms_ss_pot/define.h>
#include <tms_ss_pot/Target.h>
#include <tms_ss_pot/Laser.h>
#include <tms_ss_pot/Multiple_particle_filter.h>
#include <iostream>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <tms_msg_ss/pot_manager.h>

#define lrf_number_s  0
#define lrf_number_e  11
#define Correspondence_s 0
#define Correspondence_e 21
#define ini_value_s  0
#define ini_value_e  7
#define feature_s 0
#define feature_e 8

pthread_mutex_t mutex_laser  = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_target = PTHREAD_MUTEX_INITIALIZER;
namespace fs = boost::filesystem;

typedef struct
{
    std::vector<int>  LRF_number;
    std::vector<int>  Correspondence_points;
    std::vector<int>  init_value;
    int feature;
} composition;

typedef struct
{
    int ftof;
    int ftos;
    int ftot;
    int stof;
    int stos;
    int stot;
    int ttof;
    int ttos;
    int ttot;
    int righter;
    int lefter;
} counter_mngr;

std::vector<composition> all_cmp;
composition tmp_comp;
std::vector<counter_mngr> c_mngr;//0,1,2,3,4,5,6,7,8,9

void *Generalization( void *ptr )
{
    ros::Rate r(30);
    while (ros::ok())
    {
        const fs::path path("/home/kurt/catkin_ws/position_kai");
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
                    if (line.substr(lrf_number_s, lrf_number_e) == "LRF_number:")
                    {
                        int fp = lrf_number_e;
                        int fn;
                        for (int i = 0; i < 2; i++)
                        {
                            fn = line.find("," , fp);
                            fn = fn - fp;
                            tmp_comp.LRF_number.push_back(atoi(line.substr(fp, fn).c_str()));
                            std::cout << atoi(line.substr(fp, fn).c_str()) << " ";
                            fp = fp + fn + 1;
                        }
                        std::cout << std::endl;
                    }
                    else if (line.substr(Correspondence_s, Correspondence_e) == "Correspondence_point:")
                    {
                        int fp = Correspondence_e;
                        int fn;
                        for (int i = 0; i < 2; i++)
                        {
                            fn = line.find("," , fp);
                            fn = fn - fp;
                            tmp_comp.Correspondence_points.push_back(atoi(line.substr(fp, fn).c_str()));
                            std::cout << atoi(line.substr(fp, fn).c_str()) << " ";
                            fp = fp + fn + 1;
                        }
                        std::cout << std::endl;
                    }
                    else if (line.substr(ini_value_s, ini_value_e) == "inival:")
                    {
                        int fp = ini_value_e;
                        int fn;
                        for (int i = 0; i < 18; i++)
                        {
                            fn = line.find("," , fp);
                            fn = fn - fp;
                            tmp_comp.init_value.push_back(atoi(line.substr(fp, fn).c_str()));
                            std::cout << atoi(line.substr(fp, fn).c_str()) << " ";
                            fp = fp + fn + 1;
                        }
                        std::cout << std::endl;

                    }
                    else if (line.substr(feature_s, feature_e) == "feature:")
                    {
                        int fp = feature_e;
                        int fn;
                        fn = line.find("," , fp);
                        fn = fn - fp;
                        tmp_comp.feature = atoi(line.substr(fp, fn).c_str());
                        all_cmp.push_back(tmp_comp);
                        tmp_comp.LRF_number.clear();
                        tmp_comp.Correspondence_points.clear();
                        tmp_comp.init_value.clear();
                        tmp_comp.feature == 0;
                    }

                }

            }
        }
        int clsn = 0;
        for (clsn = 0 ; clsn < all_cmp.size() ; clsn++)
        {
            int f_ftof = c_mngr[all_cmp[clsn].LRF_number[0] - 1].ftof;
            int f_ftos = c_mngr[all_cmp[clsn].LRF_number[0] - 1].ftos;
            int f_ftot = c_mngr[all_cmp[clsn].LRF_number[0] - 1].ftot;
            int f_stof = c_mngr[all_cmp[clsn].LRF_number[0] - 1].stof;
            int f_stos = c_mngr[all_cmp[clsn].LRF_number[0] - 1].stos;
            int f_stot = c_mngr[all_cmp[clsn].LRF_number[0] - 1].stot;
            int f_ttof = c_mngr[all_cmp[clsn].LRF_number[0] - 1].ttof;
            int f_ttos = c_mngr[all_cmp[clsn].LRF_number[0] - 1].ttos;
            int f_ttot = c_mngr[all_cmp[clsn].LRF_number[0] - 1].ttot;

            int s_ftof = c_mngr[all_cmp[clsn].LRF_number[1] - 1].ftof;
            int s_ftos = c_mngr[all_cmp[clsn].LRF_number[1] - 1].ftos;
            int s_ftot = c_mngr[all_cmp[clsn].LRF_number[1] - 1].ftot;
            int s_stof = c_mngr[all_cmp[clsn].LRF_number[1] - 1].stof;
            int s_stos = c_mngr[all_cmp[clsn].LRF_number[1] - 1].stos;
            int s_stot = c_mngr[all_cmp[clsn].LRF_number[1] - 1].stot;
            int s_ttof = c_mngr[all_cmp[clsn].LRF_number[1] - 1].ttof;
            int s_ttos = c_mngr[all_cmp[clsn].LRF_number[1] - 1].ttos;
            int s_ttot = c_mngr[all_cmp[clsn].LRF_number[1] - 1].ttot;

            int f_ftof_dose = all_cmp[clsn].init_value[0];
            int f_ftos_dose = all_cmp[clsn].init_value[1];
            int f_ftot_dose = all_cmp[clsn].init_value[2];
            int f_stof_dose = all_cmp[clsn].init_value[3];
            int f_stos_dose = all_cmp[clsn].init_value[4];
            int f_stot_dose = all_cmp[clsn].init_value[5];
            int f_ttof_dose = all_cmp[clsn].init_value[6];
            int f_ttos_dose = all_cmp[clsn].init_value[7];
            int f_ttot_dose = all_cmp[clsn].init_value[8];

            int s_ftof_dose = all_cmp[clsn].init_value[9];
            int s_ftos_dose = all_cmp[clsn].init_value[10];
            int s_ftot_dose = all_cmp[clsn].init_value[11];
            int s_stof_dose = all_cmp[clsn].init_value[12];
            int s_stos_dose = all_cmp[clsn].init_value[13];
            int s_stot_dose = all_cmp[clsn].init_value[14];
            int s_ttof_dose = all_cmp[clsn].init_value[15];
            int s_ttos_dose = all_cmp[clsn].init_value[16];
            int s_ttot_dose = all_cmp[clsn].init_value[17];

            if (all_cmp[clsn].Correspondence_points[0] == 1 && all_cmp[clsn].Correspondence_points[1] == 1)
            {
                int f_in = f_ftof + f_stof + f_ttof;
                int f_in_dose = f_ftof_dose + f_stof_dose + f_ttof_dose;
                int s_in = s_ftof + s_stof + s_ttof;
                int s_in_dose = s_ftof_dose + s_stof_dose + s_ttof_dose;
                int f_out = f_ftof + f_ftos + f_ftot;
                int f_out_dose = f_ftof_dose + f_ftos_dose + f_ftot_dose;
                int s_out = s_ftof + s_ftos + s_ftot;
                int s_out_dose = s_ftof_dose + s_ftos_dose + s_ftot_dose;
                int calc;
                calc = f_in + s_in - (f_in_dose + s_in_dose) - (f_out + s_out - (f_out_dose + s_out_dose));
                if (abs(calc) > 100 )
                {
                    std::cout << "put on the area " << all_cmp[clsn].feature << std::endl;
                }
            }
            else if (all_cmp[clsn].Correspondence_points[0] == 1 && all_cmp[clsn].Correspondence_points[1] == 2)
            {
                int f_in = f_ftof + f_stof + f_ttof;
                int f_in_dose = f_ftof_dose + f_stof_dose + f_ttof_dose;
                int s_in = s_ftos + s_stos + s_ttos;
                int s_in_dose = s_ftos_dose + s_stos_dose + s_ttos_dose;
                int f_out = f_ftof + f_ftos + f_ftot;
                int f_out_dose = f_ftof_dose + f_ftos_dose + f_ftot_dose;
                int s_out = s_stof + s_stos + s_stot;
                int s_out_dose = s_stof_dose + s_stos_dose + s_stot_dose;
                int calc;
                calc = f_in + s_in - (f_in_dose + s_in_dose) - (f_out + s_out - (f_out_dose + s_out_dose));
                if (abs(calc) > 100 )
                {
                    std::cout << "put on the area " << all_cmp[clsn].feature << std::endl;
                }
            }
            else if (all_cmp[clsn].Correspondence_points[0] == 1 && all_cmp[clsn].Correspondence_points[1] == 3)
            {
                int f_in = f_ftof + f_stof + f_ttof;
                int f_in_dose = f_ftof_dose + f_stof_dose + f_ttof_dose;
                int s_in = s_ftot + s_stot + s_ttot;
                int s_in_dose = s_ftot_dose + s_stot_dose + s_ttot_dose;
                int f_out = f_ftof + f_ftos + f_ftot;
                int f_out_dose = f_ftof_dose + f_ftos_dose + f_ftot_dose;
                int s_out = s_ttof + s_ttos + s_ttot;
                int s_out_dose = s_ttof_dose + s_ttos_dose + s_ttot_dose;
                int calc;
                calc = f_in + s_in - (f_in_dose + s_in_dose) - (f_out + s_out - (f_out_dose + s_out_dose));
                if (abs(calc) > 100 )
                {
                    std::cout << "put on the area " << all_cmp[clsn].feature << std::endl;
                }
            }
            else if (all_cmp[clsn].Correspondence_points[0] == 2 && all_cmp[clsn].Correspondence_points[1] == 1)
            {
                int f_in = f_ftos + f_stos + f_ttos;
                int f_in_dose = f_ftos_dose + f_stos_dose + f_ttos_dose;
                int s_in = s_ftof + s_stof + s_ttof;
                int s_in_dose = s_ftof_dose + s_stof_dose + s_ttof_dose;
                int f_out = f_stof + f_stos + f_stot;
                int f_out_dose = f_stof_dose + f_stos_dose + f_stot_dose;
                int s_out = s_ftof + s_ftos + s_ftot;
                int s_out_dose = s_ftof_dose + s_ftos_dose + s_ftot_dose;
                int calc;
                calc = f_in + s_in - (f_in_dose + s_in_dose) - (f_out + s_out - (f_out_dose + s_out_dose));
                if (abs(calc) > 100 )
                {
                    std::cout << "put on the area " << all_cmp[clsn].feature << std::endl;
                }

            }
            else if (all_cmp[clsn].Correspondence_points[0] == 2 && all_cmp[clsn].Correspondence_points[1] == 2)
            {
                int f_in = f_ftos + f_stos + f_ttos;
                int f_in_dose = f_ftos_dose + f_stos_dose + f_ttos_dose;
                int s_in = s_ftos + s_stos + s_ttos;
                int s_in_dose = s_ftos_dose + s_stos_dose + s_ttos_dose;
                int f_out = f_stof + f_stos + f_stot;
                int f_out_dose = f_stof_dose + f_stos_dose + f_stot_dose;
                int s_out = s_stof + s_stos + s_stot;
                int s_out_dose = s_stof_dose + s_stos_dose + s_stot_dose;
                int calc;
                calc = f_in + s_in - (f_in_dose + s_in_dose) - (f_out + s_out - (f_out_dose + s_out_dose));
                if (abs(calc) > 100 )
                {
                    std::cout << "put on the area " << all_cmp[clsn].feature << std::endl;
                }

            }
            else if (all_cmp[clsn].Correspondence_points[0] == 2 && all_cmp[clsn].Correspondence_points[1] == 3)
            {
                int f_in = f_ftos + f_stos + f_ttos;
                int f_in_dose = f_ftos_dose + f_stos_dose + f_ttos_dose;
                int s_in = s_ftot + s_stot + s_ttot;
                int s_in_dose = s_ftot_dose + s_stot_dose + s_ttot_dose;
                int f_out = f_stof + f_stos + f_stot;
                int f_out_dose = f_stof_dose + f_stos_dose + f_stot_dose;
                int s_out = s_ttof + s_ttos + s_ttot;
                int s_out_dose = s_ttof_dose + s_ttos_dose + s_ttot_dose;
                int calc;
                calc = f_in + s_in - (f_in_dose + s_in_dose) - (f_out + s_out - (f_out_dose + s_out_dose));
                if (abs(calc) > 100 )
                {
                    std::cout << "put on the area " << all_cmp[clsn].feature << std::endl;
                }

            }
            else if (all_cmp[clsn].Correspondence_points[0] == 3 && all_cmp[clsn].Correspondence_points[1] == 1)
            {
                int f_in = f_ftot + f_stot + f_ttot;
                int f_in_dose = f_ftot_dose + f_stot_dose + f_ttot_dose;
                int s_in = s_ftof + s_stof + s_ttof;
                int s_in_dose = s_ftof_dose + s_stof_dose + s_ttof_dose;
                int f_out = f_ttof + f_ttos + f_ttot;
                int f_out_dose = f_ttof_dose + f_ttos_dose + f_ttot_dose;
                int s_out = s_ftof + s_ftos + s_ftot;
                int s_out_dose = s_ftof_dose + s_ftos_dose + s_ftot_dose;
                int calc;
                calc = f_in + s_in - (f_in_dose + s_in_dose) - (f_out + s_out - (f_out_dose + s_out_dose));
                if (abs(calc) > 100 )
                {
                    std::cout << "put on the area " << all_cmp[clsn].feature << std::endl;
                }
            }
            else if (all_cmp[clsn].Correspondence_points[0] == 3 && all_cmp[clsn].Correspondence_points[1] == 2)
            {
                int f_in = f_ftot + f_stot + f_ttot;
                int f_in_dose = f_ftot_dose + f_stot_dose + f_ttot_dose;
                int s_in = s_ftos + s_stos + s_ttos;
                int s_in_dose = s_ftos_dose + s_stos_dose + s_ttos_dose;
                int f_out = f_ttof + f_ttos + f_ttot;
                int f_out_dose = f_ttof_dose + f_ttos_dose + f_ttot_dose;
                int s_out = s_stof + s_stos + s_stot;
                int s_out_dose = s_stof_dose + s_stos_dose + s_stot_dose;
                int calc;
                calc = f_in + s_in - (f_in_dose + s_in_dose) - (f_out + s_out - (f_out_dose + s_out_dose));
                if (abs(calc) > 100 )
                {
                    std::cout << "put on the area " << all_cmp[clsn].feature << std::endl;
                }
            }
            else if (all_cmp[clsn].Correspondence_points[0] == 3 && all_cmp[clsn].Correspondence_points[1] == 3)
            {
                int f_in = f_ftot + f_stot + f_ttot;
                int f_in_dose = f_ftot_dose + f_stot_dose + f_ttot_dose;
                int s_in = s_ftot + s_stot + s_ttot;
                int s_in_dose = s_ftot_dose + s_stot_dose + s_ttot_dose;
                int f_out = f_ttof + f_ttos + f_ttot;
                int f_out_dose = f_ttof_dose + f_ttos_dose + f_ttot_dose;
                int s_out = s_ttof + s_ttos + s_ttot;
                int s_out_dose = s_ttof_dose + s_ttos_dose + s_ttot_dose;
                int calc;
                calc = f_in + s_in - (f_in_dose + s_in_dose) - (f_out + s_out - (f_out_dose + s_out_dose));
                if (abs(calc) > 100 )
                {
                    std::cout << "put on the area " << all_cmp[clsn].feature << std::endl;
                }
            }
        }
        all_cmp.clear();
        r.sleep();
    }

}



void GneralizarCallback0(const tms_msg_ss::pot_manager::ConstPtr &mngr)
{
    pthread_mutex_lock(&mutex_target);
    c_mngr[0].ftof = mngr->ftof;
    c_mngr[0].ftos = mngr->ftos;
    c_mngr[0].ftot = mngr->ftot;
    c_mngr[0].stof = mngr->stof;
    c_mngr[0].stos = mngr->stos;
    c_mngr[0].stot = mngr->stot;
    c_mngr[0].ttof = mngr->ttof;
    c_mngr[0].ttos = mngr->ttos;
    c_mngr[0].ttot = mngr->ttot;
    c_mngr[0].righter = mngr->righter;
    c_mngr[0].lefter = mngr->lefter;
    pthread_mutex_unlock(&mutex_target);
}

int main( int argc, char **argv )
{
    std::cout << "---pot_psen_generalizar start---" << std::endl;
    c_mngr.resize(10);
    pthread_t thread_p;
    ros::MultiThreadedSpinner spinner(4);

    ros::init(argc, argv, "pot_psen_generalizar");
    ros::NodeHandle n;
    ros::Subscriber sub0 = n.subscribe("pot_manager_s0" , 1000, GneralizarCallback0);
    //ros::Subscriber sub1 = n.subscribe("pot_manager_s1" , 1000, GneralizarCallback1);
    //ros::Subscriber sub2 = n.subscribe("pot_manager_s2" , 1000, GneralizarCallback2);
    //ros::Subscriber sub3 = n.subscribe("pot_manager_s3" , 1000, GneralizarCallback3);
    //ros::Subscriber sub4 = n.subscribe("pot_manager_s4" , 1000, GneralizarCallback4);
    //ros::Subscriber sub5 = n.subscribe("pot_manager_s5" , 1000, GneralizarCallback5);
    //ros::Subscriber sub6 = n.subscribe("pot_manager_s6" , 1000, GneralizarCallback6);
    //ros::Subscriber sub7 = n.subscribe("pot_manager_s7" , 1000, GneralizarCallback7);
    //ros::Subscriber sub8 = n.subscribe("pot_manager_s8" , 1000, GneralizarCallback8);
    //ros::Subscriber sub9 = n.subscribe("pot_manager_s9" , 1000, GneralizarCallback9);

    if ( pthread_create( &thread_p, NULL, Generalization, NULL) )
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

    return 0;
}