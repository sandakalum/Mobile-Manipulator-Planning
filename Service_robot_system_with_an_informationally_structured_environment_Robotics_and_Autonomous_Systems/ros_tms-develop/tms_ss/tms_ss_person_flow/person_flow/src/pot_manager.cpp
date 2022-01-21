//----------------------------------------------------------
// @file   : pot_psen_manager.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.4 (since 2014.05.02)
// @date   : 2016.11.18
//----------------------------------------------------------

#include <ros/ros.h>
#include <pthread.h>
#include <math.h>
#include <std_msgs/String.h>
#include <tms_msg_ss/pot_tracking_psen.h>
#include <tms_msg_ss/pot_tracking_psens.h>
#include <tms_msg_ss/pot_manager.h>

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

typedef struct
{
	int righter;
	int lefter;
} counter;

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
} counter_hex;

std::vector<int> endid;
std::vector<int>::iterator v;
std::vector<counter> all_cnt;
//std::vector<counter_hex> all_hexcnt;
counter_hex hex;

void  modecounter(double sata, double eata, double sdis, double edis)
{
	if (sdis < 1000.0 || 5000.0 < sdis) std::cout << "--------ERROR--------" << std::endl;
	else if (edis < 1000.0 || 5000.0 < edis) std::cout << "--------ERROR--------" << std::endl;
	else if ( PI / 6.0  < sata && sata < PI / 2.0)
	{
		if ( PI / 6.0  < eata && eata < PI / 2.0)
		{
			hex.ftof ++;
		}
		else if ( -PI / 6.0 < eata && eata < PI / 6.0)
		{
			hex.ftos ++;
		}
		else if ( -PI / 2.0 < eata && eata < -PI / 6.0)
		{
			hex.ftot ++;
		}
	}
	else if ( -PI / 6.0 < sata && sata < PI / 6.0)
	{
		if ( PI / 6.0  < eata && eata < PI / 2.0)
		{
			hex.stof ++;
		}
		else if ( -PI / 6.0 < eata && eata < PI / 6.0)
		{
			hex.stos ++;
		}
		else if ( -PI / 2.0 < eata && eata < -PI / 6.0)
		{
			hex.stot ++;
		}

	}
	else if ( -PI / 2.0 < sata && sata < -PI / 6.0)
	{
		if ( PI / 6.0  < eata && eata < PI / 2.0)
		{
			hex.ttof ++;
		}
		else if ( -PI / 6.0 < eata && eata < PI / 6.0)
		{
			hex.ttos ++;
		}
		else if ( -PI / 2.0 < eata && eata < -PI / 6.0)
		{
			hex.ttot ++;
		}

	}
}

void *Management( void *ptr )
{
	ros::Publisher *pub = (ros::Publisher *)ptr;
	tms_msg_ss::pot_manager mngr;
	ros::Rate r(10);
	while (ros::ok())
	{
		pthread_mutex_lock(&mutex_target);
		if (endid.size() > 0)
		{
			v = endid.begin();
			//std::cout << "ID " << *v << "endseize " <<  endid.size() << std::endl;
			std::cout << "ID " << *v << std::endl;
			char str[20];
			double sx, ex;
			double sy, ey;
			double sata, eata;
			double sdis, edis;
			sprintf(str, "txt_box/ID%d.txt", *v);
			endid.erase (v);
			std::ifstream ifs(str);
			std::string s_line, tmp_line, e_line;

			getline(ifs, s_line);
			int fs, fe;
			fs = s_line.find(",");
			fs = fs - 1;
			tmp_line = s_line.substr(1, fs);
			sx = atof(tmp_line.c_str());
			fe = s_line.find(")");
			fe = fe - fs - 2;
			tmp_line = s_line.substr(fs + 2, fe);
			sy = atof(tmp_line.c_str());
			sata = (double)atan(sy / sx);
			sdis = sqrt(pow(sx - 0, 2) + pow(sy - 0, 2));
			//std::cout << "s_x " << sx << "s_y " << sy << std::endl;
			std::cout << "sata " << sata << "  sdis " << sdis << std::endl;

			while (getline(ifs, s_line))
			{
				tmp_line = s_line ;
			}
			e_line = tmp_line;

			fs = e_line.find(",");
			fs = fs - 1;
			tmp_line = e_line.substr(1, fs);
			ex = atof(tmp_line.c_str());
			fe = e_line.find(")");
			fe = fe - fs - 2;
			tmp_line = e_line.substr(fs + 2, fe);
			ey = atof(tmp_line.c_str());
			eata = (double)atan(ey / ex);
			edis = sqrt(pow(ex - 0, 2) + pow(ey - 0, 2));
			//std::cout << "s_x " << ex << "s_y " << ey << std::endl;
			std::cout << "eata " << eata << "  edis " << edis << std::endl;
			modecounter(sata, eata, sdis, edis);
		}
		// std::cout << "/////////////////////////////////////////" << std::endl;
		// std::cout << "hex.ftof " << hex.ftof << std::endl;
		// std::cout << "hex.ftos " << hex.ftos << std::endl;
		// std::cout << "hex.ftot " << hex.ftot << std::endl;
		// std::cout << "hex.stof " << hex.stof << std::endl;
		// std::cout << "hex.stos " << hex.stos << std::endl;
		// std::cout << "hex.stot " << hex.stot << std::endl;
		// std::cout << "hex.ttof " << hex.ttof << std::endl;
		// std::cout << "hex.ttos " << hex.ttos << std::endl;
		// std::cout << "hex.ttot " << hex.ttot << std::endl;
		// std::cout << "/////////////////////////////////////////" << std::endl;
		mngr.ftof = hex.ftof;
		mngr.ftos = hex.ftos;
		mngr.ftot = hex.ftot;
		mngr.stof = hex.stof;
		mngr.stos = hex.stos;
		mngr.stot = hex.stot;
		mngr.ttof = hex.ttof;
		mngr.ttos = hex.ttos;
		mngr.ttot = hex.ttot;
		mngr.righter = all_cnt[0].righter;
		mngr.lefter  = all_cnt[0].lefter;
		  
		pub->publish(mngr);
		pthread_mutex_unlock(&mutex_target);
		r.sleep();
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
		output_x =   psens->pot_tracking_psen[i].y;
		output_y = -(psens->pot_tracking_psen[i].x);
		all_cnt[0].righter = psens->pot_tracking_psen[i].righter;
		all_cnt[0].lefter  = psens->pot_tracking_psen[i].lefter;
		sprintf(str, "txt_box/ID%d.txt", ID);
		std::ofstream strw;
		strw.open(str, std::ofstream::out | std::ofstream::app);
		strw << "(" << output_x << "," << output_y << ")" << " " << flag << std::endl;
		if (flag == 0)
		{
			//std::cout << "hogehoge " << std::endl;
			endid.push_back(ID);
		}
	}
	pthread_mutex_unlock(&mutex_target);
}

int main( int argc, char **argv )
{
	std::cout << "---pot_manager start---" << std::endl;
	all_cnt.resize(10);
	pthread_t thread_m;
	ros::MultiThreadedSpinner spinner(4);

	ros::init(argc, argv, "pot_manager");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("tracking_psen0" , 1000, PsenCallback0);
	ros::Publisher  pub = n.advertise<tms_msg_ss::pot_manager>("pot_manager_s0", 10);

	if ( pthread_create( &thread_m, NULL, Management,  (void *)&pub))
	{
		printf("error creating thread.");
		abort();
	}

	spinner.spin();

	ros::waitForShutdown();

	if ( pthread_join( thread_m, NULL) )
	{
		printf("error joining thread.");
		abort();
	}

	return 0;
}


