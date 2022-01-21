//----------------------------------------------------------
// @file   : pot_ctrl.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.4 (since 2014.05.02)
// @date   : 2016.06.09
//----------------------------------------------------------

#include <ros/ros.h>


#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <time.h>
#include <vector>
#include <math.h>

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
#include "std_msgs/String.h"

#include <sstream>

#include <sensor_msgs/LaserScan.h>

//using namespace std;
//using namespace boost;


typedef struct
{
    double x;
    double y;
} pos_cv;

typedef struct
{
    int id;
    pos_cv vector;
} pos_inf;

namespace fs = boost::filesystem;
vector<long> v_list;
int i;

void LaserSensingCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    ROS_INFO("start pot_urg_pot");

    ROS_INFO("end   pot_urg_pot");
}

int main( int argc, char **argv )
{



#if 0

    ros::Subscriber sub;
    ros::MultiThreadedSpinner spinner(4);
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    sub   = nh.subscribe("/pot_urg1/scan", 1000, LaserSensingCallback);
    spinner.spin();
    ros::waitForShutdown();
#endif
 

#if 0
ros::init(argc, argv, "talker");
ros::NodeHandle n;
ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
int count = 0;
ros::Rate loop_rate(10);
while (ros::ok())
{
   std_msgs::String msg;
   std::stringstream ss;
   ss << "hello world " << count;
   msg.data = ss.str();
   ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
 }

#endif


#if 0
 std::vector<float> test;
 test.push_back(1.0);
 test.push_back(5.0);
 test.push_back(2.0);
 test.push_back(3.0);
 test.push_back(4.0);
 test.push_back(7.0);
 test.push_back(9.0);
 test.push_back(8.0);
 test.push_back(6.0);

  std::cout << "before insert ";  
  for(int i=0; i< test.size(); i++)
  {
    std::cout << test[i] << " ";
  }
  std::cout << std::endl; 
  
  for(int i=0; i< test.size(); i++)
  {
  sort(test.begin(),test.end());
  }

  std::cout << "after insert ";  
  for(int i=0; i< test.size(); i++)
  {
    std::cout << test[i] << " ";
  }
  std::cout << std::endl;


#endif


#if 0
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {

        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

#endif

#if 0

    std::ifstream ifs("txt_box/e.txt");
    std::string str;
    std::string ckr;
    std::getline(ifs, str);
    std::cout << str << " ";
    std::cout << "aaaa";
    while (std::getline(ifs, str))
    {
        ckr = str;
        std::cout << str << " ";
        //int i;
        //i = strlen(str.c_str());
        //std::cout << "size " << i << std::endl ;
        //std::cout << " " << str.substr(5, i-5) << std::endl;
        //int pos = str.find("sf");

        //            while (pos != string::npos )
        //                std::cout << str << " " << pos << std::endl;
    }
    std::cout << ckr << std::endl;


#endif

#if 0
    int i = -5;
    printf("%d no zettaiti %d", i, abs(i));
#endif

#if 0
    while (1)
    {
        const fs::path path("/home/kurt/catkin_ws/position");
        BOOST_FOREACH(const fs::path & p, std::make_pair(fs::directory_iterator(path), fs::directory_iterator()))
        {
            if (!fs::is_directory(p) && boost::algorithm::iends_with(p.string(), ".txt"))
            {
                //std::cout << p.filename().string() << std::endl;
                //std::string str = p.filename().string();
                std::ifstream ifs(p.c_str());
                std::string line;
                std::string line_8;
                //char stt[7];
                int i;
                i = 0;
                while (getline(ifs, line))
                {
                    i++;
                    // ファイルの中身を一行づつ表示
                    //std::cout << " size " << line.size() << std::endl;
                    line_8 = line.substr(0, 5);
                    //std::cout << line_8 << std::endl;
                    if (line_8 == "mode:")
                    {
                        std::cout << line.substr(5, 5) << std::endl;
                        //stt = line.substr(8).c_str();
                    }
                }
            }
        }
    }

#endif


#if 0

    std::ifstream ifs("plot.txt");
    std::string str;
    while (std::getline(ifs, str))
    {
        unsigned int pos = str.find("\n");
        while (pos = string::npos )
            std::cout << line << std::endl;
    }

    std::ofstream ofs;
    ofs.open("txt/tot.txt", std::ofstream::out | std::ofstream::app);
    ofs << "ham" << std::endl;
    std::ifstream ifs("123tt.txt");
    if (!ifs)
    {
        std::cout << "ERRRRRRRRRRRROR" << std::endl;
    }
    else
    {
        std::ofstream ofs;
        ofs.open("123tt.txt", std::ofstream::out | std::ofstream::app);
        ofs << "ham" << std::endl;

    }
#endif

#if 0
    std::vector<pos_inf> posinf;
    std::vector<pos_inf>::iterator v;
    pos_inf tmp_inf;

    for (int i = 0; i < 10 ; i++)
    {
        tmp_inf.id = i;
        tmp_inf.vector.x = (double)i;
        tmp_inf.vector.y = (double)i;
        posinf.push_back(tmp_inf);
    }

    for (v = posinf.begin(); v != posinf.end(); ++v)
    {
        std::cout << " id " << v->id;
        std::cout << " x " <<  v->vector.x;
        std::cout << " y " <<  v->vector.y;
        std::cout << std::endl;
    }

#endif


#if 0

    v_list.push_back(100);
    v_list.push_back(200);
    v_list.push_back(300);
    vector< long >::iterator cIter;
    vector< long >::iterator v;
    cIter = find( v_list.begin(), v_list.end() , 200 );
    if ( cIter != v_list.end() )
    {
        std::cout << "200は存在してます" << std::endl;
    }
    v_list.erase (cIter);
    std::cout << "erase 200" << std::endl;

    cIter = find( cIter, v_list.end() , 300 );
    v_list.erase (cIter);
    std::cout << "erase 300" << std::endl;

    for (cIter = v_list.begin(); cIter != v_list.end(); ++cIter)
    {
        std::cout << *cIter ;
    }
    std::cout << std::endl;
#endif

#if 0
    pos_t titi;
    std::cout << "maxconnect" << MAX_CONNECT  << std::endl;
    titi.x = 9;
    titi.y = 8;
    std::cout << titi.y  << titi.x << std::endl;
#endif
#if 0
    std::vector<double> before_x;
    std::vector<double> before_y;
    std::vector<double> after_x;
    std::vector<double> after_y;
    std::vector<double>::iterator it_x;
    std::vector<double>::iterator it_y;
    int size_x = 0;

    before_x.push_back(0.0);
    before_y.push_back(1.0);

    before_x.push_back(1.0);
    before_y.push_back(1.0);

    before_x.push_back(1.0);
    before_y.push_back(0.0);

    ///////////////////////////////////////////////////////////

    after_x.push_back(0.0);
    after_y.push_back(2.0);

    after_x.push_back(1.0);
    after_y.push_back(2.0);

    after_x.push_back(2.0);
    after_y.push_back(1.0);

    after_x.push_back(2.0);
    after_y.push_back(0.0);

    after_x.push_back(100.0);
    after_y.push_back(100.0);

    ///////////////////////////////////////////////////////////

    it_x = before_x.end();
    it_y = before_y.end();

    for (int i = 0; i < after_x.size() ; i++)
    {
        for (it_x = before_x.end(), it_y = before_y.end(); it_x != before_x.begin() - 1 ; --it_x, --it_y)
        {
            double r = sqrt(pow(*it_x - after_x[i], 2) + pow(*it_y - after_y[i], 2));
            if (r < 1.2)
            {
                std::cout << "r " << r << "x y" << *it_x << *it_y << "x! y!" << after_x[i] << after_y[i] << std::endl;
                //std::cout << "r " << r << std::endl;
                before_x.insert(it_x + 1, after_x[i]);
                before_y.insert(it_y + 1, after_y[i]);
                break;
            }
            if (it_x == before_x.begin())
            {
                before_x.insert(it_x, after_x[i]);
                before_y.insert(it_y, after_y[i]);
            }
            //std::cout << "r " << r << "x y"<< before_x[i] << before_y[i] << "x! y!" << after_x[j] << after_y[j] << std::endl;
        }
        it_x = before_x.begin();
        it_y = before_y.begin();
    }

    std::cout << "before_x.size() " << before_x.size() << std::endl;
    for (int i = 0; i < before_x.size() ; i++)
    {
        std::cout << "x y" << before_x[i] << before_y[i] << std::endl;
    }
#endif

#if 0
    //
    std::ifstream ifs("test.txt");
    std::string line;
    while (std::getline(ifs, line))
    {
        //
        std::cout << line << std::endl;
    }

    //
    std::ofstream ofs("test.txt");

    //
    ofs << "ham" << std::endl;
    ofs << "egg" << std::endl;
    ofs << "spam" << std::endl;
    ofs << "spam1" << std::endl;
#endif

#if 0
    // ファイル入力ストリームの初期化
    std::ifstream ifs("test.txt");
    std::string line;
    while (std::getline(ifs, line))
    {
        // ファイルの中身を一行づつ表示
        std::cout << line << std::endl;
    }

    // ファイル出力ストリームの初期化
    std::ofstream ofs("test.txt");

    // ファイルに1行ずつ書き込み
    ofs << "ham" << std::endl;
    ofs << "egg" << std::endl;
    ofs << "spam" << std::endl;
    ofs << "spam1" << std::endl;
#endif
    return 0;
}
