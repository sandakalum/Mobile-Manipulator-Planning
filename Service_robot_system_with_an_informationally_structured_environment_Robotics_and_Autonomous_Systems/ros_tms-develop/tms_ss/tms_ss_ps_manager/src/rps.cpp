//----------------------------------------------------------
// @file   : rps.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.4 (since 2014.05.02)
// @date   : 2016.06.09
//----------------------------------------------------------
#include <ros/ros.h>
#include <tms_msg_db/TmsdbGetData.h>
#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_db/Tmsdb.h>
#include <tms_msg_rp/rp_arrow.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <iostream>
#include <ctime>
#include <time.h>

int main(int argc, char** argv)
{
#if 1
  //---------------------------------------------------------------------------------------
  // argv test
  //---------------------------------------------------------------------------------------
  printf("\n");
  switch (*argv[1])
  {
    case '1':
      printf("Good morning ");
      break;
    case '2':
      printf("Good evening ");
      break;
    case '3':
      printf("Good afternoon");
      break;
    default:
      printf("Good afternoon");
      break;
  }

  return 0;

#endif

#if 0
//---------------------------------------------------------------------------------------
//tms_db_reader  + rp_arrow
//---------------------------------------------------------------------------------------
  int i = 0;
  int por1=1,por2=0,por3=0; 
  ros::init(argc, argv, "rps_manager");
  ros::NodeHandle nh;
  ros::ServiceClient portal1 = nh.serviceClient<tms_msg_db::TmsdbGetData>("/tms_db_reader/dbreader");
  ros::ServiceClient portal2 = nh.serviceClient<tms_msg_db::TmsdbGetData>("/tms_db_reader/dbreader");
  ros::ServiceClient portal3 = nh.serviceClient<tms_msg_db::TmsdbGetData>("/tms_db_reader/dbreader");

  ros::ServiceClient portal1_arrow = nh.serviceClient<tms_msg_rp::rp_arrow>("rp_arrow");
  ros::ServiceClient portal2_arrow = nh.serviceClient<tms_msg_rp::rp_arrow>("rp_arrow");
  ros::ServiceClient portal3_arrow = nh.serviceClient<tms_msg_rp::rp_arrow>("rp_arrow");

  tms_msg_db::TmsdbGetData portal_mg1;  
  tms_msg_db::TmsdbGetData portal_mg2; 
  tms_msg_db::TmsdbGetData portal_mg3; 

  tms_msg_rp::rp_arrow portal_arrow_mg1;
  tms_msg_rp::rp_arrow portal_arrow_mg2;
  tms_msg_rp::rp_arrow portal_arrow_mg3;

  portal_mg1.request.tmsdb.id = 3011;
  portal_mg2.request.tmsdb.id = 3012;
  portal_mg3.request.tmsdb.id = 3013;

  portal_arrow_mg1.request.mode = 1;
  portal_arrow_mg1.request.id = 20002;
  portal_arrow_mg1.request.x = 1500;
  portal_arrow_mg1.request.y = 1000;
  portal_arrow_mg1.request.z = 360;
  
  portal_arrow_mg2.request.mode = 1;
  portal_arrow_mg2.request.id = 20002;
  portal_arrow_mg2.request.x = 3500;
  portal_arrow_mg2.request.y = 2500;
  portal_arrow_mg2.request.z = 1060;

  portal_arrow_mg3.request.mode = 1;
  portal_arrow_mg3.request.id = 20002;
  portal_arrow_mg3.request.x = 5000;
  portal_arrow_mg3.request.y = 2000;
  portal_arrow_mg3.request.z = 360;




  ros::Rate loop_rate(10); 
  
  while(ros::ok()){
if(por1==1){
if(portal1.call(portal_mg1)){
 if(portal_mg1.response.tmsdb[0].weight == 1){
  portal_arrow_mg1.request.mode = 1;
  if(portal1_arrow.call(portal_arrow_mg1)){ 
    portal_arrow_mg1.request.mode = 0;
   //if(portal1_arrow.call(portal_arrow_mg1)){};
    }
  }
}
por1=0;
por2=1;
}

if(por2==1){
if(portal2.call(portal_mg2)){
 if(portal_mg2.response.tmsdb[0].weight == 1){
  portal_arrow_mg2.request.mode = 1;
  if(portal2_arrow.call(portal_arrow_mg2)){ 
    portal_arrow_mg2.request.mode = 0;
  // if(portal2_arrow.call(portal_arrow_mg2)){};
    }
  }
}
por2=0;
por3=1;
}


if(por3==1){
if(portal3.call(portal_mg3)){
 if(portal_mg3.response.tmsdb[0].weight == 1){
  portal_arrow_mg3.request.mode = 1;

  if(portal3_arrow.call(portal_arrow_mg3)){ 
    portal_arrow_mg3.request.mode = 0;
//   if(portal3_arrow.call(portal_arrow_mg3)){};
    }
  }
}

por3=0;
por1=1;
}

}
  return 0;
#endif

#if 0
//---------------------------------------------------------------------------------------
//tms_db_reader  + rp_arrow
//---------------------------------------------------------------------------------------
  int i = 0; 
  ros::init(argc, argv, "rps_manager");
  ros::NodeHandle nh;
  ros::ServiceClient hoge0 = nh.serviceClient<tms_msg_db::TmsdbGetData>("/tms_db_reader/dbreader");
  ros::ServiceClient ge0 = nh.serviceClient<tms_msg_rp::rp_arrow>("rp_arrow");
  tms_msg_db::TmsdbGetData hoge1;  
  tms_msg_rp::rp_arrow ge1;

  hoge1.request.tmsdb.id = 3011;
  
  ge1.request.mode = 1;
  ge1.request.id = 20002;
  ge1.request.x = 1000;
  ge1.request.y = 1000;
  ge1.request.z = 1000;
  
  ros::Rate loop_rate(10); 
  
  while(ros::ok()){
if(hoge0.call(hoge1))
{
 if(hoge1.response.tmsdb[0].weight == 1){

  //std::cout << i << std::endl

  ge1.request.mode = 1;
  if(ge0.call(ge1))
  { 
    ge1.request.mode = 0;
   if(ge0.call(ge1)){};
    }
  }
}

}
  return 0;
#endif

#if 0
//---------------------------------------------------------------------------------------
//tms_db_reader_test
//---------------------------------------------------------------------------------------
  int i = 0; 
  ros::init(argc, argv, "RaspberryPi_Sensor_Unit_init");
  ros::NodeHandle nh;
  ros::ServiceClient hoge0 = nh.serviceClient<tms_msg_db::TmsdbGetData>("/tms_db_reader/dbreader");
  
  tms_msg_db::TmsdbGetData hoge1;

  hoge1.request.tmsdb.id = 7001;
    
  ros::Rate loop_rate(10); 

  while(ros::ok()){
  std::cin >> i ;
 if(i==1){
  if(hoge0.call(hoge1))
  {
    std::cout << hoge1.response.tmsdb[0].x <<std::endl ;    
    }
  }
}
  return 0;
//
#endif

#if 0
//---------------------------------------------------------------------------------------
//arrow_object
//---------------------------------------------------------------------------------------
  int i = 0;
  ros::init(argc, argv, "RaspberryPi_Sensor_Unit_init");
  ros::NodeHandle nh;
  ros::ServiceClient hoge0 = nh.serviceClient<tms_msg_rp::rp_arrow>("rp_arrow");
  
  tms_msg_rp::rp_arrow hoge1;

  hoge1.request.mode = 1;
  hoge1.request.x = 1000;
  hoge1.request.y = 1000;
  hoge1.request.z = 1000;
    
  ros::Rate loop_rate(10); 

  while(ros::ok()){
  std::cin >> i ;
 if(i==1){
  hoge1.request.mode = 1;
  if(hoge0.call(hoge1))
  { 
    hoge1.request.mode = 0;
   if(hoge0.call(hoge1)){};
    }
  }
}
  return 0;
//-------------------------------------------------------------------------------------------
#endif

#if 0
-------------------------------------------------------------------------------------------
tms_db_writer_test
-------------------------------------------------------------------------------------------
  int i;
  ros::init(argc, argv, "RaspberryPi_Sensor_Unit_init");
  ros::NodeHandle nh;
  ros::Publisher db_pub    = nh.advertise<tms_msg_db::TmsdbStamped> ("tms_db_data", 10);

char str[100];
//char data[5]={'1','2','3','4','5'};

//std::string str;
std::string frame_id("/world"); 
//str = std::string(data);

int32_t id       = 3011;                                     //portable sensor id
int32_t idSensor = 3011;                                     //portable sensor id 
int32_t idPlace  = 5001;

ros::Rate loop_rate(10); 

while(ros::ok()){
	      i++;
	      sprintf(str,"%2d,%2d,%2d,%2d,%2d",i,i+1,i+2,i+3,i+4);
	      puts(str);
          ros::Time now = ros::Time::now() + ros::Duration(9*60*60); // GMT +9
          
          tms_msg_db::TmsdbStamped db_msg;
          tms_msg_db::Tmsdb tmpData;

          db_msg.header.frame_id  = frame_id;
          db_msg.header.stamp     = now;

          tmpData.time    = boost::posix_time::to_iso_extended_string(now.toBoost());
          tmpData.id      = id;
          tmpData.x       = 100;
          tmpData.y       = 100;
          tmpData.z       = 100;
          tmpData.rr      = 0;
          tmpData.rp      = 0;
          tmpData.ry      = 0;
          tmpData.etcdata = str;

          tmpData.place   = idPlace;
          tmpData.sensor  = idSensor;
          tmpData.state   = 1;

          db_msg.tmsdb.push_back(tmpData);
          
          db_pub.publish(db_msg);
          //std::cout << tmpData.etcdata << std::endl;
  }
  return 0;
-------------------------------------------------------------------------------------

#endif
}
