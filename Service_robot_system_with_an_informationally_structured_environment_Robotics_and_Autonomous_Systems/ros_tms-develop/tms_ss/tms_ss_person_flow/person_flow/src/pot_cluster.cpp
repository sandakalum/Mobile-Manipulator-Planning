//----------------------------------------------------------
// @file   : pot_sensor.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.1
// @date   : 2015.1.16
//----------------------------------------------------------

//----------------------------------------------------------
//include,define
//----------------------------------------------------------

#include <ros/ros.h>
#include <pthread.h>
#include <std_msgs/String.h>
#include <tms_msg_ss/pot_sensor.h>

#include <iostream>

#define Time_Shreshhold 15.0

pthread_mutex_t mutex_laser  = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_target  = PTHREAD_MUTEX_INITIALIZER;

//----------------------------------------------------------
//impriment setting
//----------------------------------------------------------
bool lrf_key      = false;
bool lrf_sub_key  = false;

double StartTime;
double ExeTime;

ros::Subscriber sensor_sub;
ros::Publisher  odroid_info;

//----------------------------------------------------------
//open_lrf
//----------------------------------------------------------
void *open_lrf(void *ptr )
{
  ros::Rate r(1);
  while(ros::ok())
  {
    if(lrf_key)
    {
      lrf_sub_key = true;
      system("roslaunch tms_ss_pot pot_urg.launch");
    }
    r.sleep();
  }
}

//----------------------------------------------------------
//close_lrf
//----------------------------------------------------------
void *close_lrf(void *ptr )
{
  ros::Rate r(1);
  while(ros::ok())
  {
    if(lrf_key != true)
    {
      if(lrf_sub_key){
      system("rosnode kill /pot_urg1/urg_node");
      lrf_sub_key = false;
      }
    }
    r.sleep();
  }
}

//----------------------------------------------------------
//sensor Callback
//----------------------------------------------------------

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  pthread_mutex_lock(&mutex_target);
  tms_msg_ss::pot_sensor sensor;

  int count;
  int distance;
  int motion;
  int flame;
  int microphone;
  int gas;
  std::string temperature;
  std::string humidity;

  system("clear");

  std::string line;
  line = msg->data.c_str();

  //----------------------------------------------------------
  //display sensor condition
  //----------------------------------------------------------
  int fp = 0;
  int fn;

  fn = line.find(";" , fp);
  fn = fn - fp;
  count = atoi(line.substr(fp, fn).c_str());
  std::cout << "number " << atoi(line.substr(fp, fn).c_str()) << std::endl;

  fp = fp + fn + 1;

  for (int i = 0; i < 7; i++)
  {
      fn = line.find(";" , fp);
      fn = fn - fp;

      if(i == 0)
      {
      distance = atoi(line.substr(fp, fn).c_str());
      std::cout << "distance " << distance << std::endl;
      }
      if(i == 1)
      {
      motion = atoi(line.substr(fp, fn).c_str());
      std::cout << "motion " << motion << std::endl;
      }
      if(i == 2)
      {
      flame = atoi(line.substr(fp, fn).c_str());
      std::cout << "flame " << flame << std::endl;
      }
      if(i == 3)
      {
      microphone = atoi(line.substr(fp, fn).c_str());
      std::cout << "microphotne " << microphone << std::endl;
      }
      if(i == 4)
      {
      gas = atoi(line.substr(fp, fn).c_str());
      std::cout << "gas " << gas << std::endl;
      }
      if(i == 5)
      {
      temperature = line.substr(fp, fn).c_str();
      std::cout << "temperature " << line.substr(fp, fn).c_str() << std::endl;
      }
      if(i == 6)
      {
      humidity = line.substr(fp, fn).c_str();
      std::cout << "humidity " << line.substr(fp, fn).c_str() << std::endl;
      }
      fp = fp + fn + 1;


      ExeTime = ros::Time::now().toSec() - StartTime;

      if(lrf_key!= true && (distance ==1 || motion == 1 || flame == 1 || microphone == 1 || gas == 1))
      {
        lrf_key  = true ;
        StartTime = ros::Time::now().toSec();
      }
      else if(lrf_key && (ExeTime > Time_Shreshhold))
      {
         lrf_key  = false ;
      }
  }
   std::cout << "lrf " << lrf_key << std::endl;

   sensor.count                = count;

   if(distance)sensor.distance = "ON";
   else sensor.distance        = "OFF";

   if(motion)sensor.motion     = "ON";
   else sensor.motion          = "OFF";

   if(flame) sensor.flame      = "ON";
   else sensor.flame           = "OFF";

   if(gas)sensor.gas           = "ON";
   else sensor.gas             = "OFF";

   if(microphone)sensor.microphone= "ON";
   else sensor.microphone      = "OFF";

   if(lrf_key)sensor.lrf_key  = "ON";
   else   sensor.lrf_key      = "OFF";
   
   sensor.temperature    = temperature;
   sensor.humidity       = humidity;
   
   odroid_info.publish(sensor);

   pthread_mutex_unlock(&mutex_target);
}

int main(int argc, char **argv)
{
  pthread_t thread_p;
  pthread_t thread_v;
  ros::MultiThreadedSpinner spinner(4);

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  sensor_sub = n.subscribe("portable_info", 10, chatterCallback);
  odroid_info = n.advertise<tms_msg_ss::pot_sensor>("odroid_info", 1000);

  if( pthread_create( &thread_p, NULL, open_lrf, NULL) )
    {
        printf("error creating thread.");
        abort();
    }
  if( pthread_create( &thread_v, NULL, close_lrf, NULL) )
    {
        printf("error creating thread.");
        abort();
    }

  spinner.spin(); 

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
