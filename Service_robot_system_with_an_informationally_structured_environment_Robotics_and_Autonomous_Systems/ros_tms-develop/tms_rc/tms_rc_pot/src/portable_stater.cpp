//----------------------------------------------------------
// @file   : virtual_por_test.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.4 (since 2015.08.05)
// @date   : 2015.08.05
//----------------------------------------------------------
#include "ros/ros.h"
#include "math.h"
#include "pthread.h"
#include "fstream"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "tms_msg_rc/arduino_serial.h"
#include "tms_msg_rc/odom_rad.h"

#define ONE_PULSE_LENGTH 0.94247779607//0.95440789476//0.94247779607//0.94247779607mm

pthread_mutex_t mutex_serial = PTHREAD_MUTEX_INITIALIZER;

//argument_impliment
double angular_velocity_z;
int    before_encorder_front;
int    before_encorder_left;
int    before_encorder_back;
double secs, before_secs;
double imu_thai;
double v_front_, v_left_, v_back_;
double true_x_v_, true_y_v_, true_theta_v_;
double Vx_, Vy_, Vtheta_;
double Vx_position, Vy_position, Vtheta_position;

ros::Subscriber serial_sub;
ros::Subscriber imu_sub;
ros::Publisher  odom_pub;

//true_velicity----------------------------------------------------------------------------------------
void true_velocity()
{
  /*
  true_x_v_     = 0.666 * v_front_ - 0.333 * v_left_ - 0.333 * v_back_;//[ 1.0  0.0    46.62]
  true_y_v_     = 0.000 * v_front_ + 0.577 * v_left_ - 0.577 * v_back_;//[-0.5  0.866  46.62] R
  true_theta_v_ = 0.007 * v_front_ + 0.007 * v_left_ + 0.007 * v_back_;//[-0.5  0.866  46.62] の逆行列
  */
  true_x_v_     = -0.57735026919258  * v_front_ + 0.0                 * v_left_ + 0.57735026919258   * v_back_;//[ -0.8  0.5  105.0]
  true_y_v_     = -1.0               * v_front_ + 2.0                 * v_left_ - 1.0                * v_back_;//[  0.0  1.0  105.0] R
  true_theta_v_ = 0.0095238095238095 * v_front_ + -0.0095238095238095 * v_left_ + 0.0095238095238095 * v_back_;//[  0.8  0.5  105.0] の逆行列
}
//-----------------------------------------------------------------------------------------------------

//car position-----------------------------------------------------------------------------------------
void getcar_position(double rate_time)
{
  Vx_     = cos(Vtheta_position) * true_x_v_ - sin(Vtheta_position)  * true_y_v_;
  Vy_     = sin(Vtheta_position) * true_x_v_ + cos(Vtheta_position)  * true_y_v_;
  Vtheta_ = true_theta_v_;

  Vx_position     += Vx_     * rate_time;
  Vy_position     += Vy_     * rate_time;
  //Vtheta_position += Vtheta_ * rate_time; //omni_data
  Vtheta_position = imu_thai; //imu_data

  while(Vtheta_position > M_PI)Vtheta_position  -= 2.0*M_PI;
  while(Vtheta_position < -M_PI)Vtheta_position += 2.0*M_PI;
}
//------------------------------------------------------------------------------------------------------

//getcarspeed_mmps----------------------------------------------------------------------------------
double calculate_run_length(int pulses)
{
  double run_length_replacement;
  run_length_replacement = ONE_PULSE_LENGTH * pulses;
  return run_length_replacement;
}
//-----------------------------------------------------------------------------------------------------

//getcarspeed_mmps----------------------------------------------------------------------------------
void getcar_speedmmps(int front_pulse, int left_pulse, int back_pulse, double rate_time)
{
  //front
  v_front_     = calculate_run_length(front_pulse - before_encorder_front);
  v_front_     = v_front_ / rate_time;
  before_encorder_front = front_pulse;
  
  //left
  v_left_      = calculate_run_length(left_pulse - before_encorder_left);
  v_left_      = v_left_  / rate_time;
  before_encorder_left  = left_pulse;

  //back
  v_back_      = calculate_run_length(back_pulse - before_encorder_back);
  v_back_      = v_back_ / rate_time;
  before_encorder_back  = back_pulse;
}
//-----------------------------------------------------------------------------------------------------
 
void serial_callback(const tms_msg_rc::arduino_serial::ConstPtr &serial)
{
  pthread_mutex_lock(&mutex_serial);
  
  //judge encorder plot
  secs   = ros::Time::now().toSec();
  if(before_encorder_front == serial->encorder_front && 
     before_encorder_left == serial->encorder_left   &&
     before_encorder_back == serial->encorder_back     )
  {
    angular_velocity_z = 0.0;
  }

  //calcurate position
  imu_thai = imu_thai + angular_velocity_z * (secs - before_secs);
  getcar_speedmmps(serial->encorder_front, serial->encorder_left, serial->encorder_back, (secs - before_secs));
  true_velocity();
  getcar_position(secs - before_secs);
  before_secs = secs;

  //publish_data
  tms_msg_rc::odom_rad portable_odom;

  portable_odom.header.stamp = serial->header.stamp;
  portable_odom.position_x = Vx_position / 1000.0;
  portable_odom.position_y = Vy_position / 1000.0;
  portable_odom.position_z = 0.0;
  portable_odom.position_theta = Vtheta_position;
  std::cout << "x y theta " << portable_odom.position_x << " " << portable_odom.position_y << " " << portable_odom.position_theta << std::endl;

  portable_odom.velocity_x = true_x_v_ / 1000.0;
  portable_odom.velocity_y = true_y_v_ / 1000.0;
  portable_odom.velocity_z = 0.0;
  portable_odom.velocity_theta = angular_velocity_z;
  odom_pub.publish(portable_odom);

  pthread_mutex_unlock(&mutex_serial);
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &imu)
{
  pthread_mutex_lock(&mutex_serial);
  angular_velocity_z = imu->angular_velocity.z; //unit rad/s
  pthread_mutex_unlock(&mutex_serial);
}

int main(int argc, char **argv)
{
  //thread setting
  pthread_t thread_p;
  ros::MultiThreadedSpinner spinner(4);
  
  //ros setting
  ros::init(argc, argv, "portable_robot");
  ros::NodeHandle n;  
  serial_sub = n.subscribe("arduino_serial", 1, serial_callback);
  imu_sub    = n.subscribe("imu/data_raw" , 1, imu_callback);
  odom_pub   = n.advertise<tms_msg_rc::odom_rad>("odom_rad", 100);

  //initialize
  angular_velocity_z = 0.0;
  before_encorder_front = 0;
  before_encorder_left  = 0;
  before_encorder_back  = 0;
  secs   = ros::Time::now().toSec();
  before_secs = ros::Time::now().toSec(); 

  //portable_position_initialize
  imu_thai = 0.0;

  spinner.spin();
  ros::waitForShutdown();

  return 0;
}
