
#include "ros/ros.h"
#include "math.h"
#include "fstream"
#include "pthread.h"

#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

#include "ninebot/nucleo_serial.h"
#include "ninebot/odom_rad.h"

pthread_mutex_t mutex_serial = PTHREAD_MUTEX_INITIALIZER;

ros::Subscriber serial_sub;
ros::Subscriber imu_sub;
ros::Publisher  odom_pub;

//argument_impliment
double secs, before_secs;
double angular_velocity_z;
double imu_thai;
double position_x, position_y;

void simple_integral(double dr, double dl) { //Linear approximation
  position_x += cos(imu_thai) * (dr + dl) * 0.5;
  position_y += sin(imu_thai) * (dr + dl) * 0.5;
}

void precision_integral(double dr, double dl, double dt) { //Circular arc approximation 
  double dth, dx, dy;
  dth = angular_velocity_z * dt;
  if(dth != 0) {
    dx = ((dr + dl) * sin(dth * 0.5) * cos(imu_thai + (dth * 0.5))) / dth;
    dy = ((dr + dl) * sin(dth * 0.5) * sin(imu_thai + (dth * 0.5))) / dth;
  } else {
    dx = (dr + dl) * cos(imu_thai + (dth * 0.5)) * 0.5;
    dy = (dr + dl) * sin(imu_thai + (dth * 0.5)) * 0.5;            
  }
  position_x += dx; 
  position_y += dy;
}

void serial_callback(const ninebot::nucleo_serial::ConstPtr &serial)
{
  pthread_mutex_lock(&mutex_serial);

  before_secs = secs;
  secs = ros::Time::now().toSec();
  
  simple_integral(serial->delta_r, serial->delta_l);
  //precision_integral(serial->delta_r, serial->delta_l, serial->delta_t);

  imu_thai += angular_velocity_z * serial->delta_t;
  while(imu_thai >  M_PI) imu_thai -= 2.0*M_PI;
  while(imu_thai < -M_PI) imu_thai += 2.0*M_PI;

  //publish_data
  ninebot::odom_rad portable_odom;

  portable_odom.header.stamp = serial->header.stamp;
  portable_odom.position_z = 0.0;

  portable_odom.position_x = position_x;
  portable_odom.position_y = position_y;
  portable_odom.position_theta = imu_thai;
  
  std::cout << "x y theta " << portable_odom.position_x << " " << portable_odom.position_y << " " << portable_odom.position_theta << std::endl;

  portable_odom.velocity_x = (serial->delta_r + serial->delta_l) / (serial->delta_t * 2.0);
  portable_odom.velocity_y = 0.0;
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
  ros::init(argc, argv, "ninebot_stater");
  ros::NodeHandle n;  
  serial_sub = n.subscribe("nucleo_serial", 1, serial_callback);
  imu_sub    = n.subscribe("imu/data_raw" , 1, imu_callback);
  odom_pub   = n.advertise<ninebot::odom_rad>("odom_rad", 100);

  //initialize
  angular_velocity_z = 0.0;
  position_x = position_y = imu_thai = 0.0;
  secs = ros::Time::now().toSec();
  before_secs = secs; 

  spinner.spin();
  ros::waitForShutdown();

  return 0;
}
