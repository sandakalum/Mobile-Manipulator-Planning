#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <boost/algorithm/string.hpp>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include "kalman-Ndof.hpp"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <nav_msgs/Odometry.h>

#define DEVNAME "/dev/ttyACM0"
#define BAUDRATE B115200
#define BUFSIZE 256
#define PI 3.14159265
#define DEG2RAD(x) ((x)*(PI/180))

using namespace std;
using namespace boost;

Kalman *kalman;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pozyx");
  ros::NodeHandle n;
  ros::Publisher pos_pub = n.advertise<geometry_msgs::PoseStamped> ("pozyx", 1000);
  ros::Publisher ninebot_pos_pub = n.advertise<nav_msgs::Odometry> ("ninebot_measured_pos", 1000);
  std::string frame_id("/world_link");

	int fd = open(DEVNAME, O_RDWR | O_NOCTTY);
	if(fd < 0){
		ROS_ERROR("cannot open device");
		return 0;
	}
	ROS_INFO("device opened");

	struct termios tio;
	memset(&tio, 0, sizeof(tio));
	tio.c_cflag = CS8 | CLOCAL | CREAD;
	tio.c_cc[VTIME] = 0;
	tio.c_lflag = ICANON;
	tio.c_iflag = IGNPAR | ICRNL;
	cfsetspeed(&tio, BAUDRATE);
	tcsetattr(fd, TCSANOW, &tio);

  kalman = new Kalman(6, 6, 3);
  ros::Time last_time, now;
  now = ros::Time::now();

	int len;
	char buf[BUFSIZE];

  double X, Y, Z, qx, qy, qz, qw;
  double preX, preY, preZ, preqx, preqy, preqz, preqw;
  double kfX = 0;
  double kfY = 0;
  double kfZ = 0;
  bool GetInitRPY = true;
  double init_roll, init_pitch, init_yaw;

	while(ros::ok()){
		memset(&buf, 0, sizeof(buf));
		len = read(fd, buf, BUFSIZE);
		if(len == 0) continue;
	  std::string data(buf);
	  std::vector<std::string> v_data;
	  v_data.clear();
    boost::split(v_data, data, boost::is_any_of(","));

    if(v_data.size() == 8){

      preX = X;
      preY = Y;
      preZ = Z;
      preqx = qx;
      preqy = qy;
      preqz = qz;
      preqw = qw;

      int id = atoi(v_data.at(0).c_str());
      X = 0.001 * atoi(v_data.at(1).c_str());
      Y = 0.001 * atoi(v_data.at(2).c_str());
      Z = 0.001 * atoi(v_data.at(3).c_str());
      qx = atof(v_data.at(4).c_str());
      qy = atof(v_data.at(5).c_str());
      qz = atof(v_data.at(6).c_str());
      qw = atof(v_data.at(7).c_str());

      // get init eular angle as offset (called once)
      if(GetInitRPY){
        tf::Quaternion init_q(qx,qy,qz,qw);
        tf::Matrix3x3 init_mat(init_q);
        init_mat.getRPY(init_roll,init_pitch,init_yaw);
        GetInitRPY = false;
      }

      if(X == 0 && Y==0 && Z==0){
        X = preX;
        Y = preY;
        Z = preZ;
      }

      if(qx == 0 && qy == 0 && qz == 0 && qw == 0){
        qx = preqx;
        qy = preqy;
        qz = preqz;
        qw = preqw;
      }

      ROS_INFO("%d,%f,%f,%f,%f,%f,%f,%f",id, X, Y, Z, qx, qy, qz, qw);

      //calc orientation offset applied
      double ninebot_roll, ninebot_pitch, ninebot_yaw;
      tf::Quaternion temp_q(qx, qy, qz, qw);
      tf::Matrix3x3 temp_mat(temp_q);
      temp_mat.getRPY(ninebot_roll, ninebot_pitch, ninebot_yaw);

      ninebot_yaw -= init_yaw;

      tf::Matrix3x3 offset_mat;
      offset_mat.setEulerYPR(ninebot_yaw, ninebot_pitch, ninebot_roll);
      tf::Quaternion ninebot_quat;
      offset_mat.getRotation(ninebot_quat);

      //kalman_filter
      last_time = now;
      now = ros::Time::now();
      double delta_t = (now.sec + now.nsec * 0.000000001) - (last_time.sec + last_time.nsec * 0.000000001);
      double velX = (X - preX) / delta_t;
      double velY = (Y - preY) / delta_t;
      double velZ = (Z - preZ) / delta_t;

      static int count = 0;

      if(kalman == NULL) return 0;
      if(count == 0){
        count++;
        continue;
      }else if(count == 1){
        // 初期化 (システム雑音，観測雑音，積分時間)
        kalman->init(0.15,0.7,0.08);
        double C[6][6];
        memset(C, 0, sizeof(C));
        C[0][0] = C[1][1] = C[2][2] = 1;
        kalman->setC((double *)&C);
        kalman->setX(0, X);
        kalman->setX(1, Y);
        kalman->setX(2, Z);
        kalman->setX(3, velX);
        kalman->setX(4, velY);
        kalman->setX(5, velZ);
        count++;
      }

      double A[3][6];
      memset(A, 0, sizeof(A));
      A[0][0] = A[1][1] = A[2][2] = 1;
      A[0][3] = A[1][4] = A[2][5] = delta_t;
      double obs[6]={X, Y, Z, velX, velY, velZ};
      double input[3]={0, 0, 0};

      kalman->update(obs, input);

      kfX = kalman->getX(0);
      kfY = kalman->getX(1);
      kfZ = kalman->getX(2);

      ROS_INFO("kfX=%f kfY=%f kfZ=%f",kfX,kfY,kfZ);

      //----------------

      geometry_msgs::PoseStamped msg;
      msg.header.frame_id = frame_id;
      msg.header.stamp = ros::Time::now();
      msg.pose.position.x = kfX;
      msg.pose.position.y = kfY;
      msg.pose.position.z = kfZ;
      msg.pose.orientation.x = qx;
      msg.pose.orientation.y = qy;
      msg.pose.orientation.z = qz;
      msg.pose.orientation.w = qw;
      pos_pub.publish(msg);

      nav_msgs::Odometry odom;
      odom.header.frame_id = frame_id;
      odom.header.stamp = ros::Time::now();
      odom.pose.pose.position.x = kfX;
      odom.pose.pose.position.y = kfY;
      odom.pose.pose.position.z = kfZ;
      odom.pose.pose.orientation.x = qx - ninebot_quat.getX();
      odom.pose.pose.orientation.y = qy - ninebot_quat.getY();
      odom.pose.pose.orientation.z = qz - ninebot_quat.getZ();
      odom.pose.pose.orientation.w = qw - ninebot_quat.getW();
      ninebot_pos_pub.publish(odom);

    }
  }
  return 0;
}
