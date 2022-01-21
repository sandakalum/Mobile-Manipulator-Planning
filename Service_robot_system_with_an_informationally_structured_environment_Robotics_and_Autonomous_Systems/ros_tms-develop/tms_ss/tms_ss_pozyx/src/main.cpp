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
#include <tms_msg_db/TmsdbGetData.h>
#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_db/Tmsdb.h>
#include <visualization_msgs/Marker.h>

#define DEVNAME "/dev/arduino"
#define BAUDRATE B115200
#define BUFSIZE 256

#define PI 3.14159265
#define DEG2RAD(x) ((x)*(PI/180))

using namespace std;
using namespace boost;

Kalman *kalman;

#define BED_X1 9.33
#define BED_X2 10.7
#define BED_Y1 2.3
#define BED_Y2 3.1

int main(int argc, char **argv)
{
  ros::init(argc,argv,"tms_ss_pozyx");
  ros::NodeHandle n;
  ros::Publisher db_pub = n.advertise<tms_msg_db::TmsdbStamped> ("tms_db_data",1000);
  ros::Publisher pos_pub = n.advertise<visualization_msgs::Marker> ("pozyx",1000);
  std::string frame_id("/world");

	int fd = open(DEVNAME, O_RDWR | O_NOCTTY);
	if(fd<0){
		ROS_ERROR("cannot open device");
		return 0;
	}
	ROS_INFO("device opened");

	struct termios tio;
	memset(&tio,0,sizeof(tio));
	tio.c_cflag = CS8 | CLOCAL | CREAD;
	tio.c_cc[VTIME]=0;
	tio.c_lflag=ICANON;
	tio.c_iflag=IGNPAR | ICRNL;
	cfsetspeed(&tio,BAUDRATE);
	tcsetattr(fd,TCSANOW,&tio);

  kalman = new Kalman(6,6,3);
  ros::Time last_time,now;
  now = ros::Time::now();

	int len;
	char buf[BUFSIZE];
  double X,Y,Z,qx,qy,qz,qw;
  double preX,preY,preZ,preqx,preqy,preqz,preqw;
  double kfX=0;
  double kfY=0;
  double kfZ=0;
	while(ros::ok()){
		memset(&buf,0,sizeof(buf));
		len = read(fd,buf,BUFSIZE);
		if(len==0)
			continue;
	  std::string data(buf);
	  std::vector<std::string> v_data;
	  v_data.clear();
    boost::split(v_data,data,boost::is_any_of(","));
    if(v_data.size()==8){
      preX=X;
      preY=Y;
      preZ=Z;
      preqx=qx;
      preqy=qy;
      preqz=qz;
      preqw=qw;
      int id = atoi(v_data.at(0).c_str());
      X = 0.001*atoi(v_data.at(1).c_str());
      Y = 0.001*atoi(v_data.at(2).c_str());
      Z = 0.001*atoi(v_data.at(3).c_str());
      qx = atof(v_data.at(4).c_str());
      qy = atof(v_data.at(5).c_str());
      qz = atof(v_data.at(6).c_str());
      qw = atof(v_data.at(7).c_str());
      if(qx==0&&qy==0&&qz==0&&qw==0){
        qx=preqx;
        qy=preqy;
        qz=preqz;
        qw=preqw;
      }

      ROS_INFO("%d,%f,%f,%f,%f,%f,%f,%f",id,X,Y,Z,qx,qy,qz,qw);

      tf::Quaternion q0(qx,qy,qz,qw);
      tf::Matrix3x3 m(q0);
      double roll,pitch,yaw;
      m.getRPY(roll,pitch,yaw);
      yaw-=1.144898;
      double roll2,pitch2,yaw2;
      roll2=-pitch;
      pitch2=roll-PI/2;
      yaw2=yaw-PI/2;

      //kalman_filter
      last_time = now;
      now = ros::Time::now();
      double delta_t = (now.sec+now.nsec*0.000000001)-(last_time.sec+last_time.nsec*0.000000001);
      double velX = (X-preX)/delta_t;
      double velY = (Y-preY)/delta_t;
      double velZ = (Z-preZ)/delta_t;

      static int count = 0;

      if(kalman==NULL)return 0;
      if(count==0){
        count++;
        continue;
      }else if(count==1){
        // 初期化 (システム雑音，観測雑音，積分時間)
        kalman->init(0.15,0.7,0.08);
        double C[6][6];
        memset(C,0,sizeof(C));
        C[0][0]=C[1][1]=C[2][2]=1;
        kalman->setC((double *)&C);
        kalman->setX(0,X);
        kalman->setX(1,Y);
        kalman->setX(2,Z);
        kalman->setX(3,velX);
        kalman->setX(4,velY);
        kalman->setX(5,velZ);
        count++;
      }

      double A[3][6];
      memset(A,0,sizeof(A));
      A[0][0]=A[1][1]=A[2][2]=1;
      A[0][3]=A[1][4]=A[2][5]=delta_t;
      double obs[6]={X,Y,Z,velX,velY,velZ};
      double input[3]={0,0,0};

      kalman->update(obs,input);

      kfX=kalman->getX(0);
      kfY=kalman->getX(1);
      kfZ=kalman->getX(2);

      ROS_INFO("kfX=%f kfY=%f kfZ=%f",kfX,kfY,kfZ);

      //----------------

      ros::Time db_time = ros::Time::now() + ros::Duration(9*60*60);
      tms_msg_db::TmsdbStamped db_msg;

      db_msg.header.frame_id = frame_id;
      db_msg.header.stamp = db_time;
      db_msg.tmsdb.clear();
      tms_msg_db::Tmsdb tmpData;

      tmpData.time = boost::posix_time::to_iso_extended_string(db_time.toBoost());
      tmpData.name = "person_pozyx1";
      tmpData.id = 1100;
      tmpData.sensor = 0;
      tmpData.state = 1;
      tmpData.x = kfX;
      tmpData.y = kfY;
      tmpData.rr = roll2;
      tmpData.rp = pitch2;
      tmpData.ry = yaw2;
      double height = 1.1*cos(pitch2)*cos(roll2);
      if(height<0.1) height = 0.1;

      if(kfX > BED_X1 && kfX < BED_X2 && kfY > BED_Y1 && kfY < BED_Y2){
        tmpData.place = 6017;
        tmpData.z = height+0.3;
      }else{
        tmpData.place = 5001;
        tmpData.z = height;
      }

      db_msg.tmsdb.push_back(tmpData);
      db_pub.publish(db_msg);

      visualization_msgs::Marker msg;
      msg.header.frame_id = "world_link";
      msg.header.stamp = ros::Time();
      msg.type = visualization_msgs::Marker::SPHERE;
      msg.pose.position.x = kfX;
      msg.pose.position.y = kfY;
      msg.pose.position.z = kfZ;
      msg.pose.orientation.x = qx;
      msg.pose.orientation.y = qy;
      msg.pose.orientation.z = qz;
      msg.pose.orientation.w = qw;
      msg.scale.x = 0.5;
      msg.scale.y = 0.5;
      msg.scale.z = 0.5;
      msg.color.a = 0.5;
      msg.color.r = 1.0;
      pos_pub.publish(msg);
    }
  }
  return 0;
}
