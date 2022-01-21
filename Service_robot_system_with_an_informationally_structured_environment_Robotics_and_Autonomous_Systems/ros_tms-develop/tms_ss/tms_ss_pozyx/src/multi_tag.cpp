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
#include <boost/date_time/posix_time/posix_time.hpp>
#include <tms_msg_db/TmsdbGetData.h>
#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_db/Tmsdb.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#define DEVNAME "/dev/arduino"
#define BAUDRATE B115200
#define BUFSIZE 256

#define PI 3.14159265
#define DEG2RAD(x) ((x)*(PI/180))

using namespace std;
using namespace boost;

int main(int argc, char **argv)
{
  ros::init(argc,argv,"tms_ss_pozyx_multi");
  ros::NodeHandle n;
  ros::Publisher pos_pub0 = n.advertise<visualization_msgs::Marker> ("pozyx0",1000);
  ros::Publisher pos_pub1 = n.advertise<visualization_msgs::Marker> ("pozyx1",1000);
  ros::Publisher pos_pub2 = n.advertise<visualization_msgs::Marker> ("pozyx2",1000);
  ros::Publisher pos_pub3 = n.advertise<visualization_msgs::Marker> ("pozyx3",1000);
  ros::Publisher pos_pub4 = n.advertise<visualization_msgs::Marker> ("pozyx4",1000);
  ros::Publisher pos_pub5 = n.advertise<visualization_msgs::Marker> ("pozyx5",1000);
  ros::Publisher db_pub = n.advertise<tms_msg_db::TmsdbStamped> ("tms_db_data",1000);
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

  tf::TransformBroadcaster broadcaster;
  tf::TransformListener listener;
  tf::Transform transform;

  transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
  transform.setRotation(tf::Quaternion(0,0,0));
  broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time(0),"world_link","pozyx_center"));
  transform.setOrigin(tf::Vector3(0.0,0.0,0.3));
  broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time(0),"pozyx_center","pozyx_neck"));
  transform.setOrigin(tf::Vector3(0.0,0.0,-0.4));
  broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time(0),"pozyx_center","pozyx_crotch"));
  transform.setOrigin(tf::Vector3(0.0,0.0,0.45));
  broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time(0),"pozyx_center","pozyx_head"));
  // transform.setOrigin(tf::Vector3(0.0,0.2,0));
  // broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time(0),"pozyx_neck","pozyx_shoulder_r"));
  // transform.setOrigin(tf::Vector3(0.0,-0.2,0));
  // broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time(0),"pozyx_neck","pozyx_shoulder_l"));
  // transform.setOrigin(tf::Vector3(0.0,0.0,-1.1));
  // broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time(0),"pozyx_center","pozyx_leg"));
  //

  int len;
  char buf[BUFSIZE];
  int id;
  double X,Y,Z,qx,qy,qz,qw;
  double preX,preY,preZ,preqx,preqy,preqz,preqw;
  double X0,Y0,Z0,X1,Y1,Z1,X2,Y2,Z2;
  while(ros::ok()){
		memset(&buf,0,sizeof(buf));
		len = read(fd,buf,BUFSIZE);
		if(len==0)
			continue;
    std::string data(buf);
  	std::vector<std::string> v_data;
  	v_data.clear();
    boost::split(v_data,data,boost::is_any_of(","));
    int id = atoi(v_data.at(0).c_str());
    X = 0.001*atoi(v_data.at(1).c_str());
    Y = 0.001*atoi(v_data.at(2).c_str());
    Z = 0.001*atoi(v_data.at(3).c_str());
    qx = atof(v_data.at(4).c_str());
    qy = atof(v_data.at(5).c_str());
    qz = atof(v_data.at(6).c_str());
    qw = atof(v_data.at(7).c_str());
    ROS_INFO("%x,%f,%f,%f,%f,%f,%f,%f",id,X,Y,Z,qx,qy,qz,qw);

    double r_roll,r_pitch,r_yaw;
    double roll,pitch,yaw;
    if(!(qx==0&&qy==0&&qz==0&&qw==0)){
      tf::Quaternion q0(qx,qy,qz,qw);
      tf::Matrix3x3 m(q0);
      m.getRPY(r_roll,r_pitch,r_yaw);
      r_yaw-=1.144898;
      roll=-r_pitch;
      pitch=r_roll-PI/2;
      yaw=r_yaw-PI/2;
    }

    // visualization_msgs::Marker msg;
    // msg.header.frame_id = "world_link";
    // msg.header.stamp = ros::Time();
    // msg.type = visualization_msgs::Marker::SPHERE;
    // msg.pose.position.x = X;
    // msg.pose.position.y = Y;
    // msg.pose.position.z = Z;
    // msg.pose.orientation.x = qx;
    // msg.pose.orientation.y = qy;
    // msg.pose.orientation.z = qz;
    // msg.pose.orientation.w = qw;
    // msg.scale.x = 0.5;
    // msg.scale.y = 0.5;
    // msg.scale.z = 0.5;
    // msg.color.a = 0.5;
    // if(id == 0x6077){
    //   msg.color.r = 1.0;
    //   pos_pub1.publish(msg);
    // }else if(id == 0x6076){
    //   msg.color.g = 1.0;
    //   pos_pub2.publish(msg);
    // }else if(id == 0x6e28){
    //   msg.color.b = 1.0;
    //   pos_pub3.publish(msg);
    // }else if(id == 0x6905){
    //   msg.color.r = 1.0;
    //   msg.color.g = 1.0;
    //   pos_pub4.publish(msg);
    // }else if(id == 0x6814){
    //   msg.color.r = 1.0;
    //   msg.color.b = 1.0;
    //   pos_pub5.publish(msg);
    // }
    if(id == 0x6077){
      transform.setOrigin(tf::Vector3(X,Y,Z));
      if(!(qx==0&&qy==0&&qz==0&&qw==0)){
        tf::Quaternion q1 = tf::createQuaternionFromRPY(roll,pitch,yaw);
        transform.setRotation(q1);
      }
      broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time(0),"world_link","pozyx_center"));
      printf("(%f,%f,%f) (%f,%f,%f)\n",X,Y,Z,roll,pitch,yaw);
    }
    visualization_msgs::Marker msg;
    msg.header.frame_id = "world_link";
    msg.header.stamp = ros::Time();
    msg.type = visualization_msgs::Marker::LINE_LIST;
    msg.scale.x = 0.1;
    msg.color.a = 1.0;
    msg.color.g = 1.0;
    if(id == 0x6077){
      tf::StampedTransform stamp;
      ros::Time now = ros::Time(0);
      listener.lookupTransform("world_link","pozyx_head",now,stamp);
      X0 = stamp.getOrigin().x();
      Y0 = stamp.getOrigin().y();
      Z0 = stamp.getOrigin().z();
      listener.lookupTransform("world_link","pozyx_neck",now,stamp);
      X1 = stamp.getOrigin().x();
      Y1 = stamp.getOrigin().y();
      Z1 = stamp.getOrigin().z();
      listener.lookupTransform("world_link","pozyx_crotch",now,stamp);
      X2 = stamp.getOrigin().x();
      Y2 = stamp.getOrigin().y();
      Z2 = stamp.getOrigin().z();
      geometry_msgs::Point p;
      p.x = X0;
      p.y = Y0;
      p.z = Z0;
      msg.points.push_back(p);
      p.x = X2;
      p.y = Y2;
      p.z = Z2;
      msg.points.push_back(p);
      pos_pub1.publish(msg);
      msg.type = visualization_msgs::Marker::SPHERE;
      msg.scale.x = 0.3;
      msg.scale.y = 0.3;
      msg.scale.z = 0.3;
      msg.pose.position.x = X0;
      msg.pose.position.y = Y0;
      msg.pose.position.z = Z0;
      pos_pub0.publish(msg);
    }else if(id == 0x6924){
      geometry_msgs::Point p;
      p.x = X1;
      p.y = Y1;
      p.z = Z1;
      msg.points.push_back(p);
      p.x = X;
      p.y = Y;
      p.z = Z;
      msg.points.push_back(p);
      pos_pub2.publish(msg);
    }else if(id == 0x6e3d){
      geometry_msgs::Point p;
      p.x = X1;
      p.y = Y1;
      p.z = Z1;
      msg.points.push_back(p);
      p.x = X;
      p.y = Y;
      p.z = Z;
      msg.points.push_back(p);
      pos_pub3.publish(msg);
    }else if(id == 0x6908){
      geometry_msgs::Point p;
      p.x = X2;
      p.y = Y2;
      p.z = Z2;
      msg.points.push_back(p);
      p.x = X;
      p.y = Y;
      p.z = Z;
      msg.points.push_back(p);
      pos_pub4.publish(msg);
    }else if(id == 0x6905){
      geometry_msgs::Point p;
      p.x = X2;
      p.y = Y2;
      p.z = Z2;
      msg.points.push_back(p);
      p.x = X;
      p.y = Y;
      p.z = Z;
      msg.points.push_back(p);
      pos_pub5.publish(msg);
    }


  }
  return 0;
}
