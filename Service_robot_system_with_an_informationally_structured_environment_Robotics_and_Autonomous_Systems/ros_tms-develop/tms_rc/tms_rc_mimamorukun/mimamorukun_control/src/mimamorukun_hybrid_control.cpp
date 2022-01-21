//------------------------------------------------------------------------------
// FileName : mimamorukun_automatic_control.cpp
// Date     : 2015.01.30
// author   : Akio Shigekane
//------------------------------------------------------------------------------
// 2014. 9.23        adjust scale of speed
// 2014.11.01        adjust wheel spin PID constants
// 2014.11.07        included to ROT_TMS project
// 2015.01.30        fork from mimamorukun_manual_control
// 2015.02.01        fork from mimamorukun_automatic_control
#include <ros/ros.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>

#include "ClientSocket.h"
#include "SocketException.h"
#include <iostream>
#include <string>

#include <sensor_msgs/Joy.h>
#include <tms_msg_db/TmsdbGetData.h>
#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_db/Tmsdb.h>
#include <tms_msg_rc/rc_robot_control.h>

#include "kalman-Ndof.hpp"

#define ROS_RATE 10

using namespace std;

ClientSocket client_socket("", 54300);
const int ENC_MAX = 3932159;
const int SPEED_MAX = 32767;
const float DIST_PER_PULSE = 0.552486;  // mm par pulse
const int WHEEL_DIST = 570;             // 533;

long int ENC_L = 0;
long int ENC_R = 0;
int POS_X = 0;
int POS_Y = 0;

double TURN_KP;
double SPD_KP;
int ARV_DIST;

/*double joy_cmd_spd = 0.0;
double joy_cmd_turn = 0.0;*/

ros::ServiceClient db_client;
ros::Publisher db_pub;

pthread_t thread_vicon;
pthread_t thread_odom;
pthread_mutex_t mutex_update_kalman = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_socket = PTHREAD_MUTEX_INITIALIZER;

class MachinePose_s
{
private:
public:
  MachinePose_s()
  {
    ROS_DEBUG("In Mimamorukun Constructor");
    this->pos_vicon.x = 0.0;
    this->pos_vicon.y = 0.0;
    this->pos_vicon.theta = 0.0;
    this->pos_odom.x = 0.0;
    this->pos_odom.y = 0.0;
    this->pos_odom.theta = 0.0;
    this->pos_fusioned.x = 0.0;
    this->pos_fusioned.y = 0.0;
    this->pos_fusioned.theta = 0.0;
    this->vel_odom.x = 0.0;
    this->vel_odom.y = 0.0;
    this->vel_odom.theta = 0.0;
    this->vel_vicon.x = 0.0;
    this->vel_vicon.y = 0.0;
    this->vel_vicon.theta = 0.0;
    this->vel_fusioned.x = 0.0;
    this->vel_fusioned.y = 0.0;
    this->vel_fusioned.theta = 0.0;
    kalman = new Kalman(6, 3, 3);
  };
  ~MachinePose_s()
  {
    delete kalman;
  };
  void updateOdom();
  void updateVicon();
  void updateCompFilter();
  bool goPose(/*const geometry_msgs::Pose2D::ConstPtr& cmd_pose*/);
  bool postPose();
  geometry_msgs::Pose2D tgtPose;
  geometry_msgs::Twist tgtTwist;
  geometry_msgs::Pose2D pos_odom;
  geometry_msgs::Pose2D pos_vicon;
  geometry_msgs::Pose2D pos_fusioned;
  /*    geometry_msgs::Twist    vel_odom;
      geometry_msgs::Twist    vel_vicon;
      geometry_msgs::Twist    vel_fusioned;*/
  geometry_msgs::Pose2D vel_odom;
  geometry_msgs::Pose2D vel_vicon;
  geometry_msgs::Pose2D vel_fusioned;

  void setCurrentPosition(geometry_msgs::Pose2D pose);
  geometry_msgs::Pose2D getCurrentPosition();
  double getCurrentPositionX();
  double getCurrentPositionY();
  double getCurrentTheta();
  double getCurrentVelocityX();
  double getCurrentVelocityY();
  double getCurrentOmega();
  // bool goPose2(/*const geometry_msgs::Pose2D::ConstPtr& cmd_pose*/);

  Kalman *kalman;
  enum InfoType
  {
    VELOCITY,
    POSISION
  };
  geometry_msgs::Pose2D UpdatePosition(geometry_msgs::Pose2D tpose, InfoType Info);
} mchn_pose;

int Dist2Pulse(int dist)
{
  return ((float)dist) / DIST_PER_PULSE;
}
int Pulse2Dist(int pulse)
{
  return ((float)pulse) * DIST_PER_PULSE;
}
double Rad2Deg(double rad)
{
  return rad * (180.0) / M_PI;
}
double Deg2Rad(double deg)
{
  return deg * M_PI / 180.0;
}
// double Deg2Rad(double deg) { return deg * 3 / 180.0; }
double MM2M(double mm)
{
  return mm * 0.001;
}
double M2MM(double M)
{
  return M * 1000;
}
double sqr(double val)
{
  return pow(val, 2);
}
double Limit(double val, double max, double min)
{
  if (val > max)
    return max;
  else if (min > val)
    return min;
  else
    return val;
}
double nomalizeAng(double rad)
{
  while (rad > M_PI)
  {  //角度を-180°~180°(-π~π)の範囲に合わせる
    rad = rad - (2 * M_PI);
  }
  while (rad < -M_PI)
  {
    rad = rad + (2 * M_PI);
  }
  return rad;
}

void MachinePose_s::updateVicon()
{
  tms_msg_db::TmsdbGetData srv;
  srv.request.tmsdb.id = 2007;
  srv.request.tmsdb.sensor = 3001;
  if (!db_client.call(srv))
  {
    ROS_ERROR("Failed to get vicon data from DB via tms_db_reader");
  }
  else if (srv.response.tmsdb.empty())
  {
    ROS_ERROR("DB response empty");
  }
  else
  {
    boost::posix_time::ptime set_time = boost::posix_time::time_from_string(srv.response.tmsdb[0].time);
    ros::Time ros_now = ros::Time::now() + ros::Duration(9 * 60 * 60);
    boost::posix_time::ptime now = ros_now.toBoost();
    // cout << "time:" << now-set_time << "\n";
    // cout << "3sec:" << boost::posix_time::time_duration(0,0,3,0) << "\n";
    if (boost::posix_time::time_duration(0, 0, 3, 0) > now - set_time)
    {  // check if data is fresh (in last 3sec)
      this->pos_vicon.x = srv.response.tmsdb[0].x;
      this->pos_vicon.y = srv.response.tmsdb[0].y;
      this->pos_vicon.theta = Deg2Rad(srv.response.tmsdb[0].ry);
    }
  }
  return;
}

void MachinePose_s::updateOdom()
{
  // update Encoder value
  long int tmpENC_L = 0;
  long int tmpENC_R = 0;
  string reply;
  pthread_mutex_lock(&mutex_socket);
  client_socket << "@GP1@GP2"; /*use 250ms for send and get reply*/
  client_socket >> reply;
  pthread_mutex_unlock(&mutex_socket);
  ROS_DEBUG_STREAM("@GP raw:" << reply);
  sscanf(reply.c_str(), "@GP1,%ld@GP2,%ld", &tmpENC_L, &tmpENC_R);
  ROS_DEBUG_STREAM("tmpENC_L:" << tmpENC_L << "    tmpENC_R:" << tmpENC_R);
  if (tmpENC_L > ENC_MAX / 2)
    ENC_L = tmpENC_L - (ENC_MAX + 1);
  else
    ENC_L = tmpENC_L;
  if (tmpENC_R > ENC_MAX / 2)
    ENC_R = tmpENC_R - (ENC_MAX + 1);
  else
    ENC_R = tmpENC_R;

  static long int ENC_R_old = 0;
  static long int ENC_L_old = 0;
  double detLp /*,r , dX ,dY,dL*/;

  if (fabs(ENC_L - ENC_L_old) > ENC_MAX / 2)
    ENC_L_old -= ENC_MAX + 1;
  if (fabs(ENC_R - ENC_R_old) > ENC_MAX / 2)
    ENC_R_old -= ENC_MAX + 1;

  //エンコーダーの位置での前回からの移動距離dL_R,dL_Lを算出
  double dL_L = (double)(ENC_L - ENC_L_old) * (-DIST_PER_PULSE);
  double dL_R = (double)(ENC_R - ENC_R_old) * DIST_PER_PULSE;

  //角速度SIGMAを算出
  double POS_SIGMA = (dL_R - dL_L) / WHEEL_DIST;
  double dL = (dL_R + dL_L) * 0.50000;

  double dX = dL * cos(mchn_pose.pos_odom.theta + POS_SIGMA);  // X,Yの前回からの移動量計算
  double dY = dL * sin(mchn_pose.pos_odom.theta + POS_SIGMA);
  ENC_R_old = ENC_R;  //前回のエンコーダーの値を記録
  ENC_L_old = ENC_L;
  mchn_pose.pos_odom.x += dX;
  mchn_pose.pos_odom.y += dY;
  mchn_pose.pos_odom.theta += POS_SIGMA;
  mchn_pose.pos_odom.theta = nomalizeAng(mchn_pose.pos_odom.theta);
  mchn_pose.vel_odom.x = dX;
  mchn_pose.vel_odom.y = dY;
  mchn_pose.vel_odom.theta = POS_SIGMA;
  return;
}

/*------------------------------------
 * send command to mbed in wheel chair
 * argument:
 *   arg_speed: forward speed   [mm/sec]
 *   arg_theta: CCW turn speed  [radian/sec]
 * ----------------------------------*/
void spinWheel(/*double arg_speed, double arg_theta*/)
{
  double arg_speed = mchn_pose.tgtTwist.linear.x;
  double arg_theta = mchn_pose.tgtTwist.angular.z;
  // ROS_INFO("X:%4.2f   Theta:%4.2f",arg_speed,arg_theta);
  double val_L = -Dist2Pulse(arg_speed) + Dist2Pulse((WHEEL_DIST / 2) * arg_theta);
  double val_R = Dist2Pulse(arg_speed) + Dist2Pulse((WHEEL_DIST / 2) * arg_theta);
  val_L = (int)Limit(val_L, (double)SPEED_MAX, (double)-SPEED_MAX);
  val_R = (int)Limit(val_R, (double)SPEED_MAX, (double)-SPEED_MAX);
  printf("val_L:%2.f   val_R:%2.f", val_L, val_R);

  string cmd_L = boost::lexical_cast< string >(val_L);
  string cmd_R = boost::lexical_cast< string >(val_R);

  string message;
  message = "@SS1," + cmd_L + "@SS2," + cmd_R;
  pthread_mutex_lock(&mutex_socket);
  client_socket << message;
  // message = "@SS2," + cmd_R;
  // client_socket << message;
  string reply;
  client_socket >> reply;
  pthread_mutex_unlock(&mutex_socket);
  ROS_DEBUG_STREAM("@SS raw: " << reply);
  // cout << "Response:" << reply << "   ";
}

// void receiveGoalPose(const geometry_msgs::Pose2D::ConstPtr& cmd_pose){
//     mchn_pose.tgtPose = *cmd_pose;
// }

bool receiveGoalPose(tms_msg_rc::rc_robot_control::Request &req, tms_msg_rc::rc_robot_control::Response &res)
{
  if (1 != req.unit)
  {                  // Is not vicle unit
    res.result = 0;  // SRV_UNIT_ERR;
    return true;
  }
  if (15 != req.cmd)
  {
    res.result = 0;  // SRV_CMD_ERR;
    return true;
  }
  mchn_pose.tgtPose.x = req.arg[0];
  mchn_pose.tgtPose.y = req.arg[1];
  mchn_pose.tgtPose.theta = Deg2Rad(req.arg[2]);
  // while(! mchn_pose.goPose()){
  //     ROS_INFO("doing goPose");
  // }
  // ros::Rate r(4);
  ros::Rate r(ROS_RATE);
  while (ros::ok())
  {
    ROS_DEBUG("doing goPose");
    printf("pos x:%4.2lf y:%4.2lf th:%4.2lf     \n", mchn_pose.pos_vicon.x, mchn_pose.pos_vicon.y,
           Rad2Deg(mchn_pose.pos_vicon.theta));
    printf("tgt x:%4.2lf y:%4.2lf th:%4.2lf     ", mchn_pose.tgtPose.x, mchn_pose.tgtPose.y,
           Rad2Deg(mchn_pose.tgtPose.theta));
    bool isArrived = mchn_pose.goPose();
    mchn_pose.updateVicon();
    spinWheel();
    mchn_pose.postPose();
    if (isArrived)
      break;
    r.sleep();
  }
  // while(1){
  //     // printf("in moving loop");
  //     ros::Duration(0.1).sleep();
  //     printf("doing goPose");
  //     ROS_INFO("pos x:%4.2lf y:%4.2lf th:%4.2lf     ",
  //         mchn_pose.pos_vicon.x,
  //         mchn_pose.pos_vicon.y,
  //         Rad2Deg(mchn_pose.pos_vicon.theta));
  //     printf("tgt x:%4.2lf y:%4.2lf th:%4.2lf     ",
  //         mchn_pose.tgtPose.x,
  //         mchn_pose.tgtPose.y,
  //         Rad2Deg(mchn_pose.tgtPose.theta));
  //     bool isArrived = mchn_pose.goPose();
  //     mchn_pose.updateVicon();
  //     spinWheel();
  //     if(isArrived) break;
  //  }
  return true;
}

void receiveCmdVel(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
  mchn_pose.tgtTwist = *cmd_vel;
  // spinWheel(/*cmd_vel->linear.x,cmd_vel->angular.z*/);
}

void receiveJoy(const sensor_msgs::Joy::ConstPtr &joy)
{
  // ROS_INFO("Rrecieve joy");
  mchn_pose.tgtTwist.linear.x = joy->axes[1] * 300;   // 600;
  mchn_pose.tgtTwist.angular.z = joy->axes[3] * 0.7;  // 1;
}

bool MachinePose_s::goPose(/*const geometry_msgs::Pose2D::ConstPtr& cmd_pose*/)
{
  // original PID feedback on error of Angle and Distance
  const double KPang = TURN_KP;  // 1.0;
  const double KDang = 0;
  const double KPdist = SPD_KP;  // 2.0;
  const double KDdist = 0;
  bool ret = false;

  double errorX = this->tgtPose.x - this->pos_vicon.x;
  double errorY = this->tgtPose.y - this->pos_vicon.y;
  double targetT = atan2(errorY, errorX);

  double theta = this->pos_vicon.theta;
  double errorNX = errorX * cos(-theta) - errorY * sin(-theta);
  // double errorNY = errorX * sin(-theta) +  errorY * cos(-theta);
  double errorNT = nomalizeAng(targetT - theta);

  if (this->tgtPose.x == 0.0 && this->tgtPose.y == 0.0)
  {  // mokutekiti
    errorNX /*= errorNY */ = errorNT = 0.0;
  }
  double tmp_spd = KPdist * errorNX;
  double tmp_turn = KPang * Rad2Deg(errorNT);
  tmp_spd = Limit(tmp_spd, 100, -100);
  tmp_turn = Limit(tmp_turn, 30, -30);
  double distance = sqrt(sqr(errorX) + sqr(errorY));
  printf("spd:%+8.2lf turn:%+4.1lf", tmp_spd, tmp_turn);
  if (distance <= ARV_DIST /* && 60>fabs(Rad2Deg(errorNT))*/)
  {
    this->tgtTwist.angular.z = 0;
    this->tgtTwist.linear.x = 0;
    return true;
  }
  else
  {
    this->tgtTwist.angular.z = Deg2Rad(tmp_turn);
    this->tgtTwist.linear.x = tmp_spd;
    return false;
  }
}

bool MachinePose_s::postPose()
{
  // @todo publish pose data
  /*    tms_msg_db::TmsdbStamped db_msg;
      tms_msg_db::Tmsdb tmpData;
      ros::Time now = ros::Time::now() + ros::Duration(9*60*60); // GMT +9
      db_msg.header.frame_id  = "/world";
      db_msg.header.stamp     = now;
      tmpData.time    = boost::posix_time::to_iso_extended_string(now.toBoost());
      tmpData.id      = 2007;                     //mimamorukun ID
      tmpData.x       = this->getCurrentPositionX();
      tmpData.y       = this->getCurrentPositionY();
      tmpData.z       = 0;
      tmpData.rr      = Rad2Deg(0.0);
      tmpData.rp      = Rad2Deg(0.0);
      tmpData.ry      = Rad2Deg(this->getCurrentTheta());
      tmpData.place   = 5001;
      tmpData.sensor  = 3501;     //kalman_filter
      tmpData.state   = 1;

      db_msg.tmsdb.push_back(tmpData);
      db_pub.publish(db_msg);
      printf("@postPose x:%lf y:%lf",this->getCurrentPositionX(),this->getCurrentPositionY());*/
  return true;
}

void *vicon_update(void *ptr)
{
  // ros::Rate r(30);
  ros::Rate r(ROS_RATE);
  while (ros::ok())
  {
    mchn_pose.updateVicon();
    if (1000 < mchn_pose.pos_vicon.x && 1000 < mchn_pose.pos_vicon.y)
    {
      pthread_mutex_lock(&mutex_update_kalman);
      mchn_pose.UpdatePosition(mchn_pose.pos_vicon, MachinePose_s::POSISION);
      pthread_mutex_unlock(&mutex_update_kalman);
    }
    r.sleep();
  }
}

void *odom_update(void *ptr)
{
  // ros::Rate r(30);
  ros::Rate r(ROS_RATE);
  while (ros::ok())
  {
    mchn_pose.updateOdom();
    pthread_mutex_lock(&mutex_update_kalman);
    mchn_pose.UpdatePosition(mchn_pose.vel_odom, MachinePose_s::VELOCITY);
    pthread_mutex_unlock(&mutex_update_kalman);
    r.sleep();
  }
}

int main(int argc, char **argv)
{
  ROS_INFO("wc_controller");
  ros::init(argc, argv, "wc_controller");
  ros::NodeHandle n;

  int Kp_, Ki_, Kd_;
  string s_Kp_, s_Ki_, s_Kd_;
  ros::NodeHandle nh_param("~");
  string tmp_ip;
  nh_param.param< string >("IP_ADDR", tmp_ip, "192.168.11.99");
  nh_param.param< int >("spin_Kp", Kp_, 4800);
  nh_param.param< int >("spin_Ki", Ki_, /*30*/ 100);
  nh_param.param< int >("spin_Kd", Kd_, 40000);
  nh_param.param< double >("spd_Kp", SPD_KP, 2.0);
  nh_param.param< double >("turn_Kp", TURN_KP, 1.0);
  nh_param.param< int >("arv_dist", ARV_DIST, 200);
  // acces like "mimamorukun_controller/spd_Kp"
  client_socket.init(tmp_ip, 54300);
  s_Kp_ = boost::lexical_cast< string >(Kp_);
  s_Ki_ = boost::lexical_cast< string >(Ki_);
  s_Kd_ = boost::lexical_cast< string >(Kd_);

  try
  {
    string reply;
    try
    {
      // koyuusinndou 3Hz at Kp = 8000
      // client_socket << "@CR1@CR2@SM1,1@SM2,1@PP1,50@PP2,50@PI1,100@PI2,100@PD1,10@PD2,10";
      client_socket << "@CR1@CR2@SM1,1@SM2,1@PP1," + s_Kp_ + "@PP2," + s_Kp_ + "@PI1," + s_Ki_ + "@PI2," + s_Ki_ +
                           "@PD1," + s_Kd_ + "@PD2," + s_Kd_;
      client_socket >> reply;
    }
    catch (SocketException &)
    {
    }
    cout << "Response:" << reply << "\n";
    ;
  }
  catch (SocketException &e)
  {
    cout << "Exception was caught:" << e.description() << "\n";
  }

  db_client = n.serviceClient< tms_msg_db::TmsdbGetData >("/tms_db_reader/dbreader");
  // db_pub    = nh2.advertise<tms_msg_db::TmsdbStamped> ("tms_db_data", 10);
  // ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1,
  // receiveCmdVel);
  ros::Subscriber cmd_vel_sub = n.subscribe< sensor_msgs::Joy >("/joy", 1, receiveJoy);
  // ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Pose2D>("/mkun_goal_pose", 1,
  // receiveGoalPose);
  ros::ServiceServer service = n.advertiseService("mkun_goal_pose", receiveGoalPose);
  /*    ros::Time current_time, last_time;
      current_time    = ros::Time::now();
      last_time       = ros::Time::now();*/

  mchn_pose.updateVicon();
  printf("initial val  x:%4.2lf y:%4.2lf th:%4.2lf\n\r", mchn_pose.pos_vicon.x, mchn_pose.pos_vicon.y,
         Rad2Deg(mchn_pose.pos_vicon.theta));
  mchn_pose.setCurrentPosition(mchn_pose.pos_vicon);

  if (pthread_create(&thread_vicon, NULL, vicon_update, NULL))
  {
    cout << "error creating thread." << endl;
    abort();
  }

  if (pthread_create(&thread_odom, NULL, odom_update, NULL))
  {
    cout << "error creating thread." << endl;
    abort();
  }

  ros::Rate r(ROS_RATE);
  while (n.ok())
  {
    spinWheel(/*joy_cmd_spd,joy_cmd_turn*/);

    //        mchn_pose.goPose();
    // mchn_pose.goPose2();

    ROS_INFO("x:%4.2lf y:%4.2lf th:%4.2lf", mchn_pose.pos_fusioned.x, mchn_pose.pos_fusioned.y,
             Rad2Deg(mchn_pose.pos_fusioned.theta));
    /*        ROS_INFO("x:%4.2lf y:%4.2lf th:%4.2lf",
                mchn_pose.pos_odom.x,
                  mchn_pose.pos_odom.y,
                Rad2Deg(mchn_pose.pos_odom.theta));*/
    /*        last_time = current_time;
            current_time = ros::Time::now();*/
    ros::spinOnce();
    mchn_pose.postPose();
    r.sleep();
  }
  return (0);
}

/*****************************************************************************************************/
geometry_msgs::Pose2D MachinePose_s::UpdatePosition(geometry_msgs::Pose2D tpose, MachinePose_s::InfoType Info)
{
  static int count = 0;
  static double posA[6][6];  // 位置の状態行列
  static double velA[6][6];  // 速度の状態行列
  static double posC[3][6];  // 位置の観測行列
  static double velC[3][6];  // 速度の観測行列
  // Kalman kalman(6,3,3); //状態、観測、入力変数。順にn,m,l

  if (kalman == NULL)
    return pos_fusioned;

  if (count == 0)
  {
    // 初期化 (システム雑音，観測雑音，積分時間)
    kalman->init(0.1, 0.1, 1.0);
    kalman->setX(0, pos_fusioned.x);
    kalman->setX(1, pos_fusioned.y);
    kalman->setX(2, pos_fusioned.theta);
    memset(posA, 0, sizeof(posA));
    memset(velA, 0, sizeof(velA));
    memset(posC, 0, sizeof(posC));
    memset(velC, 0, sizeof(velC));

    for (int i = 0; i < 6; i++)
      posA[i][i] = 1.0;
    for (int i = 0; i < 6; i++)
      velA[i][i] = 1.0;
    for (int i = 0; i < 3; i++)
      velA[i][i + 3] = 1.0;
    for (int i = 0; i < 3; i++)
      posC[i][i] = 1.0;
    for (int i = 0; i < 3; i++)
      velC[i][i + 3] = 1.0;
  }

  // 観測値のセット
  double obs[3] = {tpose.x, tpose.y, tpose.theta};
  double input[3] = {0, 0, 0};

  switch (Info)
  {
    case MachinePose_s::POSISION:
      // 位置を観測する場合
      kalman->setA((double *)&posA);
      kalman->setC((double *)&posC);
      break;
    case MachinePose_s::VELOCITY:
      // 速度を観測する場合
      kalman->setA((double *)&velA);
      kalman->setC((double *)&velC);
      break;
  }

  // カルマンフィルタの計算　→　答えは　getX(i) で得られる
  kalman->update(obs, input);

  pos_fusioned.x = kalman->getX(0);
  pos_fusioned.y = kalman->getX(1);
  pos_fusioned.theta = nomalizeAng(kalman->getX(2));
  vel_fusioned.x = kalman->getX(3);
  vel_fusioned.y = kalman->getX(4);
  vel_fusioned.theta = nomalizeAng(kalman->getX(5));
  // printf("f_vel_x:%f \n",vel_fusioned.x );
  count++;

  return pos_fusioned;
}

void MachinePose_s::setCurrentPosition(geometry_msgs::Pose2D pose)
{
  pos_fusioned = pose;
}

geometry_msgs::Pose2D MachinePose_s::getCurrentPosition()
{
  return pos_fusioned;
}

double MachinePose_s::getCurrentPositionX()
{
  return pos_fusioned.x;
}

double MachinePose_s::getCurrentPositionY()
{
  return pos_fusioned.y;
}

double MachinePose_s::getCurrentTheta()
{
  return pos_fusioned.theta;
}

double MachinePose_s::getCurrentVelocityX()
{
  return vel_fusioned.x;
}

double MachinePose_s::getCurrentVelocityY()
{
  return vel_fusioned.y;
}

double MachinePose_s::getCurrentOmega()
{
  return vel_fusioned.theta;
}

// bool MachinePose_s::goPose2(/*const geometry_msgs::Pose2D::ConstPtr& cmd_pose*/) {
//     /* Kanayama, Y.; Kimura, Y.; Miyazaki, F.; Noguchi, T.; ,
//     "A stable tracking control method for an autonomous mobile robot," Robotics and Automation,
//     1990.
//     Proceedings., 1990 IEEE International Conference on , vol., no., pp.384-389 vol.1, 13-18 May
//     1990 */
//
//     double dirVel = 250.0;
//     double dirOmega = 0.0;
//
//     double Kx = 0.0001;  // 0.01;
//     double Ky = 1.5e-6;
//     double Kt = 0.1;
//
//     double gain = 3.0;  // 0.1;
//
//     double targetX = this->tgtPose.x;
//     double targetY = this->tgtPose.y;
//     double errorX = targetX - this->getCurrentPositionX();
//     double errorY = targetY - this->getCurrentPositionY();
//     double targetT = atan2(errorY, errorX);
//     double theta = this->getCurrentTheta();
//
//     double errorNX = errorX * cos(theta) + errorY * sin(theta);
//     double errorNY = -errorX * sin(theta) + errorY * cos(theta);
//     double errorNT = targetT - theta;
//
//     double vel = sqrt(sqr(this->getCurrentVelocityX()) + sqr(this->getCurrentVelocityY()));
//     double omega = this->getCurrentOmega();
//     double mu1 = -Kx * errorNX;
//     double mu2 = -Ky * errorNY * dirVel - Kt * sin(errorNT);
//     double u1 = fabs(dirVel * cos(errorNT)) - mu1;
//     double u2 = dirOmega - mu2;
//
//     bool ret = false;
//
//     if (this->tgtPose.x == 0.0 && this->tgtPose.y == 0.0) {  // mokutekiti
//         u1 = u2 = 0;
//     }
//
//     double tmp_spd = u1 * gain;
//     double tmp_turn = Rad2Deg(u2) * gain;
//     tmp_spd = Limit(tmp_spd, 100, -100);
//     tmp_turn = Limit(tmp_turn, 30, -30);
//     double distance = sqrt(sqr(errorX) + sqr(errorY));
//     if (distance <= 200) {
//         tmp_spd = 0.0;
//     }
//     printf("spd:%+8.2lf turn:%+4.1lf", tmp_spd, tmp_turn);
//     printf("spd:%+8.2lf turn:%+4.1lf", tmp_spd, tmp_turn);
//     if (distance <= 200 /* && 60>fabs(Rad2Deg(errorNT))*/) {
//         this->tgtTwist.angular.z = 0;
//         this->tgtTwist.linear.x = 0;
//         return true;
//     } else {
//         this->tgtTwist.angular.z = Deg2Rad(tmp_turn);
//         this->tgtTwist.linear.x = tmp_spd;
//         return false;
//     }
// }
