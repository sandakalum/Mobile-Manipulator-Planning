#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include "ClientSocket.h"
#include "SocketException.h"
#include <iostream>
#include <string>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <tms_msg_db/TmsdbGetData.h>
#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_db/Tmsdb.h>
#include "kalman-Ndof.hpp"

#define ROS_RATE 10

using namespace std;

ClientSocket client_socket("", 54300);
const int ENC_MAX = 3932159;
const int SPEED_MAX = 32767;
const float DIST_PER_PULSE = 0.552486;  // mm par pulse
const int WHEEL_DIST = 544;
bool is_publish_tf;
// const int WHEEL_DIST = 570;             // 533;

// long int ENC_L = 0;
// long int ENC_R = 0;

pthread_t thread_odom;
pthread_mutex_t mutex_socket = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_update_kalman = PTHREAD_MUTEX_INITIALIZER;
ros::ServiceClient db_client;

class MachinePose_s
{
private:
public:
  MachinePose_s()
  {
    ROS_DEBUG("In Mimamorukun Constructor");
    m_Odom.header.frame_id = "odom";
    m_Odom.child_frame_id = "base_footprint";
    m_Odom.pose.pose.position.x = 0.0;
    m_Odom.pose.pose.position.y = 0.0;
    m_Odom.pose.pose.position.z = 0.0;
    m_Odom.pose.pose.orientation.x = 0.0;
    m_Odom.pose.pose.orientation.y = 0.0;
    m_Odom.pose.pose.orientation.z = 0.0;
    m_Odom.pose.pose.orientation.w = 1.0;
  };
  ~MachinePose_s(){};
  void updateOdom();
  bool postPose();
  nav_msgs::Odometry m_Odom;
  geometry_msgs::Twist tgtTwist;

  void setCurrentPosition(geometry_msgs::Pose pose);
  geometry_msgs::Pose2D getCurrentPosition();

  Kalman *kalman;
  geometry_msgs::Pose2D pos_fusioned;
  geometry_msgs::Pose2D vel_fusioned;
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

geometry_msgs::Pose2D updateVicon()
{
  geometry_msgs::Pose2D ret;
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
      ret.x = srv.response.tmsdb[0].x;
      ret.y = srv.response.tmsdb[0].y;
      ret.theta = Deg2Rad(srv.response.tmsdb[0].ry);
    }
  }
  return ret;
}

void MachinePose_s::updateOdom()
{
  // update Encoder value
  m_Odom.header.stamp = ros::Time::now();
  long int ENC_L, ENC_R;
  long int tmpENC_L = 0;
  long int tmpENC_R = 0;
  string reply;
  pthread_mutex_lock(&mutex_socket);
  client_socket << "@GP1@GP2"; /*use 250ms for send and get reply*/
  client_socket >> reply;
  pthread_mutex_unlock(&mutex_socket);
  ROS_DEBUG_STREAM("@GP raw:" << reply);
  sscanf(reply.c_str(), "@GP1,%ld@GP2,%ld", &tmpENC_L, &tmpENC_R);
  // ROS_DEBUG_STREAM("tmpENC_L:" << tmpENC_L << "    tmpENC_R:" << tmpENC_R);
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

  // double dX = dL * cos(mchn_pose.pos_odom.theta + POS_SIGMA);  // X,Yの前回からの移動量計算
  // double dY = dL * sin(mchn_pose.pos_odom.theta + POS_SIGMA);
  double dX = dL * cos(tf::getYaw(m_Odom.pose.pose.orientation) + POS_SIGMA);  // X,Yの前回からの移動量計算
  double dY = dL * sin(tf::getYaw(m_Odom.pose.pose.orientation) + POS_SIGMA);
  ENC_R_old = ENC_R;  //前回のエンコーダーの値を記録
  ENC_L_old = ENC_L;

  m_Odom.pose.pose.position.x += MM2M(dX);
  m_Odom.pose.pose.position.y += MM2M(dY);
  tf::Quaternion q1;
  tf::quaternionMsgToTF(m_Odom.pose.pose.orientation, q1);
  tf::Quaternion q2;
  tf::quaternionMsgToTF(tf::createQuaternionMsgFromYaw(POS_SIGMA), q2);
  q1 *= q2;
  tf::quaternionTFToMsg(q1.normalized(), m_Odom.pose.pose.orientation);
  m_Odom.twist.twist.linear.x = MM2M(dL) * (double)ROS_RATE;
  m_Odom.twist.twist.linear.y = 0.0;
  m_Odom.twist.twist.angular.z = POS_SIGMA * (double)ROS_RATE;

  m_Odom.twist.covariance.at(0) = sqr(0.05 * m_Odom.twist.twist.linear.x);
  m_Odom.twist.covariance.at(7) = sqr(0.01);
  m_Odom.twist.covariance.at(14) = 1000000;
  m_Odom.twist.covariance.at(21) = 1000000;
  m_Odom.twist.covariance.at(28) = 1000000;
  m_Odom.twist.covariance.at(35) = sqr(0.05 * POS_SIGMA);
  m_Odom.pose.covariance.at(0) += sqr(0.05 * MM2M(dX));
  m_Odom.pose.covariance.at(7) += sqr(0.05 * MM2M(dY));
  m_Odom.pose.covariance.at(14) = 1000000;
  m_Odom.pose.covariance.at(21) = 1000000;
  m_Odom.pose.covariance.at(28) = 1000000;
  m_Odom.pose.covariance.at(35) += sqr(POS_SIGMA * 0.05);
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
  double arg_speed = M2MM(mchn_pose.tgtTwist.linear.x);
  double arg_theta = mchn_pose.tgtTwist.angular.z;
  // ROS_INFO("X:%4.2f   Theta:%4.2f",arg_speed,arg_theta);
  double val_L = -Dist2Pulse(arg_speed) + Dist2Pulse((WHEEL_DIST / 2) * arg_theta);
  double val_R = Dist2Pulse(arg_speed) + Dist2Pulse((WHEEL_DIST / 2) * arg_theta);
  val_L = (int)Limit(val_L, (double)SPEED_MAX, (double)-SPEED_MAX);
  val_R = (int)Limit(val_R, (double)SPEED_MAX, (double)-SPEED_MAX);
  ROS_DEBUG("val_L:%2.f   val_R:%2.f", val_L, val_R);

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

void receiveCmdVel(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
  // ROS_DEBUG("receive /cmd_vel");
  mchn_pose.tgtTwist = *cmd_vel;
  // spinWheel(/*cmd_vel->linear.x,cmd_vel->angular.z*/);
}

void pub_tf()
{
  static tf::TransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped ts;
  ts.header.frame_id = mchn_pose.m_Odom.header.frame_id;
  ts.child_frame_id = mchn_pose.m_Odom.child_frame_id;
  ts.header.stamp = ros::Time::now();
  ts.transform.translation.x = mchn_pose.m_Odom.pose.pose.position.x;
  ts.transform.translation.y = mchn_pose.m_Odom.pose.pose.position.y;
  ts.transform.translation.z = mchn_pose.m_Odom.pose.pose.position.z;
  // ts.transform.translation.x = pose.x();
  // ts.transform.translation.y = pose.y();
  // ts.transform.translation.z = 0.0;
  ts.transform.rotation = mchn_pose.m_Odom.pose.pose.orientation;
  broadcaster.sendTransform(ts);
}

void pub_odom()
{
  static ros::NodeHandle nh;
  static ros::Publisher pub = nh.advertise< nav_msgs::Odometry >("odom", 100);
  pub.publish(mchn_pose.m_Odom);
}

void *vicon_update(void *ptr)
{
  // ros::Rate r(30);
  ros::Rate r(ROS_RATE);
  while (ros::ok())
  {
    geometry_msgs::Pose2D pose_vicon = updateVicon();

    pthread_mutex_lock(&mutex_update_kalman);
    mchn_pose.UpdatePosition(pose_vicon, MachinePose_s::POSISION);
    pthread_mutex_unlock(&mutex_update_kalman);
    r.sleep();
  }
}

void *odom_update(void *ptr)
{
  ros::Rate r(ROS_RATE);
  while (ros::ok())
  {
    mchn_pose.updateOdom();

    double yaw = tf::getYaw(mchn_pose.m_Odom.pose.pose.orientation);
    double speed = mchn_pose.m_Odom.twist.twist.linear.x;
    geometry_msgs::Pose2D vel;
    vel.x = speed * cos(yaw);
    vel.y = speed * sin(yaw);
    vel.theta = mchn_pose.m_Odom.twist.twist.angular.z;

    pthread_mutex_lock(&mutex_update_kalman);
    // mchn_pose.UpdatePosition(mchn_pose.vel_odom, MachinePose_s::VELOCITY);
    mchn_pose.UpdatePosition(vel, MachinePose_s::VELOCITY);
    pthread_mutex_unlock(&mutex_update_kalman);

    pub_odom();
    r.sleep();
  }
}

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

  if (is_publish_tf)
  {
    pub_tf();
  }

  return pos_fusioned;
}

int main(int argc, char **argv)
{
  ROS_INFO("wc_controller");
  ros::init(argc, argv, "wc_controller");
  ros::NodeHandle n;

  int Kp_, Ki_, Kd_;
  string s_Kp_, s_Ki_, s_Kd_;
  ros::NodeHandle nh_param("~");

  ros::Subscriber cmd_vel_sub = n.subscribe< geometry_msgs::Twist >("cmd_vel", 10, receiveCmdVel);

  string tmp_ip;
  nh_param.param< string >("IP_ADDR", tmp_ip, "192.168.11.99");
  nh_param.param< int >("spin_Kp", Kp_, 4800);
  nh_param.param< int >("spin_Ki", Ki_, /*30*/ 100);
  nh_param.param< int >("spin_Kd", Kd_, 40000);
  // acces like "mimamorukun_controller/spd_Kp"
  client_socket.init(tmp_ip, 54300);
  s_Kp_ = boost::lexical_cast< string >(Kp_);
  s_Ki_ = boost::lexical_cast< string >(Ki_);
  s_Kd_ = boost::lexical_cast< string >(Kd_);
  nh_param.param< bool >("publish_tf", is_publish_tf, true);

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

  // mchn_pose.updateVicon();
  // printf("initial val  x:%4.2lf y:%4.2lf th:%4.2lf\n\r", mchn_pose.pos_odom.x,
  // mchn_pose.pos_odom.y,
  //        Rad2Deg(mchn_pose.pos_odom.theta));
  // printf("initial val  x:%4.2lf y:%4.2lf th:%4.2lf\n\r", mchn_pose.m_Odom.pose.pose.position.x,
  //        mchn_pose.m_Odom.pose.pose.position.y,
  //        Rad2Deg(tf::getYaw(mchn_pose.m_Odom.pose.pose.orientation)));
  // mchn_pose.setCurrentPosition(mchn_pose.pos_vicon);

  if (pthread_create(&thread_odom, NULL, odom_update, NULL))
  {
    cout << "error creating thread." << endl;
    abort();
  }

  ros::Rate r(ROS_RATE);
  while (n.ok())
  {
    spinWheel();
    ros::spinOnce();
    r.sleep();
  }
  return (0);
}

// void MachinePose_s::setCurrentPosition(geometry_msgs::Pose2D pose) { pos_odom = pose; }
void MachinePose_s::setCurrentPosition(geometry_msgs::Pose pose)
{
  m_Odom.pose.pose = pose;
  // pos_odom = pose;
}
