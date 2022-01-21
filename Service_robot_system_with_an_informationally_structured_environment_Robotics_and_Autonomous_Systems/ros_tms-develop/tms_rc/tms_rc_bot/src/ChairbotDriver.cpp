#include <iostream>
#include <string>
#include <math.h> 

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include "ClientSocket.h"
#include "SocketException.h"

using namespace std;

const int ROS_RATE  = 10;
const int ENC_MAX   = 3932159;
const int SPEED_MAX = 32767;

double MAX_ROT_VEL    = 1.0;
double MAX_TRANS_VEL  = 0.5;
double DIST_PER_PULSE = 0.144 * 4.0; // mm par pulse
double WHEEL_DIST     = 544;
bool   PRINT_TF       = true;

string ODOM_TOPIC    = "odom";
string CMD_TOPIC     = "cmd_vel";

pthread_t thread_odom;
pthread_mutex_t mutex_socket = PTHREAD_MUTEX_INITIALIZER;

double MM2M(double mm) { return mm * 0.001; }
double M2MM(double M) { return M * 1000; }
double Dist2Pulse(double dist) { return dist / DIST_PER_PULSE; }
double Rad2Deg(double rad) { return rad * (180.0) / M_PI; }
double Limit(double val, double max, double min) {
  if      (max < val) return max;
  else if (val < min) return min;
  else                return val;
}

ClientSocket client_socket("", 54300);

class Chairbot {
 private:
 public:
  Chairbot() {
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 1.0;
    ENC_R_old = ENC_L_old;
  }
  ~Chairbot(){}
  void updateOdom();

  nav_msgs::Odometry odom;
  geometry_msgs::Twist tgtTwist;

  long int ENC_R_old;
  long int ENC_L_old;
} chairbot;

void Chairbot::updateOdom() {

  long int ENC_L = 0, ENC_R = 0;
  
  string reply;
  pthread_mutex_lock(&mutex_socket);
  client_socket << "@GP1";
  client_socket >> reply;
  // ROS_INFO_STREAM(reply);
  sscanf(reply.c_str(), "@GP1,%ld", &ENC_L);
  client_socket << "@GP2";
  client_socket >> reply;
  // ROS_INFO_STREAM(reply);
  sscanf(reply.c_str(), "@GP2,%ld", &ENC_R);
  pthread_mutex_unlock(&mutex_socket);

  double old_time = odom.header.stamp.toSec();
  odom.header.stamp = ros::Time::now();
  double delta_t = odom.header.stamp.toSec() - old_time;

  if (ENC_L > ENC_MAX / 2) ENC_L = ENC_L - (ENC_MAX + 1);
  if (ENC_R > ENC_MAX / 2) ENC_R = ENC_R - (ENC_MAX + 1);

  if (fabs(ENC_L - ENC_L_old) > ENC_MAX / 2) ENC_L_old -= ENC_MAX + 1;
  if (fabs(ENC_R - ENC_R_old) > ENC_MAX / 2) ENC_R_old -= ENC_MAX + 1;

  //エンコーダーの位置での前回からの移動距離dL_R,dL_Lを算出
  double dL_L = (double)(ENC_L - ENC_L_old) * (-DIST_PER_PULSE);
  double dL_R = (double)(ENC_R - ENC_R_old) * DIST_PER_PULSE;

  //角速度SIGMAを算出
  double POS_SIGMA = (dL_R - dL_L) / WHEEL_DIST;
  double dL = (dL_R + dL_L) * 0.50000;

  double dX = dL * cos(tf::getYaw(odom.pose.pose.orientation) + POS_SIGMA); // X,Yの前回からの移動量計算
  double dY = dL * sin(tf::getYaw(odom.pose.pose.orientation) + POS_SIGMA);
  ENC_R_old = ENC_R;  //前回のエンコーダーの値を記録
  ENC_L_old = ENC_L;
  odom.pose.pose.position.x += MM2M(dX);
  odom.pose.pose.position.y += MM2M(dY);

  tf::Quaternion q1;
  tf::quaternionMsgToTF(odom.pose.pose.orientation, q1);
  tf::Quaternion q2;
  tf::quaternionMsgToTF(tf::createQuaternionMsgFromYaw(POS_SIGMA), q2);
  q1 *= q2;
  tf::quaternionTFToMsg(q1.normalized(), odom.pose.pose.orientation);
  odom.twist.twist.linear.x = MM2M(dL) / delta_t;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = POS_SIGMA / delta_t;
  return;
}

void spinWheel() {

  double arg_speed = M2MM(Limit(chairbot.tgtTwist.linear.x, MAX_TRANS_VEL, -MAX_TRANS_VEL));
  double arg_theta = Limit(chairbot.tgtTwist.angular.z, MAX_ROT_VEL, -MAX_ROT_VEL);

  double val_L = -Dist2Pulse(arg_speed) + Dist2Pulse((WHEEL_DIST / 2.0) * arg_theta);
  double val_R =  Dist2Pulse(arg_speed) + Dist2Pulse((WHEEL_DIST / 2.0) * arg_theta);
  val_L = (int)(Limit(val_L, (double)SPEED_MAX, (double)-SPEED_MAX));
  val_R = (int)(Limit(val_R, (double)SPEED_MAX, (double)-SPEED_MAX));

  string cmd_L = boost::lexical_cast<string>(val_L);
  string cmd_R = boost::lexical_cast<string>(val_R);

  string reply;
  pthread_mutex_lock(&mutex_socket);
  client_socket << "@SS1," + cmd_L;
  client_socket >> reply;
  // ROS_INFO_STREAM(reply);
  client_socket << "@SS2," + cmd_R;
  client_socket >> reply;
  // ROS_INFO_STREAM(reply);
  pthread_mutex_unlock(&mutex_socket);
}

void pub_odom(){
  static ros::NodeHandle nh;
  static ros::Publisher pub = nh.advertise<nav_msgs::Odometry>(ODOM_TOPIC, 100);
  pub.publish(chairbot.odom);
}

void pub_tf(){
  static tf::TransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped ts;
  ts.header.frame_id = chairbot.odom.header.frame_id;
  ts.child_frame_id  = chairbot.odom.child_frame_id;
  ts.header.stamp    = ros::Time::now();
  ts.transform.translation.x = chairbot.odom.pose.pose.position.x;
  ts.transform.translation.y = chairbot.odom.pose.pose.position.y;
  ts.transform.translation.z = chairbot.odom.pose.pose.position.z;
  ts.transform.rotation      = chairbot.odom.pose.pose.orientation;
  broadcaster.sendTransform(ts);
}

void *odom_update(void *ptr) {
  ros::Rate r(ROS_RATE);
  chairbot.odom.header.stamp = ros::Time::now();
  while (ros::ok()) {
    chairbot.updateOdom();
    pub_odom();
    if (PRINT_TF) pub_tf();
    r.sleep();
  }
}

void receiveCmdVel(const geometry_msgs::Twist::ConstPtr &cmd_vel) {
  chairbot.tgtTwist = *cmd_vel;
}
void callback_radius(const std_msgs::Float64::ConstPtr &data) {
  //DIST_PER_PULSE = 2.0 * M_PI * (data->data) / 8.0;  
  DIST_PER_PULSE = M_PI * (data->data);
}
void callback_tread(const std_msgs::Float64::ConstPtr &data) {
  WHEEL_DIST = (data->data) * 1000;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "chairbot_driver");
  ros::NodeHandle n;
  ros::NodeHandle nh_param("~");

  int Kp_, Ki_, Kd_, tmp_port;
  double  wheel_radius, wheel_tread;
  string tmp_ip, tmp_odom, tmp_base;
  nh_param.param<string>("ip_address", tmp_ip, "192.168.11.99");
  nh_param.param<int>("port", tmp_port, 54300);
  nh_param.param<int>("spin_Kp", Kp_, 4800);
  nh_param.param<int>("spin_Ki", Ki_, 100);
  nh_param.param<int>("spin_Kd", Kd_, 40000);
  nh_param.param<string>("odom_frame_id", tmp_odom, "odom");
  nh_param.param<string>("base_frame_id", tmp_base, "base_footprint");
  nh_param.param<string>("odom_topic_name", ODOM_TOPIC, "odom");
  nh_param.param<string>("cmd_vel_topic", CMD_TOPIC, "cmd_vel");
  nh_param.param<double>("wheel_radius", wheel_radius, 0.184);
  nh_param.param<double>("wheel_tread", wheel_tread, 0.544);
  nh_param.param<double>("max_rot_vel", MAX_ROT_VEL, 1.0);
  nh_param.param<double>("max_trans_vel", MAX_TRANS_VEL, 0.5);
  nh_param.param<bool>("print_tf", PRINT_TF, true);

  string s_Kp_, s_Ki_, s_Kd_;
  s_Kp_ = boost::lexical_cast<string>(Kp_);
  s_Ki_ = boost::lexical_cast<string>(Ki_);
  s_Kd_ = boost::lexical_cast<string>(Kd_);

  chairbot.odom.header.frame_id = tmp_odom;
  chairbot.odom.child_frame_id  = tmp_base;

  //DIST_PER_PULSE = 2.0 * M_PI * wheel_radius / 8.0;
  DIST_PER_PULSE = M_PI * wheel_radius;
  WHEEL_DIST = wheel_tread * 1000;

  ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>(CMD_TOPIC, 10, receiveCmdVel);
  //ros::Subscriber wheel_radius_sub = n.subscribe<std_msgs::Float64>("wheel_radius", 1, callback_radius);
  //ros::Subscriber wheel_tread_sub  = n.subscribe<std_msgs::Float64>("wheel_tread", 1, callback_tread);

  client_socket.init(tmp_ip, tmp_port);

  string reply;
  client_socket << "@CR1";
  client_socket >> reply;
  ROS_INFO_STREAM(reply);
  client_socket << "@CR2";
  client_socket >> reply;
  ROS_INFO_STREAM(reply);
  client_socket << "@SM1,1";
  client_socket >> reply;
  ROS_INFO_STREAM(reply);
  client_socket << "@SM2,1";
  client_socket >> reply;
  ROS_INFO_STREAM(reply);
  client_socket << "@PP1," + s_Kp_;
  client_socket >> reply;
  ROS_INFO_STREAM(reply);
  client_socket << "@PP2," + s_Kp_;
  client_socket >> reply;
  ROS_INFO_STREAM(reply);
  client_socket << "@PI1," + s_Ki_;
  client_socket >> reply;
  ROS_INFO_STREAM(reply);
  client_socket << "@PI2," + s_Ki_;
  client_socket >> reply;
  ROS_INFO_STREAM(reply);
  client_socket << "@PD1," + s_Kd_;
  client_socket >> reply;
  ROS_INFO_STREAM(reply);
  client_socket << "@PD2," + s_Kd_;
  client_socket >> reply;
  ROS_INFO_STREAM(reply);

  if (pthread_create(&thread_odom, NULL, odom_update, NULL)) {
    ROS_ERROR_STREAM("Error creating thread : Could not create chairbot thread_odom");
    abort();
  }

  ros::Rate r(ROS_RATE);
  while (n.ok()) {
    spinWheel();
    ros::spinOnce();
    r.sleep();
  }
  return (0);
}