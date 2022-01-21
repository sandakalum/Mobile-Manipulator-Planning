#ifndef CONTROL_BASE_HPP_
#define CONTROL_BASE_HPP_

#include <tms_msg_rc/kobuki_control_1.h>

#include <cmath>
#include <math.h>
#include <tms_msg_ss/vicon_data.h>
#include <nav_msgs/Odometry.h>
#include <tms_msg_db/TmsdbGetData.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <stdio.h>

// kobukiの初期位置を保持するための大域変数
double x;
double y;
double th;

//現在地を更新するための大域変数(for odom)
double pos_x;
double pos_y;
double ori_th;
//現在地を更新するための大域変数(for vicon)
double v_pos_x;
double v_pos_y;
double v_ori_th;

double distance(double x0, double y0, double x1, double y1);
double quaternion_to_enler(double z, double w);

void pub_vel(double angular, double linear);
void vicon_sysCallback(const tms_msg_ss::vicon_data::ConstPtr &msg);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
void control_base(double goal_dis, double goal_ang);

bool getKobukiInfo(double *robot_x, double *robot_y, double *robot_th);
bool callback(tms_msg_rc::kobuki_control_1::Request &req, tms_msg_rc::kobuki_control_1::Response &res);

#endif /* CONTROL_BASE_HPP_ */
