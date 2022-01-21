/*
 * kobuki_control.h
 *
 *  Created on: 2015/01/16
 *      Author: hashiguchi
 */

#ifndef KOBUKI_CONTROL_H_
#define KOBUKI_CONTROL_H_

#include <tms_msg_rc/rc_robot_control.h>

#include <cmath>
#include <math.h>
#include <tms_msg_ss/vicon_data.h>
#include <nav_msgs/Odometry.h>
#include <tms_msg_db/TmsdbGetData.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <stdio.h>

// doubleの初期位置を保持するための大域変数
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
double v_pos_z;
double v_ori_th;
double roll;
double pitch;
double yaw;
double ini_pos_x, ini_pos_y, ini_ori_th;
double var_pos_x, var_pos_y, var_ori_th;  // ini_th->quaternion to euler

geometry_msgs::Quaternion v_rotation;

double distance(double x0, double y0, double x1, double y1);
double quaternion_to_enler(double z, double w);

void pub_vel(double angular, double linear);
void vicon_sysCallback(const tms_msg_ss::vicon_data::ConstPtr &msg);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

bool getKobukiInfo(double *robot_x, double *robot_y, double *robot_th);
bool callback(tms_msg_rc::rc_robot_control::Request &req, tms_msg_rc::rc_robot_control::Response &res);

#endif /* KOBUKI_CONTROL_H_ */
