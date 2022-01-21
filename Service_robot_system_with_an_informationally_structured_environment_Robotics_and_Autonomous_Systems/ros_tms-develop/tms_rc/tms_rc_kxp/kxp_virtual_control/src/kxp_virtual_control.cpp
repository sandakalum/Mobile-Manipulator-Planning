/*
 * kxp_virtual_control.cpp
 *
 *  Created on: 2014/06/25
 *      Author: Yoonseok Pyo and Yuka Hashiguchi
 */
#include <ros/ros.h>
#include <unistd.h>

#include <tms_msg_rc/rc_robot_control.h>
#include <tms_msg_rc/tms_rc_ppose.h>
#include <tms_msg_rc/tms_rc_pmove.h>
#include <tms_msg_rc/katana_pos_array.h>

#include <tms_msg_db/TmsdbStamped.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sstream>
#include <boost/thread.hpp>

#define SUCCESS 1
#define FAILURE -1
#define SRV_UNIT_ERR -112
#define SRV_CMD_ERR -113

ros::Publisher pose_publisher;
// ロボット初期値----------------------------------------------------------------
double g_x = 2000.0;
double g_y = 500.0;
double g_t = 0.0;
double g_joint[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
double g_hL = 20.0;  // hand
double g_hR = 20.0;
double g_r_state = 1;

// robot model 2
double g_x2 = 2000.0;
double g_y2 = 500.0;
double g_t2 = 0.0;
double g_joint2[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
double g_hL2 = 20.0;  // hand
double g_hR2 = 20.0;
double g_r_state2 = 1;
//------------------------------------------------------------------------------
int g_oid;
double g_ox;
double g_oy;
double g_oz;
double g_orr;
double g_orp;
double g_ory;

double g_place;
double g_o_state = 1;

bool grasping = false;
//------------------------------------------------------------------------------
int8_t SyncObj(double r_x, double r_y, double r_ry, double r_state, double o_id, double o_x, double o_y, double o_z,
               double o_rr, double o_rp, double o_ry, double o_place, double o_state)
{
  bool ret = true;
  grasping = true;

  if (r_x != -1)
  {
    g_x = r_x;   // mm
    g_x2 = r_x;  // mm
  }
  if (r_y != -1)
  {
    g_y = r_y;   // mm
    g_y2 = r_y;  // mm
  }
  if (r_ry != -1)
  {
    g_t = r_ry;   // deg
    g_t2 = r_ry;  // deg
  }
  if (r_state != -1)
  {
    g_r_state = r_state;
    g_r_state2 = r_state;
  }

  if (o_id != -1)
    g_oid = (int)o_id;
  if (o_x != -1)
    g_ox = o_x;
  if (o_y != -1)
    g_oy = o_y;
  if (o_z != -1)
    g_oz = o_z;
  if (o_rr != -1)
    g_orr = o_rr;
  if (o_rp != -1)
    g_orp = o_rp;
  if (o_ry != -1)
    g_ory = o_ry;

  if (o_place != -1)
    g_place = o_place;
  if (o_state != -1)
    g_o_state = o_state;

  printf("SyncObj result: %0.1fmm, %0.1fmm, %0.1fdeg, %f\n ", g_x, g_y, g_t, g_r_state);
  printf("SyncObj result: %d, %0.1fmm, %0.1fmm, %0.1fmm, %0.1fdeg, %0.1fdeg, %0.1fdeg\n", g_oid, g_ox, g_oy, g_oz,
         g_orr, g_orp, g_ory);
  printf("SyncObj result: %0.1f, %0.1f\n ", g_place, g_o_state);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
  {
    grasping = false;
    return SUCCESS;
  }
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
int8_t CalcBackground(double r2_x, double r2_y, double r2_ry, double o_x, double o_y, double o_z, double o_rr,
                      double o_rp, double o_ry, double r2_wh, double r2_wl, double r2_j0, double r2_j1, double r2_j2,
                      double r2_j3, double r2_j4, double r2_j5, double r2_j6, double r2_gR)
{
  bool ret = true;
  grasping = true;

  g_x2 = r2_x;   // mm
  g_y2 = r2_y;   // mm
  g_t2 = r2_ry;  // deg

  if (o_x != -1)
    g_ox = o_x;
  if (o_y != -1)
    g_oy = o_y;
  //	if (o_z != -1) g_oz = o_z;
  if (o_rr != -1)
    g_orr = o_rr;
  if (o_rp != -1)
    g_orp = o_rp;
  if (o_ry != -1)
    g_ory = o_ry;

  //	if (r2_wh != -1) g_lumba_high2 = r2_wh;
  //	if (r2_wl != -1) g_lumba_low2 = r2_wl;
  //	if (r2_j0 != -1) g_joint2[0] = r2_j0;
  //	if (r2_j1 != -1) g_joint2[1] = r2_j1;
  //	if (r2_j2 != -1) g_joint2[2] = r2_j2;
  //	if (r2_j3 != -1) g_joint2[3] = r2_j3;
  //	if (r2_j4 != -1) g_joint2[4] = r2_j4;
  ////	if (r2_j5 != -1) g_joint2[5] = r2_j5;
  //	if (r2_j6 != -1) g_jR2[6] = r2_j6;

  //	if (r2_gR != -1) g_gripper_right2 = r2_gR;

  printf("CalcBackground result: %0.1fmm, %0.1fmm, %0.1fdeg\n ", g_x2, g_y2, g_t2);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
  {
    grasping = false;
    return SUCCESS;
  }
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
bool pSetOdom(tms_msg_rc::tms_rc_ppose::Request &req, tms_msg_rc::tms_rc_ppose::Response &res)
{
  bool ret = true;

  printf("pSetOdom(%0.1fmm, %0.1fmm, %0.1fdeg) result:", req.px, req.py, req.pth);
  ret ? printf("Success\n") : printf("Failure\n");

  return ret;
}

//------------------------------------------------------------------------------
bool pMove(tms_msg_rc::tms_rc_pmove::Request &req, tms_msg_rc::tms_rc_pmove::Response &res)
{
  bool ret = true;

  g_x = req.w_x;
  g_y = req.w_y;
  g_t = req.w_th;

  printf("pMoveLinearAbs(%0.1fmm, %0.1fmm, %0.1fdeg) result:", g_x, g_y, g_t);
  ret ? printf("Success\n") : printf("Failure\n");

  return ret;
}

//------------------------------------------------------------------------------
bool kMoveAngleArray(tms_msg_rc::katana_pos_array::Request &req, tms_msg_rc::katana_pos_array::Response &res)
{
  bool ret = true;

  for (int i = 0; i < req.pose_array.size(); i++)
  {
    if (req.pose_array[i].pose.size() == 6)
    {
      printf("armPose : ");
      for (int j = 0; j < 5; j++)
      {
        g_joint[j] = req.pose_array[i].pose[j];
        printf("%0.1f ", g_joint[j]);
      }
      switch ((int)req.pose_array[i].pose[5])
      {
        case 1:
          g_hL = 20.0;
          g_hR = 20.0;
          printf("%0.1f %0.1f\n", g_hL, g_hR);
          break;
        case 2:
          g_hL = -10.0;
          g_hR = -10.0;
          printf("%0.1f %0.1f\n", g_hL, g_hR);
          break;
        case 3:
          g_hL = -10.0;
          g_hR = -10.0;
          printf("%0.1f %0.1f\n", g_hL, g_hR);
          break;
        default:
          printf("Incorrect gripper command[%f]", req.pose_array[i].pose[6]);
          ret = false;
      }
    }
    else
    {
      printf("Incorrect joint size[%d]", (int)req.pose_array[i].pose.size());
      ret = false;
    }
  }
  return ret;
}

//------------------------------------------------------------------------------
bool robotControl(tms_msg_rc::rc_robot_control::Request &req, tms_msg_rc::rc_robot_control::Response &res)
{
  switch (req.unit)
  {
    //----------------------------------------------------------------------------
    case 0:  // all
      switch (req.cmd)
      {
        case 8:
          res.result = SyncObj(req.arg[0], req.arg[1], req.arg[2], req.arg[3], req.arg[4], req.arg[5], req.arg[6],
                               req.arg[7], req.arg[8], req.arg[9], req.arg[10], req.arg[11], req.arg[12]);
          break;
        case 9:
          res.result =
              CalcBackground(req.arg[0], req.arg[1], req.arg[2], req.arg[3], req.arg[4], req.arg[5], req.arg[6],
                             req.arg[7], req.arg[8], req.arg[9], req.arg[10], req.arg[11], req.arg[12], req.arg[13],
                             req.arg[14], req.arg[15], req.arg[16], req.arg[17], req.arg[18]);
          break;
        default:
          res.result = SRV_CMD_ERR;
          break;
      }
      break;

    //--------------------------------------------------------------------------
    default:
      res.result = SRV_UNIT_ERR;
      break;
  }
  return true;
}

//------------------------------------------------------------------------------
void RobotDataUpdate()
{                           // for kxp_1(2006) 計算用モデル
  ros::Rate loop_rate(10);  // 10Hz frequency (0.1 sec)

  while (ros::ok())
  {
    ros::Time now = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9
    double secs = now.toSec();
    // ROS_INFO("r1|Time:%f,x:%f,y:%f,ry:%f", secs,g_x,g_y,g_t);

    tms_msg_db::TmsdbStamped db_msg;
    tms_msg_db::Tmsdb current_pos_data;

    db_msg.header.frame_id = "/world";
    db_msg.header.stamp = now;

    current_pos_data.time = boost::posix_time::to_iso_extended_string(now.toBoost());
    current_pos_data.id = 2006;
    current_pos_data.x = g_x;
    current_pos_data.y = g_y;
    current_pos_data.z = 180;
    current_pos_data.rr = 0.0;
    current_pos_data.rp = 0.0;
    current_pos_data.ry = g_t;
    current_pos_data.offset_z = 180;
    current_pos_data.place = 5001;
    current_pos_data.sensor = 3005;
    current_pos_data.state = g_r_state;

    std::stringstream ss;
    for (int i = 0; i < 5; i++)
    {
      ss << g_joint[i] << ";";
    }
    ss << g_hL << ";" << g_hR;

    // joint information of kxp
    // joint[0]...[4], gripper_left, gripper_right
    current_pos_data.joint = ss.str();

    db_msg.tmsdb.push_back(current_pos_data);
    pose_publisher.publish(db_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

//------------------------------------------------------------------------------
void RobotDataUpdate2()
{                           // for kxp2 表示用モデル
  ros::Rate loop_rate(10);  // 10Hz frequency (0.1 sec)

  while (ros::ok())
  {
    //	if (grasping == true) {
    // DBの主キーがtimeなので，RobotDataUpdataとかぶらないように人為的に0.0005秒プラス
    ros::Time now = ros::Time::now() + ros::Duration(9 * 60 * 60) + ros::Duration(0.0005);  // GMT +9
    double secs = now.toSec();
    // ROS_INFO("r2|Time:%f,x:%f,y:%f,ry:%f", secs,g_x2,g_y2,g_t2);

    tms_msg_db::TmsdbStamped db_msg;
    tms_msg_db::Tmsdb current_pos_data;

    db_msg.header.frame_id = "/world";
    db_msg.header.stamp = now;

    current_pos_data.time = boost::posix_time::to_iso_extended_string(now.toBoost());
    current_pos_data.id = 2011;
    current_pos_data.x = g_x2;
    current_pos_data.y = g_y2;
    current_pos_data.z = 180;
    current_pos_data.rr = 0.0;
    current_pos_data.rp = 0.0;
    current_pos_data.ry = g_t2;
    current_pos_data.offset_z = 180;
    current_pos_data.place = 5001;
    current_pos_data.sensor = 3005;
    current_pos_data.state = g_r_state2;

    std::stringstream ss;
    for (int i = 0; i < 5; i++)
    {
      ss << g_joint[i] << ";";
    }
    ss << g_hL << ";" << g_hR;

    current_pos_data.joint = ss.str();

    db_msg.tmsdb.push_back(current_pos_data);
    pose_publisher.publish(db_msg);

    ros::spinOnce();
    loop_rate.sleep();

    //		}
  }
}

//------------------------------------------------------------------------------
void ObjectDataUpdate()
{
  ros::Rate loop_rate(10);  // 10Hz frequency (0.1 sec)

  while (ros::ok())
  {
    if (grasping == true)
    {
      // ros::Time now = ros::Time::now() + ros::Duration(9*60*60) + ros::Duration(0.0005); // GMT +9
      ros::Time now = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9
      double secs = now.toSec();
      // ROS_INFO("o|Time:%f,x:%f,y:%f,z:%f,rr:%f,rp:%f,ry:%f\n", secs,g_ox,g_oy,g_oz,g_orr,g_orp,g_ory);

      tms_msg_db::TmsdbStamped db_msg;
      tms_msg_db::Tmsdb current_pos_data;

      db_msg.header.frame_id = "/world";
      db_msg.header.stamp = now;

      current_pos_data.time = boost::posix_time::to_iso_extended_string(now.toBoost());
      current_pos_data.id = g_oid;
      current_pos_data.x = g_ox;  // m -> mm
      current_pos_data.y = g_oy;
      current_pos_data.z = g_oz;
      current_pos_data.rr = g_orr;
      current_pos_data.rp = g_orp;
      current_pos_data.ry = g_ory;
      current_pos_data.place = 2006;
      current_pos_data.sensor = 3005;
      current_pos_data.probability = 1.0;
      current_pos_data.state = 2;

      db_msg.tmsdb.push_back(current_pos_data);

      pose_publisher.publish(db_msg);
    }
  }
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //--------------------------------------------------------------------------
  ros::init(argc, argv, "kxp_virtual_control");
  ros::NodeHandle nh;

  // kxp initialize
  printf("Virtual KXP initialization has been completed.\n\n");

  ros::AsyncSpinner spinner(4);
  pose_publisher = nh.advertise< tms_msg_db::TmsdbStamped >("tms_db_data", 10);

  // スレッド処理でロボット情報の更新と物体情報の更新を同時に行う
  boost::thread thr_rdu(&RobotDataUpdate);
  boost::thread thr_rdu2(&RobotDataUpdate2);
  boost::thread thr_odu(&ObjectDataUpdate);

  ros::ServiceServer service1 = nh.advertiseService("virtual_psetodom", pSetOdom);
  ros::ServiceServer service2 = nh.advertiseService("virtual_pmove", pMove);
  ros::ServiceServer service3 = nh.advertiseService("virtual_katana_move_angle_array", kMoveAngleArray);
  ros::ServiceServer service4 = nh.advertiseService("kxp_virtual_control", robotControl);
  spinner.start();
  ros::waitForShutdown();

  // スレッドの終了を待つ
  thr_rdu.join();
  thr_rdu2.join();
  thr_odu.join();

  return (0);
}
//------------------------------------------------------------------------------
