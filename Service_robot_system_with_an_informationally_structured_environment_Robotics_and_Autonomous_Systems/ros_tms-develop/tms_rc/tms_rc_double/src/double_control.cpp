#include <ros/ros.h>

#include <double_control.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/algorithm/string.hpp>
#include <tms_msg_ss/vicon_data.h>
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

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>







#define ODOM
#define VICON
//#define POZYX
//#define DB

#ifdef VICON
  int sensor =3001;
#endif

#ifdef POZYX
  int sensor =3005;
  //TODO define sensor id of pozyx and modify it
#endif

#define rad2deg(x) ((x) * (180.0) / M_PI)
#define deg2rad(x) ((x)*M_PI / 180.0)

//ros::Publisher odom_pub;
ros::ServiceClient db_client;

double distance(double x0, double y0, double x1, double y1)
{
  return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
}

// 小数第N位切り捨て
double floor2(double input, int N)
{
  double output;

  output = input * pow(10.0, (N - 1));
  output = (double)(int)(output);

  return output * pow(10.0, 1 - N);
}

// calculate z-axis rotation from Odom_Quaternion(z, w) -180~+180
double quaternion_to_euler(double z, double w)
{
  double sqw, sqz;
  sqw = w * w;
  sqz = z * z;
  return atan2(2.0 * (z * w), (-sqz + sqw));
}

// forward speed linear[m/s] rotational_speed angular[rad/s]


void pub_tf()
{
  static tf::TransformBroadcaster broadcaster;
  geometry_msgs::Quaternion robot_quat;
  //robot_quat = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
  robot_quat = tf::createQuaternionMsgFromYaw(deg2rad(ori_th));
  geometry_msgs::TransformStamped ts;
  ts.header.frame_id = "map";
  ts.child_frame_id = "base_footprint";
  ts.header.stamp = ros::Time::now();
  ts.transform.translation.x = var_pos_x / 1000;
  ts.transform.translation.y = var_pos_y /1000; 
  ts.transform.translation.z = 0;//v_pos_z / 1000;
  // ts.transform.translation.x = pose.x();
  // ts.transform.translation.y = pose.y();
  // ts.transform.translation.z = 0.0;
  ts.transform.rotation = robot_quat;
#ifdef VICON
  ts.transform.rotation = v_rotation;
#endif
  broadcaster.sendTransform(ts);
}


void vicon_sysCallback(const tms_msg_ss::vicon_data::ConstPtr &msg)
{
  int32_t id =0;
	std::string name;
	std::stringstream ss;

	std::vector<std::string> v_name;
	v_name.clear();
	boost::split(v_name, msg->subjectName, boost::is_any_of("#"));

	if(v_name.size() == 2){
		ss << v_name.at(0);
		ss >>id;
		name = v_name.at(1).c_str();
	}


  if (id == 2012)//msg->subjectName == "double (double)")
  {
    //大域変数を更新
    v_pos_x = msg->translation.x;
    v_pos_y = msg->translation.y;
    v_pos_z = msg->translation.z;
    v_rotation = msg->rotation;//floor2(rad2deg(msg->eulerXYZ[2]), 1);

    var_pos_x = v_pos_x;
    var_pos_y = v_pos_y;

    std::cout << v_pos_x <<"," << v_pos_y << "," <<v_rotation <<std::endl;
  }
  pub_tf();
}
/*
void pub_odom()
{
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";

  geometry_msgs::Quaternion odom_quat;
  odom_quat = tf::createQuaternionMsgFromYaw(ori_th);

  odom.pose.pose.position.x = pos_x;
  odom.pose.pose.position.y = pos_y;
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = odom_quat;

  odom.child_frame_id = "base_footprint";
  //odom.twist.twist.linear.x = pos_x;

odom_pub.publish(odom);
}
*/


void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  double z, w;  // quaternion
  double temp_th;
  //大域変数を更新
  //pos_x = msg->pose.pose.position.x + x;
  //pos_y = msg->pose.pose.position.y + y;

  z = msg->pose.pose.orientation.z;
  w = msg->pose.pose.orientation.w;

  temp_th = quaternion_to_euler(z, w);
  temp_th = temp_th + th;
  ori_th = rad2deg(temp_th);
  //ori_th = rad2deg(temp_th) + th;

  //pub_odom();
}

// command 0: rotate only
// command 1: go straight only
// command 2: rotate and go straight
bool control_base(int command, double goal_dis, double goal_ang)
{
  //移動前のオドメトリ情報を格納


// initialize variables
#ifdef ODOM
  //ini_pos_x = pos_x;
  //ini_pos_y = pos_y;
  ini_ori_th = ori_th;

  //var_pos_x = pos_x;
  //var_pos_y = pos_y;
  var_ori_th = ori_th;
#endif

#ifdef VICON
  ini_pos_x = v_pos_x;
  ini_pos_y = v_pos_y;
  ini_ori_th = v_ori_th;

  var_pos_x = v_pos_x;
  var_pos_y = v_pos_y;
  var_ori_th = v_ori_th;
#endif


#ifdef DB
  tms_msg_db::TmsdbGetData srv;
  srv.request.tmsdb.id = 2012;  // ID : double
  srv.request.tmsdb.sensor = sensor;
  if (db_client.call(srv))
  {
    ini_pos_x = srv.response.tmsdb[0].x;
    ini_pos_y = srv.response.tmsdb[0].y;
    //ini_ori_th = srv.response.tmsdb[0].ry;

    var_pos_x = srv.response.tmsdb[0].x;
    var_pos_y = srv.response.tmsdb[0].y;
    //var_ori_th = srv.response.tmsdb[0].ry;

    //roll = srv.response.tmsdb[0].rr;
    //pitch = srv.response.tmsdb[0].rp;
    //yaw = srv.response.tmsdb[0].ry;
  }
  else
  {
    ROS_ERROR("Failed to call service tms db get double's data\n");
  }
#endif

  if (command == 0 || command == 2)
  {
    if (goal_ang > 0)
    {  // rotate +
      while (1)
      {
        // 180度の境界線を越えるとき
        if (ini_ori_th + goal_ang > 180)
        {
          if ((ini_ori_th <= var_ori_th && var_ori_th <= 180) ||
              (-180 <= var_ori_th && var_ori_th < ini_ori_th + goal_ang - 360))
          {
        //    pub_vel(0.0, 0.523596);  // 30deg/s
#ifdef ODOM
            var_ori_th = ori_th;
#endif
#ifdef VICON
            var_ori_th = v_ori_th;
#endif
/*
#ifdef DB
            if (db_client.call(srv))
            {
              var_ori_th = srv.response.tmsdb[0].ry;
            }
            else
            {
              ROS_ERROR("Failed to call service tms db get double's data\n");
            }
#endif
*/
          }
          else
          {
    //        pub_vel(0.0, 0.0);
            break;
          }
        }
        else
        {
          if (fabs(var_ori_th - ini_ori_th) < fabs(goal_ang))
          {
            // 1.+回転
    //        pub_vel(0.0, 0.523596);  // 30deg/s
#ifdef ODOM
            var_ori_th = ori_th;
#endif
#ifdef VICON
            var_ori_th = v_ori_th;
#endif
/*
#ifdef DB
            if (db_client.call(srv))
            {
              var_ori_th = srv.response.tmsdb[0].ry;
            }
            else
            {
              ROS_ERROR("Failed to call service tms db get double's data\n");
            }
#endif
*/
          }
          else
          {
    //        pub_vel(0.0, 0.0);
            break;
          }
        }
      }
    }
    else if (goal_ang < 0)
    {  // rotate -
      while (1)
      {
        // 180度の境界線を越えるとき
        if (ini_ori_th + goal_ang < -180)
        {
          if ((-180 <= var_ori_th && var_ori_th <= ini_ori_th) ||
              (360 + goal_ang + ini_ori_th < var_ori_th && var_ori_th <= 180))
          {
    //        pub_vel(0.0, -0.523596);  //-(PI/2m)/s
#ifdef ODOM
            var_ori_th = ori_th;
#endif
#ifdef VICON
            var_ori_th = v_ori_th;
#endif
/*
#ifdef DB
            if (db_client.call(srv))
            {
              var_ori_th = srv.response.tmsdb[0].ry;
            }
            else
            {
              ROS_ERROR("Failed to call service tms db get double's data\n");
            }
#endif
*/
          }
          else
          {
    //        pub_vel(0.0, 0.0);
            break;
          }
        }
        else
        {
          if (fabs(var_ori_th - ini_ori_th) < fabs(goal_ang))
          {
            // 1.+回転
      //      pub_vel(0.0, -0.523596);  // -(PI/2m)/s
#ifdef ODOM
            var_ori_th = ori_th;
#endif
#ifdef VICON
            var_ori_th = v_ori_th;
#endif
/*
#ifdef DB
            if (db_client.call(srv))
            {
              var_ori_th = srv.response.tmsdb[0].ry;
            }
            else
            {
              ROS_ERROR("Failed to call service tms db get double's data\n");
            }
#endif
*/
          }
          else
          {
    //        pub_vel(0.0, 0.0);
            break;
          }
        }
      }
    }
  }

  if (command == 1 || command == 2)
  {
    if (goal_dis > 0)
    {
      while (1)
      {
        if (distance(ini_pos_x, ini_pos_y, var_pos_x, var_pos_y) < goal_dis)
        {                      // 単位：mm
                               // 2.直進
  //        pub_vel(0.15, 0.0);  // 0.25m/s
/*
#ifdef ODOM
          var_pos_x = pos_x;
          var_pos_y = pos_y;
#endif
*/
#ifdef VICON
          var_pos_x = v_pos_x;
          var_pos_y = v_pos_y;
#endif

#ifdef DB
          if (db_client.call(srv))
          {
            var_pos_x = srv.response.tmsdb[0].x;
            var_pos_y = srv.response.tmsdb[0].y;
          }
          else
          {
            ROS_ERROR("Failed to call service tms db get double's data\n");
          }
#endif
        }
        else
        {
  //        pub_vel(0.0, 0.0);
          break;
        }
      }
    }
  }
  return true;
}

bool callback(tms_msg_rc::rc_robot_control::Request &req, tms_msg_rc::rc_robot_control::Response &res)
{
  int command = 0;
  switch (req.cmd)
  {
    case 0:
    {
      ///// control robot base1(絶対座標)
      // エラーチェック
      if (req.arg.size() != 3 || req.arg[0] < 0 || req.arg[0] > 8000 || req.arg[1] < 0 || req.arg[1] > 4500 ||
          req.arg[2] < -180 || req.arg[2] > 180)
      {
        ROS_ERROR("case0 : An illegal arguments' type.\n");
        res.result = 0;  // false
        return true;
      }

      // 現在地取得
      double current_x, current_y, current_th;
#ifdef ODOM
      //current_x = pos_x;
      //current_y = pos_y;
      current_th = ori_th;
      ROS_INFO("current_x=%fmm, current_y=%fmm, current_th=%fdeg\n", current_x, current_y, current_th);
#endif
#ifdef VICON
      current_x = v_pos_x;
      current_y = v_pos_y;
      current_th = v_ori_th;
      ROS_INFO("current_x=%fmm, current_y=%fmm, current_th=%fdeg\n", current_x, current_y, current_th);
#endif
#ifdef DB
      tms_msg_db::TmsdbGetData srv;
      srv.request.tmsdb.id = 2012;  // ID : double
      srv.request.tmsdb.sensor = sensor;
      if (db_client.call(srv))
      {
        current_x = srv.response.tmsdb[0].x;
        current_y = srv.response.tmsdb[0].y;
        //current_th = srv.response.tmsdb[0].ry;
      }
      else
      {
        ROS_ERROR("Failed to call service tms db get double's data\n");
        return false;
      }
      ROS_INFO("current_x=%fmm, current_y=%fmm, current_th=%fdeg\n", current_x, current_y, current_th);
#endif
      double goal_distance, goal_theta;
      goal_distance = distance(current_x, current_y, req.arg[0], req.arg[1]);
      goal_theta = req.arg[2] - current_th;
      if (goal_theta > 180.0)
        goal_theta = goal_theta - 360.0;
      else if (goal_theta < -180.0)
        goal_theta = goal_theta + 360.0;

      if (goal_distance >= 5 && goal_theta < 2)
        command = 1;
      else if (goal_distance >= 5 && goal_theta >= 2)
        command = 2;

      ROS_INFO("command:%d: goal_dis=%fmm, goal_arg=%fdeg", command, goal_distance, goal_theta);
      if (control_base(command, goal_distance, goal_theta))
      {
        ROS_INFO("Finish control_base");
      }
      res.result = 1;
      break;
    }
    case 1:
    {  ///// control robot base2(相対座標)
      // エラーチェック
      double current_x, current_y, current_th;
#ifdef VICON
      current_x = v_pos_x;
      current_y = v_pos_y;
      current_th = v_ori_th;
      ROS_INFO("current_x=%fmm, current_y=%fmm, current_th=%fdeg\n", current_x, current_y, current_th);
#endif
#ifdef DB
      tms_msg_db::TmsdbGetData srv;
      srv.request.tmsdb.id = 2012;  // ID : double
      srv.request.tmsdb.sensor = sensor;
      if (db_client.call(srv))
      {
        current_x = srv.response.tmsdb[0].x;
        current_y = srv.response.tmsdb[0].y;
        //current_th = srv.response.tmsdb[0].ry;
      }
      else
      {
        ROS_ERROR("Failed to call service tms db get double's data\n");
        return false;
      }
      ROS_INFO("current_x=%fmm, current_y=%fmm, current_th=%fdeg\n", current_x, current_y, current_th);
#endif
      double goal_distance, goal_theta;
      goal_distance = distance(current_x, current_y, req.arg[0], req.arg[1]);
      goal_theta = req.arg[2] - current_th;
      if (goal_theta > 180.0)
        goal_theta = goal_theta - 360.0;
      else if (goal_theta < -180.0)
        goal_theta = goal_theta + 360.0;


      if (req.arg.size() != 2 || req.arg[0] < 0 || req.arg[0] > 9179 || req.arg[1] < -180 || req.arg[1] > 180)
      {
        ROS_ERROR("case0 : An illegal arguments' type.\n");
        res.result = 0;  // false
        return true;
      }

      if (goal_distance >= 5 && goal_theta < 2)
        command = 1;
      else if (goal_distance >= 5 && goal_theta >= 2)
        command = 2;

      ROS_INFO("command:%d: goal_dis=%fmm, goal_arg=%fdeg", command, req.arg[0], req.arg[1]);
      if (control_base(command, (double)req.arg[0], (double)req.arg[1]))
      {
        ROS_INFO("Finish control_base");
      }
      res.result = 1;
      break;
    }
    default:
    {
      ROS_ERROR("An illegal command : %d\n", req.cmd);
      res.result = 0;
      return true;
    }
  }
  pub_tf();
  sleep(1.0);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "double_control");
  ros::NodeHandle n;
  //odom_pub  = n.advertise<nav_msgs::Odometry>("odom", 50);

  db_client = n.serviceClient< tms_msg_db::TmsdbGetData >("/tms_db_reader");

#ifdef DB
  // doubleの初期位置を格納
  tms_msg_db::TmsdbGetData srv;
  srv.request.tmsdb.id = 2012;  // ID : double
  srv.request.tmsdb.sensor = sensor;
  if (db_client.call(srv))
  {
    var_pos_x = srv.response.tmsdb[0].x;
    var_pos_y = srv.response.tmsdb[0].y;
    ori_th = srv.response.tmsdb[0].ry;

    roll = srv.response.tmsdb[0].rr;
    pitch = srv.response.tmsdb[0].rp;
    yaw = srv.response.tmsdb[0].ry;
    th = 0;
  }
  else
  {
    ROS_ERROR("Failed to call service tms db get double's data\n");
    return false;
  }
  ROS_INFO("Double's initial pos = [%fmm, %fmm, %fdeg]\n", var_pos_x, var_pos_y, var_ori_th);
#endif

  ros::ServiceServer service = n.advertiseService("tms_rc_double", callback);
  ROS_INFO("Ready to activate double_control_node\n");
  ros::AsyncSpinner spinner(3);
  spinner.start();

  //現在地取得方法(vicon or odom)
  ros::Subscriber vicon_sub = n.subscribe("vicon_stream/output", 10, vicon_sysCallback);
  ros::Subscriber odom_sub = n.subscribe("/tms_rc_double/room957/odom", 10, odomCallback);  // Odometry情報取得
  ros::Rate loop_rate(3);

  while(ros::ok()){
#ifdef DB
    // doubleの初期位置を格納
    tms_msg_db::TmsdbGetData srv;
    srv.request.tmsdb.id = 2012;  // ID : double
    srv.request.tmsdb.sensor = sensor;
    if (db_client.call(srv))
    {
      var_pos_x = srv.response.tmsdb[0].x;
      var_pos_y = srv.response.tmsdb[0].y;
      var_ori_th = srv.response.tmsdb[0].ry;

      roll = srv.response.tmsdb[0].rr;
      pitch = srv.response.tmsdb[0].rp;
      yaw = srv.response.tmsdb[0].ry;
    }
    else
    {
      ROS_ERROR("Failed to call service tms db get double's data\n");
      return false;
    }
 #endif

    pub_tf();
    loop_rate.sleep();
  }

//  vel_pub = n.advertise< geometry_msgs::Twist >("mobile_base/commands/velocity", 1);  // doubleに速度を送る
  ros::waitForShutdown();

  return 0;
}
