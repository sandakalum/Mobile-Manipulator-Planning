/*
 * kobuki_control.cpp
 *
 * @brief
 *   for control robot base : using robot's odometry
 *      input  : (goal_dis, goal_ang)
 *      output : result
 *   for control robot manipulator :
 *      input  : servos' angle(th0, th1, th2, th3)
 *      output : result
 *
 *  Created on: 2013/10/30
 *      Author: hashiguchi
 */

#include <ros/ros.h>

#include <turtlebot_control/control_arm.hpp>
#include <turtlebot_control/control_base.hpp>

#define ODOM
//#define VICON
//#define DB

#define rad2deg(x) ((x) * (180.0) / M_PI)
#define deg2rad(x) ((x)*M_PI / 180.0)
// make 8bit data
#define LSB(data) ((unsigned char)(data))
#define MSB(data) ((unsigned char)((unsigned int)(data) >> 8) & 0xFF)

ros::Publisher vel_pub;
ros::ServiceClient db_client;
// ros::Publisher robotInfo_pub;

double distance(double x0, double y0, double x1, double y1)
{
  return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
}

// calculate z-axis rotation from Odom_Quaternion(z, w) -180~+180
double quaternion_to_enler(double z, double w)
{
  double sqw, sqz;
  sqw = w * w;
  sqz = z * z;
  return atan2(2.0 * (z * w), (-sqz + sqw));
}

// forward speed linear[m/s] rotational_speed angular[rad/s]
void pub_vel(double linear, double angular)
{
  geometry_msgs::Twist vel;
  vel.linear.x = linear;
  vel.angular.z = angular;

  // publish the message
  vel_pub.publish(vel);
  return;
}

void vicon_sysCallback(const tms_msg_ss::vicon_data::ConstPtr &msg)
{
  if (msg->subjectName == "kobuki")
  {
    //大域変数を更新
    v_pos_x = msg->translation.x;
    v_pos_y = msg->translation.y;
    v_ori_th = msg->eulerXYZ[2];
  }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  double z, w;  // quaternion
  double temp_th;
  //大域変数を更新
  pos_x = msg->pose.pose.position.x + x;
  pos_y = msg->pose.pose.position.y + y;

  z = msg->pose.pose.orientation.z;
  w = msg->pose.pose.orientation.w;
  temp_th = quaternion_to_enler(z, w);
  ori_th = rad2deg(temp_th) + th;

  // 絶対座標におけるkobukiの位置
  // TMS_DBからロボットの起動位置を取ってもよい
  //    x = pos_x + 1.0;//m
  //    y = pos_y + 1.0;//m
  //    th = ori_th;//-180deg~180deg
  //
  //    geometry_msgs::Point point;
  //    point.x = x;
  //    point.y = y;
  //    point.z = th;
  //    robotInfo_pub.publish(point);
}

void control_base(double goal_dis, double goal_ang)
{
  //移動前のオドメトリ情報を格納
  double ini_pos_x, ini_pos_y, ini_ori_th;
  double var_pos_x, var_pos_y, var_ori_th;  // ini_th->quaternion to euler

// initialize variables
#ifdef ODOM
  ini_pos_x = pos_x;
  ini_pos_y = pos_y;
  ini_ori_th = ori_th;

  var_pos_x = pos_x;
  var_pos_y = pos_y;
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
  srv.request.tmsdb.id = 2005;  // ID : kobuki
  if (db_client.call(srv))
  {
    ini_pos_x = srv.response.tmsdb[0].x;
    ini_pos_y = srv.response.tmsdb[0].y;
    ini_ori_th = srv.response.tmsdb[0].ry;

    var_pos_x = srv.response.tmsdb[0].x;
    var_pos_y = srv.response.tmsdb[0].y;
    var_ori_th = srv.response.tmsdb[0].ry;
  }
  else
  {
    ROS_ERROR("Failed to call service tms db get kobuki's data\n");
  }
#endif

  if (goal_ang > 0)
  {  // rotate +
    while (1)
    {
      // 180度の境界線を越えるとき
      if (ini_ori_th + goal_ang > 180)
      {
        if ((-180 - ini_ori_th <= var_ori_th && var_ori_th < goal_ang - 360) ||
            (0 <= var_ori_th && var_ori_th <= 180 - goal_ang))
        {
          pub_vel(0.0, 0.523596);  // 30deg/s
#ifdef ODOM
          var_ori_th = ori_th;
#endif
#ifdef VICON
          var_ori_th = v_ori_th;
#endif
#ifdef DB
          if (db_client.call(srv))
          {
            var_ori_th = srv.response.tmsdb[0].ry;
          }
          else
          {
            ROS_ERROR("Failed to call service tms db get kobuki's data\n");
          }
#endif
        }
        else
        {
          pub_vel(0.0, 0.0);
          break;
        }
      }
      else
      {
        if (fabs(var_ori_th - ini_ori_th) < fabs(goal_ang))
        {
          // 1.+回転
          pub_vel(0.0, 0.523596);  // 30deg/s
#ifdef ODOM
          var_ori_th = ori_th;
#endif
#ifdef VICON
          var_ori_th = v_ori_th;
#endif
#ifdef DB
          if (db_client.call(srv))
          {
            var_ori_th = srv.response.tmsdb[0].ry;
          }
          else
          {
            ROS_ERROR("Failed to call service tms db get kobuki's data\n");
          }
#endif
        }
        else
        {
          pub_vel(0.0, 0.0);
          break;
        }
      }
    }
  }
  else
  {  // rotate -
    while (1)
    {
      // 180度の境界線を越えるとき
      if (ini_ori_th + goal_ang < -180)
      {
        if ((-180 - ini_ori_th <= var_ori_th && var_ori_th <= 0) ||
            (goal_ang + 360 < var_ori_th && var_ori_th <= 180 - ini_ori_th))
        {
          pub_vel(0.0, -0.523596);  //-(PI/2m)/s
#ifdef ODOM
          var_ori_th = ori_th;
#endif
#ifdef VICON
          var_ori_th = v_ori_th;
#endif
#ifdef DB
          if (db_client.call(srv))
          {
            var_ori_th = srv.response.tmsdb[0].ry;
          }
          else
          {
            ROS_ERROR("Failed to call service tms db get kobuki's data\n");
          }
#endif
        }
        else
        {
          pub_vel(0.0, 0.0);
          break;
        }
      }
      else
      {
        if (fabs(var_ori_th - ini_ori_th) < fabs(goal_ang))
        {
          // 1.+回転
          pub_vel(0.0, -0.523596);  // -(PI/2m)/s
#ifdef ODOM
          var_ori_th = ori_th;
#endif
#ifdef VICON
          var_ori_th = v_ori_th;
#endif
#ifdef DB
          if (db_client.call(srv))
          {
            var_ori_th = srv.response.tmsdb[0].ry;
          }
          else
          {
            ROS_ERROR("Failed to call service tms db get kobuki's data\n");
          }
#endif
        }
        else
        {
          pub_vel(0.0, 0.0);
          break;
        }
      }
    }
  }

  while (1)
  {
    if (distance(ini_pos_x, ini_pos_y, var_pos_x, var_pos_y) < goal_dis / 1000)
    {  // 単位：mm -> m
      // 2.直進
      pub_vel(0.2, 0.0);  // 0.25m/s
#ifdef ODOM
      var_pos_x = pos_x;
      var_pos_y = pos_y;
#endif
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
        ROS_ERROR("Failed to call service tms db get kobuki's data\n");
      }
#endif
    }
    else
    {
      pub_vel(0.0, 0.0);
      break;
    }
  }
}

void control_manipulator(unsigned int iPos0, unsigned int iPos1, unsigned int iPos2, unsigned int iPos3,
                         SendIJogPacket *stSendIJOGPacket)
{
  unsigned char ucDOF = 4;

  stSendIJOGPacket->ucHeader[0] = HEADER;
  stSendIJOGPacket->ucHeader[1] = HEADER;

  stSendIJOGPacket->ucPacketSize = MIN_PACKET_SIZE + CMD_I_JOG_STRUCT_SIZE * ucDOF;
  stSendIJOGPacket->ucChipID = BROADCAST_ID;
  stSendIJOGPacket->ucCmd = CMD_I_JOG;

  // set position (servo0)
  stSendIJOGPacket->ucData[0] = LSB(iPos0);
  stSendIJOGPacket->ucData[1] = MSB(iPos0);
  stSendIJOGPacket->ucData[2] = 0;
  stSendIJOGPacket->ucData[3] = 0;
  stSendIJOGPacket->ucData[4] = 200;

  // set position (servo1)
  stSendIJOGPacket->ucData[5] = LSB(iPos1);
  stSendIJOGPacket->ucData[6] = MSB(iPos1);
  stSendIJOGPacket->ucData[7] = 0;
  stSendIJOGPacket->ucData[8] = 1;
  stSendIJOGPacket->ucData[9] = 200;

  // set position (servo2)
  stSendIJOGPacket->ucData[10] = LSB(iPos2);
  stSendIJOGPacket->ucData[11] = MSB(iPos2);
  stSendIJOGPacket->ucData[12] = 0;
  stSendIJOGPacket->ucData[13] = 2;
  stSendIJOGPacket->ucData[14] = 200;

  // set position (servo3) gripper
  stSendIJOGPacket->ucData[15] = LSB(iPos3);
  stSendIJOGPacket->ucData[16] = MSB(iPos3);
  stSendIJOGPacket->ucData[17] = 0;
  stSendIJOGPacket->ucData[18] = 3;
  stSendIJOGPacket->ucData[19] = 200;

  // CheckSum
  stSendIJOGPacket->ucCheckSum1 = stSendIJOGPacket->ucPacketSize ^ stSendIJOGPacket->ucChipID ^ stSendIJOGPacket->ucCmd;
  for (unsigned char i = 0; i < CMD_I_JOG_STRUCT_SIZE * ucDOF; i++)
    stSendIJOGPacket->ucCheckSum1 ^= stSendIJOGPacket->ucData[i];

  stSendIJOGPacket->ucCheckSum2 = ~(stSendIJOGPacket->ucCheckSum1);
  stSendIJOGPacket->ucCheckSum1 &= CHKSUM_MASK;
  stSendIJOGPacket->ucCheckSum2 &= CHKSUM_MASK;

  ROS_INFO("Succeed to set msg from control_servo\n");
}

// bool getKobukiInfo(double *robot_x, double *robot_y, double *robot_th) {
//	ROS_INFO("In callback function to get kobuki info service.\n");
//	*robot_x = x;
//        ROS_INFO("assigned x to robot_x\n");
//	*robot_y = y;
//        ROS_INFO("assigned y to robot_y\n");
//	*robot_th = th;
//        ROS_INFO("assigned th to robot_th\n");
//	return true;
//}

bool callback(tms_msg_rc::kobuki_control_1::Request &req, tms_msg_rc::kobuki_control_1::Response &res)
{
  switch (req.cmd)
  {
    case 0:
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
      current_x = pos_x;
      current_y = pos_y;
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
      srv.request.tmsdb.id = 2005;  // ID : kobuki
      if (db_client.call(srv))
      {
        current_x = srv.response.tmsdb[0].x;
        current_y = srv.response.tmsdb[0].y;
        current_th = srv.response.tmsdb[0].ry;
      }
      else
      {
        ROS_ERROR("Failed to call service tms db get kobuki's data\n");
        return false;
      }
      ROS_INFO("current_x=%fmm, current_y=%fmm, current_th=%fdeg\n", current_x, current_y, current_th);
#endif
      double goal_distance, goal_theta;
      goal_distance = distance(current_x, current_y, req.arg[0], req.arg[1]);
      goal_theta = req.arg[2] - current_th;

      ROS_INFO("goal_dis=%fmm, goal_arg=%fdeg\n", goal_distance, goal_theta);
      control_base(goal_distance, goal_theta);
      res.result = 1;
      break;

    case 1:
      ///// control robot base2(相対座標)
      // エラーチェック
      if (req.arg.size() != 2 || req.arg[0] < 0 || req.arg[0] > 9179 || req.arg[1] < -180 || req.arg[1] > 180)
      {
        ROS_ERROR("case0 : An illegal arguments' type.\n");
        res.result = 0;  // false
        return true;
      }
      ROS_INFO("goal_dis=%fmm, goal_arg=%fdeg\n", req.arg[0], req.arg[1]);
      control_base(req.arg[0], req.arg[1]);
      res.result = 1;
      break;

    case 2:
      ///// control robot's manipulator
      // エラーチェック サーボ可動範囲チェックをRPでやる場合不要
      if (req.arg.size() != 4 || req.arg[0] < 308 || req.arg[0] > 715 || req.arg[1] < 120 || req.arg[1] > 900 ||
          req.arg[2] < 235 || req.arg[2] > 788)
      {
        ROS_ERROR("case1 : An illegal arguments' type.\n");
        res.result = 0;  // false
        return true;
      }
      control_manipulator(req.arg[0], req.arg[1], req.arg[2], req.arg[3], &stSendIJOGPacket);
      p = write(fd, &stSendIJOGPacket.ucHeader[0], stSendIJOGPacket.ucPacketSize);
      ros::Duration(3.0).sleep();
      res.result = 1;
      break;

    //	case 3:
    //		///// get kobuki info
    //		// エラーチェック
    //		if (req.arg.size() != 0) {
    //			ROS_ERROR("case2 : An illegal arguments' type.\n");
    //			res.result = 0;
    //			return true;}
    //		double rx, ry, rth;
    //		getKobukiInfo(&rx, &ry, &rth);
    //		res.val[0] = rx;
    //		res.val[1] = ry;
    //		res.val[2] = rth;
    //		res.result = 1;
    //		break;
    default:
      ROS_ERROR("An illegal command : %d\n", req.cmd);
      res.result = 0;
      return true;
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot_control");
  ros::NodeHandle n;
  //--------------モータの初期化------------------------------------------------
  struct termios newtio;

  // open device
  fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NONBLOCK);  // 192.168.4.168->ttyUSB0
  memset(&newtio, 0, sizeof(newtio));

  // control terminal
  newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;  // parity error is ignored
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;

  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);

  // make and send RAM_WRITE packet for servo0(torque on)
  stSendRWPacket.ucHeader[0] = HEADER;
  stSendRWPacket.ucHeader[1] = HEADER;
  stSendRWPacket.ucPacketSize = MIN_PACKET_SIZE + 3;
  stSendRWPacket.ucChipID = 0;
  stSendRWPacket.ucCmd = CMD_RAM_WRITE;
  stSendRWPacket.ucAddress = 52;
  stSendRWPacket.ucLen = 1;
  stSendRWPacket.ucData[0] = 0x60;

  // CheckSum
  stSendRWPacket.ucCheckSum1 = stSendRWPacket.ucPacketSize ^ stSendRWPacket.ucChipID ^ stSendRWPacket.ucCmd ^
                               stSendRWPacket.ucAddress ^ stSendRWPacket.ucLen;

  for (unsigned char i = 0; i < stSendRWPacket.ucLen; i++)
    stSendRWPacket.ucCheckSum1 ^= stSendRWPacket.ucData[i];

  stSendRWPacket.ucCheckSum2 = ~(stSendRWPacket.ucCheckSum1);
  stSendRWPacket.ucCheckSum1 &= CHKSUM_MASK;
  stSendRWPacket.ucCheckSum2 &= CHKSUM_MASK;

  p = write(fd, &stSendRWPacket.ucHeader[0], stSendRWPacket.ucPacketSize);
  //--------------------------------------------------------------------------

  // make and send RAM_WRITE packet for servo1(torque on)
  stSendRWPacket.ucHeader[0] = HEADER;
  stSendRWPacket.ucHeader[1] = HEADER;
  stSendRWPacket.ucPacketSize = MIN_PACKET_SIZE + 3;
  stSendRWPacket.ucChipID = 1;
  stSendRWPacket.ucCmd = CMD_RAM_WRITE;
  stSendRWPacket.ucAddress = 52;
  stSendRWPacket.ucLen = 1;
  stSendRWPacket.ucData[0] = 0x60;

  // CheckSum
  stSendRWPacket.ucCheckSum1 = stSendRWPacket.ucPacketSize ^ stSendRWPacket.ucChipID ^ stSendRWPacket.ucCmd ^
                               stSendRWPacket.ucAddress ^ stSendRWPacket.ucLen;

  for (unsigned char i = 0; i < stSendRWPacket.ucLen; i++)
    stSendRWPacket.ucCheckSum1 ^= stSendRWPacket.ucData[i];

  stSendRWPacket.ucCheckSum2 = ~(stSendRWPacket.ucCheckSum1);
  stSendRWPacket.ucCheckSum1 &= CHKSUM_MASK;
  stSendRWPacket.ucCheckSum2 &= CHKSUM_MASK;

  p = write(fd, &stSendRWPacket.ucHeader[0], stSendRWPacket.ucPacketSize);
  //--------------------------------------------------------------------------

  // make and send RAM_WRITE packet for servo2(torque on)
  stSendRWPacket.ucHeader[0] = HEADER;
  stSendRWPacket.ucHeader[1] = HEADER;
  stSendRWPacket.ucPacketSize = MIN_PACKET_SIZE + 3;
  stSendRWPacket.ucChipID = 2;
  stSendRWPacket.ucCmd = CMD_RAM_WRITE;
  stSendRWPacket.ucAddress = 52;
  stSendRWPacket.ucLen = 1;
  stSendRWPacket.ucData[0] = 0x60;

  // CheckSum
  stSendRWPacket.ucCheckSum1 = stSendRWPacket.ucPacketSize ^ stSendRWPacket.ucChipID ^ stSendRWPacket.ucCmd ^
                               stSendRWPacket.ucAddress ^ stSendRWPacket.ucLen;

  for (unsigned char i = 0; i < stSendRWPacket.ucLen; i++)
    stSendRWPacket.ucCheckSum1 ^= stSendRWPacket.ucData[i];

  stSendRWPacket.ucCheckSum2 = ~(stSendRWPacket.ucCheckSum1);
  stSendRWPacket.ucCheckSum1 &= CHKSUM_MASK;
  stSendRWPacket.ucCheckSum2 &= CHKSUM_MASK;

  p = write(fd, &stSendRWPacket.ucHeader[0], stSendRWPacket.ucPacketSize);
  //--------------------------------------------------------------------------
  // make and send RAM_WRITE packet for servo3(torque on)
  stSendRWPacket.ucHeader[0] = HEADER;
  stSendRWPacket.ucHeader[1] = HEADER;
  stSendRWPacket.ucPacketSize = MIN_PACKET_SIZE + 3;
  stSendRWPacket.ucChipID = 3;
  stSendRWPacket.ucCmd = CMD_RAM_WRITE;
  stSendRWPacket.ucAddress = 52;
  stSendRWPacket.ucLen = 1;
  stSendRWPacket.ucData[0] = 0x60;

  // CheckSum
  stSendRWPacket.ucCheckSum1 = stSendRWPacket.ucPacketSize ^ stSendRWPacket.ucChipID ^ stSendRWPacket.ucCmd ^
                               stSendRWPacket.ucAddress ^ stSendRWPacket.ucLen;

  for (unsigned char i = 0; i < stSendRWPacket.ucLen; i++)
    stSendRWPacket.ucCheckSum1 ^= stSendRWPacket.ucData[i];

  stSendRWPacket.ucCheckSum2 = ~(stSendRWPacket.ucCheckSum1);
  stSendRWPacket.ucCheckSum1 &= CHKSUM_MASK;
  stSendRWPacket.ucCheckSum2 &= CHKSUM_MASK;

  p = write(fd, &stSendRWPacket.ucHeader[0], stSendRWPacket.ucPacketSize);
  //------------------------------------------------------------------------------

  db_client = n.serviceClient< tms_msg_db::TmsdbGetData >("/tms_db_reader/dbreader");

#ifdef ODOM
  // kobukiの初期位置を格納
  tms_msg_db::TmsdbGetData srv;
  srv.request.tmsdb.id = 2005;  // ID : kobuki
  if (db_client.call(srv))
  {
    x = srv.response.tmsdb[0].x;
    y = srv.response.tmsdb[0].y;
    th = srv.response.tmsdb[0].ry;
  }
  else
  {
    ROS_ERROR("Failed to call service tms db get kobuki's data\n");
    return false;
  }
  ROS_INFO("kobuki's initial pos = [%fmm, %fmm, %fdeg]\n", x, y, th);
#endif

  ros::ServiceServer service = n.advertiseService("turtlebot_control", callback);
  // ros::ServiceServer robotInfo_srv = n.advertiseService("kobuki_info", getKobukiInfo);
  ROS_INFO("Ready to activate kobuki_control_node\n");

  ros::AsyncSpinner spinner(3);
  spinner.start();

  //現在地取得方法(vicon or odom)
  ros::Subscriber vicon_sub = n.subscribe("output", 10, vicon_sysCallback);
  ros::Subscriber odom_sub = n.subscribe("odom", 10, odomCallback);  // Odometry情報取得

  vel_pub = n.advertise< geometry_msgs::Twist >("mobile_base/commands/velocity", 1);  // kobukiに速度を送る
  // robotInfo_pub = n.advertise<geometry_msgs::Point>("kobuki_info", 1);
  ros::waitForShutdown();

  return 0;
}
