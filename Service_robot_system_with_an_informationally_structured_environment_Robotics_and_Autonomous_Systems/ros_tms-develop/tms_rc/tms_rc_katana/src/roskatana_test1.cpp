// include for ROS
#include "ros/ros.h"
#include <unistd.h>
#include <boost/date_time/local_time/local_time.hpp>

#include <tms_msg_rc/katana_pos.h>
#include <tms_msg_rc/katana_pos_array.h>
#include <tms_msg_rc/katana_pos_single.h>
#include <tms_msg_rc/gripper_action.h>

#include "kniBase.h"
//#include "kmlMotBase.h"
#include "common/MathHelperFunctions.h"
#include <iostream>
#include <cstdio>
#include <memory>
#include <vector>
#include <fstream>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include "common/Timer.h"

//////////////////////////////////////////////////////////////////////////////////
#ifdef WIN32
#include <conio.h>
#else  // LINUX
#include "keyboard.h"
#endif
#define LEFT false
#define RIGHT true
//////////////////////////////////////////////////////////////////////////////////
// Thread structs:
pthread_mutex_t mutex;

struct TPoint
{
  double X, Y, Z;
  double phi, theta, psi;
};

struct TCurrentMot
{
  int idx;
  bool running;
  bool dir;
};

struct Tpos
{
  static std::vector< int > x, y, z, u, v, w;
  static const int xArr[], yArr[], zArr[], uArr[], vArr[], wArr[];
};

// Katana obj.
std::auto_ptr< CLMBase > katana;

// std::vector<int> Foo::vec(array, array + sizeof(array)/sizeof(*array));
// positionen, hard-coded. Use values from file instead
const int Tpos::xArr[] = {30206, -23393, -3066, 14454, 30000, 30000};
const int Tpos::yArr[] = {24327, -7837, -16796, 5803, 30290, 31000};
const int Tpos::zArr[] = {24327, -7837, -16796, 5802, 30290, 10924};
const int Tpos::uArr[] = {5333, -13791, -9985, 11449, 30996, 12063};
const int Tpos::vArr[] = {-3799, -5703, -11676, 8210, 30995, 12063};
const int Tpos::wArr[] = {-3799, -5703, -11676, 8210, 30995, 30992};
std::vector< int > Tpos::x(xArr, xArr + sizeof(xArr) / sizeof(*xArr));
std::vector< int > Tpos::y(yArr, yArr + sizeof(yArr) / sizeof(*yArr));
std::vector< int > Tpos::z(zArr, zArr + sizeof(zArr) / sizeof(*zArr));
std::vector< int > Tpos::u(uArr, uArr + sizeof(uArr) / sizeof(*uArr));
std::vector< int > Tpos::v(vArr, vArr + sizeof(vArr) / sizeof(*vArr));
std::vector< int > Tpos::w(wArr, wArr + sizeof(wArr) / sizeof(*wArr));
std::vector< TPoint > points(0);
// void StartPointlistMovement();
// void StartProgram(int index);
pthread_t tid;
// void* RunProgram(void*);
pid_t threadPid;
int retVal = 0;
bool progRunning = false;
// const double PI = 3.14159265358979323846;

const double SafeDegreeMaxMot1 = 360;
const double SafeDegreeMinMot1 = 0;
const double closegripcom = 0;
const int hand_force = -30;

//関数名【 round() 】  丸め処理関数    小数点第1位でdoubleの正負の数を丸める。
int user_round(double val)
{
  double ret0;
  int ret;
  char buf[256] = {'¥0'};
  char *p;

  // ***** 丸めを行い文字列に変換 ***** */
  sprintf(buf, "%.*f", 1, val);
  // ***** 文字列から数値に再変換 ***** */
  ret0 = strtod(buf, &p);

  ret = (double)(int)ret;

  return ret0;
}
////////////////////////////////////////////////////////////////////////////////
int user_angle_rad2enc(CLMBase *robot, int index, double radian)
{
  // Init paramater の取得
  const TMotInit *param = robot->GetBase()->GetMOT()->arr[index].GetInitialParameters();

  double c;
  int d;

  c = (double)param->encoderOffset +
      (double)param->rotationDirection * ((param->angleOffset - radian) / (2 * M_PI)) * (double)param->encodersPerCycle;
  d = user_round(c);

  return c;
}
////////////////////////////////////////////////////////////////////////////////
double user_angle_deg2rad(double degree)
{
  double c;

  c = (degree * M_PI) / 180;
  return c;
}

bool katana_move_angle_array(tms_msg_rc::katana_pos_array::Request &req, tms_msg_rc::katana_pos_array::Response &res)
{
  //角度[6]配列(&req)→Enc値[6]配列(act_enc)→pose値[6]配列act_pose

  std::vector< std::vector< int > > act_enc;  // Encoder値
  std::vector< int > temp_enc;

  std::vector< std::vector< double > > act_pose;
  std::vector< double > temp_pose;

  unsigned int i = 0;  // for用カウンタ
  unsigned int j = 0;  // std::cout用カウンタ

  double actual_pose[6];
  //関節角度から関節Encoder値に変換
  for (i = 0; i < req.pose_array.size(); i++)
  {
    ROS_INFO("%f, %f, %f, %f, %f, %f, %f", req.pose_array[i].pose[0], req.pose_array[i].pose[1],
             req.pose_array[i].pose[2], req.pose_array[i].pose[3], req.pose_array[i].pose[4], req.pose_array[i].pose[5],
             req.pose_array[i].pose[6]);

    temp_enc.clear();

    actual_pose[0] = (double)req.pose_array[i].pose[0] + 180.0;
    actual_pose[1] = 90.0 - (double)req.pose_array[i].pose[1];
    actual_pose[2] = 180.0 - (double)req.pose_array[i].pose[2];
    actual_pose[3] = (double)req.pose_array[i].pose[3] + 180.0;
    actual_pose[4] = 180.0 - (double)req.pose_array[i].pose[4];
    actual_pose[5] = -1 * (double)(req.pose_array[i].pose[5] + req.pose_array[i].pose[6]);

    temp_enc.push_back(user_angle_rad2enc(katana.get(), 0, user_angle_deg2rad(actual_pose[0])));  // motor1
    temp_enc.push_back(user_angle_rad2enc(katana.get(), 1, user_angle_deg2rad(actual_pose[1])));  // motor2
    temp_enc.push_back(user_angle_rad2enc(katana.get(), 2, user_angle_deg2rad(actual_pose[2])));  // motor3
    temp_enc.push_back(user_angle_rad2enc(katana.get(), 3, user_angle_deg2rad(actual_pose[3])));  // motor4
    temp_enc.push_back(user_angle_rad2enc(katana.get(), 4, user_angle_deg2rad(actual_pose[4])));  // motor5
    temp_enc.push_back(user_angle_rad2enc(katana.get(), 5, user_angle_deg2rad(actual_pose[5])));  // motor6(=Gripper)

    for (j = 0; j < 6; j++)
      //			printf("actual:%fdeg \n",actual_pose[j]);
      printf("actual:%f, encoder:%d \n", actual_pose[j], temp_enc.at(j));
    printf("\n");

    //		ROS_INFO("movDegrees_%d", i);
    //		katana->movDegrees(0, actual_pose[0]);
    //		katana->movDegrees(1, actual_pose[1]);
    //		katana->movDegrees(2, actual_pose[2]);
    //		katana->movDegrees(3, actual_pose[3]);
    //		katana->movDegrees(4, actual_pose[4]);
    //		katana->movDegrees(5, actual_pose[5]);
    //		ROS_INFO("movDegrees%d", i);
    //		sleep(6);

    ROS_INFO("moveRobotToEnc_%d", i);
    katana->moveRobotToEnc(temp_enc, true);
    ROS_INFO("moveRobotToEnc%d", i);
    sleep(3);
    //		act_enc.push_back(temp_enc);
  }
  printf("\n");

  //関節Encoder値からロボット姿勢に変換
  //	for(i=0;i<act_enc.size();i++){
  //		ROS_INFO("Debug1");
  //		katana->getCoordinatesFromEncoders(temp_pose, act_enc[i]);	//motor1~6(Encoder) ->
  // RobotPose(x,y,z,phi,theta,psi)

  //		for(j=0;j<6;j++)
  //			printf("%lf ",temp_pose[j]);
  //		printf("\n");

  //		act_pose.push_back(temp_pose);
  //		temp_pose.clear();
  //	}
  //	printf("\n");

  //ロボット動作
  //	for(i=0;i<act_pose.size();i++){

  //		katana->moveRobotTo(act_pose[i],true);		//姿勢のパラメータ→逆運動学→Move

  //	}

  return true;
}

// for test (rosservice call / rqt)
//実際はkatana_move_angle_arrayを投げたほうが楽のはず
bool katana_move_angle(tms_msg_rc::katana_pos_single::Request &req, tms_msg_rc::katana_pos_single::Response &res)
{
  std::vector< int > temp_enc;
  std::vector< double > temp_pose;

  temp_enc.push_back(user_angle_rad2enc(katana.get(), 0, user_angle_deg2rad((double)req.pose[0])));  // motor1
  temp_enc.push_back(user_angle_rad2enc(katana.get(), 1, user_angle_deg2rad((double)req.pose[1])));  // motor1
  temp_enc.push_back(user_angle_rad2enc(katana.get(), 2, user_angle_deg2rad((double)req.pose[2])));  // motor1
  temp_enc.push_back(user_angle_rad2enc(katana.get(), 3, user_angle_deg2rad((double)req.pose[3])));  // motor1
  temp_enc.push_back(user_angle_rad2enc(katana.get(), 4, user_angle_deg2rad((double)req.pose[4])));  // motor1
  temp_enc.push_back(user_angle_rad2enc(katana.get(), 5, user_angle_deg2rad((double)req.pose[5])));  // motor1

  katana->getCoordinatesFromEncoders(temp_pose, temp_enc);

  katana->moveRobotTo(temp_pose, true);

  return true;
}

bool katana_move_pose_array(tms_msg_rc::katana_pos_array::Request &req, tms_msg_rc::katana_pos_array::Response &res)
{
  std::vector< double > temp_pose;

  for (unsigned int i = 0; i < req.pose_array.size(); i++)
  {
  }

  return true;
}

bool katana_move_enc(tms_msg_rc::katana_pos_single::Request &req, tms_msg_rc::katana_pos_single::Response &res)
{
  std::vector< int > temp_enc;

  temp_enc.push_back(user_angle_rad2enc(katana.get(), 0, user_angle_deg2rad((double)req.pose[0])));  // motor1
  temp_enc.push_back(user_angle_rad2enc(katana.get(), 1, user_angle_deg2rad((double)req.pose[1])));  // motor1
  temp_enc.push_back(user_angle_rad2enc(katana.get(), 2, user_angle_deg2rad((double)req.pose[2])));  // motor1
  temp_enc.push_back(user_angle_rad2enc(katana.get(), 3, user_angle_deg2rad((double)req.pose[3])));  // motor1
  temp_enc.push_back(user_angle_rad2enc(katana.get(), 4, user_angle_deg2rad((double)req.pose[4])));  // motor1
  temp_enc.push_back(user_angle_rad2enc(katana.get(), 5, user_angle_deg2rad((double)req.pose[5])));  // motor1

  katana->moveRobotToEnc(temp_enc, true);

  return true;
}

bool katana_move_motor_angle(tms_msg_rc::katana_pos_single::Request &req, tms_msg_rc::katana_pos_single::Response &res)
{
  katana->movDegrees((unsigned int)req.pose[0], req.pose[1]);

  return true;
}

bool katana_gripper_action(tms_msg_rc::gripper_action::Request &req, tms_msg_rc::gripper_action::Response &res)
{
  int hand_degree = -40;

  switch (req.gripper)
  {
    case (1):
      katana->openGripper();
      break;
    case (2):
      katana->closeGripper();
      break;
    case (3):  //力限界まで閉じる。物体を把持するときに利用する。
      while ((katana->getForce(6)) > hand_force)
      {
        hand_degree++;
        katana->movDegrees(5, hand_degree);
        sleep(2);

        std::cout << "getForce(6) = " << katana->getForce(6) << ", ";
        std::cout << "hand degree = " << hand_degree << std::endl;
        katana->freezeMotor(5);
        std::cout << "freeze" << std::endl;
        sleep(10);
        if (hand_degree > 15)
        {
          ROS_INFO("hand_degree is over 15");
          exit(1);
        }
      }
      break;

    default:
      ROS_INFO("error! [1:open gripper / 2:close gripper]");
      break;
  }

  return true;
}

////////////////////////////////////  main  ////////////////////////////////////
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ros_katana");
  ros::NodeHandle n;
  ROS_INFO("ros_katana : init");

  ros::ServiceServer service1 = n.advertiseService("katana_move_angle_array", katana_move_angle_array);
  ros::ServiceServer service2 = n.advertiseService("katana_move_pose_array", katana_move_pose_array);
  ros::ServiceServer service3 = n.advertiseService("katana_move_enc", katana_move_enc);
  ros::ServiceServer service4 = n.advertiseService("katana_move_motor_angle", katana_move_motor_angle);
  ros::ServiceServer service5 = n.advertiseService("katana_move_angle", katana_move_angle);
  ros::ServiceServer service6 = n.advertiseService("katana_gripper_action", katana_gripper_action);

  sleep(1);

  char dir[255];
  getcwd(dir, 255);
  std::cout << "Current Directory : " << dir << std::endl;

  // open device: a serial port is opened in this case
  std::auto_ptr< CCdlSocket > device;
  std::auto_ptr< CCplSerialCRC > protocol;

  try
  {
    int port = 5566;
    char *ip = "192.168.168.232";
    device.reset(new CCdlSocket(ip, port));

    std::cout << "-------------------------------------------" << std::endl;
    std::cout << "success:  port " << port << " open" << std::endl;
    std::cout << "-------------------------------------------" << std::endl;

    // init protocol:
    protocol.reset(new CCplSerialCRC());
    std::cout << "-------------------------------------------" << std::endl;
    std::cout << "success: protocol class instantiated" << std::endl;
    std::cout << "-------------------------------------------" << std::endl;
    protocol->init(device.get());  // fails if no response from Katana
    std::cout << "-------------------------------------------" << std::endl;
    std::cout << "success: communication with Katana initialized" << std::endl;
    std::cout << "-------------------------------------------" << std::endl;

    // init robot:
    katana.reset(new CLMBase());
    katana->create("./catkin_ws/src/ros_tms/tms_rc/tms_rc_katana/KNI_4.3.0/configfiles450/katana6M180_G.cfg",
                   protocol.get());
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
    return -1;
  }
  std::cout << "-------------------------------------------" << std::endl;
  std::cout << "success: katana initialized" << std::endl;
  std::cout << "-------------------------------------------" << std::endl;

  // DisplayHelp();

  // KatanaのInitializeここまで

  short counter = 0;
  bool loop = true;  //ループカウンタ trueなら操作続行
  int pos = 0;
  bool DispInRad = true;
  bool IsOff = false;
  bool useLinearMode = false;
  double RadToDeg = 1.0;
  std::vector< int > encoders(katana->getNumberOfMotors(), 0);

  const TKatMOT *motors;
  TCurrentMot mot[6];
  for (int i = 0; i < 6; i++)
  {
    mot[i].running = false;
    mot[i].idx = i;
    mot[i].dir = true;
    mot[i].dir = RIGHT;
  }
  bool stepMode = false;
  int stepSize = 100;
  bool runProgram = false;

  CSctBase *sensctrl = &katana->GetBase()->GetSCT()->arr[0];

  //グリッパー開放用
  bool closegrip = false;

  //設定変更・キャリブレーション
  katana->setMaximumLinearVelocity(30);  // set linear velocity to 30
  std::cout << "setMaximumLinearVelocity 30." << std::endl;
  katana->calibrate();  // caliblate katana
  std::cout << "Katana was calibrated." << std::endl;

  int hand_degree = -40;

  //	while((katana->getForce(6)) > hand_force){
  //		hand_degree++;
  //		katana->movDegrees(5,hand_degree);
  //		sleep(2);
  //
  //		std::cout<<"getForce(6) = "<<katana->getForce(6)<<", ";
  //		std::cout<<"hand degree = "<<hand_degree<<std::endl;
  //		katana->freezeMotor(5);
  //		std::cout<<"freeze"<<std::endl;
  //		Sleep(10000);
  //		if(hand_degree > 15){
  //			printf("hand_degree is over 15¥n");
  //			exit(1);
  //		}
  //	}

  // calibration後の姿勢の記録
  std::vector< int > enc_init;
  std::vector< double > pose_init;

  //直立姿勢の記録
  std::vector< int > enc_fst;
  std::vector< double > pose_fst;

  pose_init.resize(6);

  enc_init.push_back(katana->getMotorEncoders(0));
  enc_init.push_back(katana->getMotorEncoders(1));
  enc_init.push_back(katana->getMotorEncoders(2));
  enc_init.push_back(katana->getMotorEncoders(3));
  enc_init.push_back(katana->getMotorEncoders(4));
  enc_init.push_back(katana->getMotorEncoders(5));

  //	katana->getCoordinatesFromEncoders(pose_init,enc_init);
  katana->getCoordinates(pose_init[0], pose_init[1], pose_init[2], pose_init[3], pose_init[4], pose_init[5]);
  // calibration後の姿勢の記録ここまで

  ROS_INFO("Encoder Value init Motor 0 : %d", enc_init[0]);
  ROS_INFO("Encoder Value init Motor 1 : %d", enc_init[1]);
  ROS_INFO("Encoder Value init Motor 2 : %d", enc_init[2]);
  ROS_INFO("Encoder Value init Motor 3 : %d", enc_init[3]);
  ROS_INFO("Encoder Value init Motor 4 : %d", enc_init[4]);
  ROS_INFO("Encoder Value init Motor 5 : %d", enc_init[5]);

  ROS_INFO("Encoder degree init Motor 0 : %lf", pose_init[0]);
  ROS_INFO("Encoder degree init Motor 1 : %lf", pose_init[1]);
  ROS_INFO("Encoder degree init Motor 2 : %lf", pose_init[2]);
  ROS_INFO("Encoder degree init Motor 3 : %lf", pose_init[3]);
  ROS_INFO("Encoder degree init Motor 4 : %lf", pose_init[4]);
  ROS_INFO("Encoder degree init Motor 5 : %lf", pose_init[5]);

  // Motor1 to 180 degree (front)
  //	katana->movDegrees(0, 180);
  //	sleep(5);

  goto SPIN;

  //	// get actual position
  //	double x1, y1, z1, phi1, theta1, psi1;
  //	katana->getCoordinates(x1, y1, z1, phi1, theta1, psi1);
  //	// create second position
  //	double x2 = x1 + 100.0;
  //	double y2 = y1 - 50.0;
  //	double z2 = z1 + 80.0;
  //	double phi2 = phi1;
  //	double theta2 = theta1;
  //	double psi2 = psi1;
  //	// move to position 2
  //	katana->moveRobotTo(x2, y2, z2, phi2, theta2, psi2);
  //	sleep(7);
  //	// move linear back to position 1
  //	katana->moveRobotTo(x1, y1, z1, phi1, theta1, psi1);
  //	sleep(7);
  //	katana->moveRobotTo(pose_init);

  //	katana->movDegrees(1, 90);
  //	katana->movDegrees(2, 180);
  //	katana->movDegrees(3, 180);
  //	katana->movDegrees(4, 180);
  //	katana->movDegrees(5, -90);
  //	sleep(5);
  //
  //	ROS_INFO("Encoder Value fst Motor 0 : %d", katana->getMotorEncoders(0));
  //	ROS_INFO("Encoder Value fst Motor 1 : %d", katana->getMotorEncoders(1));
  //	ROS_INFO("Encoder Value fst Motor 2 : %d", katana->getMotorEncoders(2));
  //	ROS_INFO("Encoder Value fst Motor 3 : %d", katana->getMotorEncoders(3));
  //	ROS_INFO("Encoder Value fst Motor 4 : %d", katana->getMotorEncoders(4));
  //	ROS_INFO("Encoder Value fst Motor 5 : %d", katana->getMotorEncoders(5));

  //	exit(1);

  //回転命令後5秒停止
  //	sleep(5);

  //	std::cout<<"initEnc = " << user_angle_rad2enc(katana.get(),0, user_angle_deg2rad(270)) <<std::endl;
  //	katana->moveMotorToEnc(0, user_angle_rad2enc(katana.get(),0, user_angle_deg2rad(270)));
  ////モーター0を角度45度へEnc値で動かすテスト

  //初期姿勢(直立状態)へ
  //	katana->movDegrees(1, 90);
  //	katana->movDegrees(2, 180);
  //	katana->movDegrees(3, 180);
  //	katana->movDegrees(4, 180);

  //初期姿勢(直立状態)へ

  //	enc_fst.push_back(6346);	enc_fst.push_back(-10092);	enc_fst.push_back(-26080);
  //	enc_fst.push_back(11587);	enc_fst.push_back(6609);	enc_fst.push_back(18271);

  enc_fst.push_back(user_angle_rad2enc(katana.get(), 0, user_angle_deg2rad(180)));
  enc_fst.push_back(user_angle_rad2enc(katana.get(), 1, user_angle_deg2rad(45)));
  enc_fst.push_back(user_angle_rad2enc(katana.get(), 2, user_angle_deg2rad(90)));
  enc_fst.push_back(user_angle_rad2enc(katana.get(), 3, user_angle_deg2rad(200)));
  enc_fst.push_back(user_angle_rad2enc(katana.get(), 4, user_angle_deg2rad(180)));
  enc_fst.push_back(user_angle_rad2enc(katana.get(), 5, user_angle_deg2rad(-30)));

  katana->moveRobotToEnc(enc_fst, true);
  exit(1);

  katana->getCoordinatesFromEncoders(pose_fst, enc_fst);

  katana->moveRobotTo(pose_fst);
  sleep(5);
  katana->movDegrees(0, 90);
  sleep(5);

  enc_fst.push_back(user_angle_rad2enc(katana.get(), 0, user_angle_deg2rad(180)));
  enc_fst.push_back(user_angle_rad2enc(katana.get(), 1, user_angle_deg2rad(90)));
  enc_fst.push_back(user_angle_rad2enc(katana.get(), 2, user_angle_deg2rad(180)));
  enc_fst.push_back(user_angle_rad2enc(katana.get(), 3, user_angle_deg2rad(180)));
  enc_fst.push_back(user_angle_rad2enc(katana.get(), 4, user_angle_deg2rad(180)));
  enc_fst.push_back(user_angle_rad2enc(katana.get(), 5, user_angle_deg2rad(-90)));

  ROS_INFO("Encoder Value fst Motor 0 : %d", enc_fst[0]);
  ROS_INFO("Encoder Value fst Motor 1 : %d", enc_fst[1]);
  ROS_INFO("Encoder Value fst Motor 2 : %d", enc_fst[2]);
  ROS_INFO("Encoder Value fst Motor 3 : %d", enc_fst[3]);
  ROS_INFO("Encoder Value fst Motor 4 : %d", enc_fst[4]);
  ROS_INFO("Encoder Value fst Motor 5 : %d", enc_fst[5]);

  katana->moveRobotToEnc(enc_fst, true);
  sleep(10);

  katana->moveRobotTo(pose_init);

  exit(1);

  katana->getCoordinatesFromEncoders(pose_fst, enc_fst);

  ROS_INFO("Encoder pose fst Motor 0 : %lf", pose_fst[0]);
  ROS_INFO("Encoder pose fst Motor 1 : %lf", pose_fst[1]);
  ROS_INFO("Encoder pose fst Motor 2 : %lf", pose_fst[2]);
  ROS_INFO("Encoder pose fst Motor 3 : %lf", pose_fst[3]);
  ROS_INFO("Encoder pose fst Motor 4 : %lf", pose_fst[4]);
  ROS_INFO("Encoder pose fst Motor 5 : %lf", pose_fst[5]);

  katana->moveRobotTo(pose_fst);

  ROS_INFO("setting start position finished. Sleep this system for 1000 ms.");
  sleep(3);  // 3秒停止
  //初期姿勢移行　ここまで

  exit(1);

  /*

  ///////////////////////////////////////////////////////////////////
  //Gripper Action
  ///////////////////////////////////////////////////////////////////

  //	katana->closeGripper();

    int grip_number = 0;	//1回目の計測異常を読み飛ばす

    std::cout<<"first getForce(6) = "<<katana->getForce(6)<<", ";
    std::cout<<"hand degree = "<<hand_degree<<std::endl;

    while(((katana->getForce(6)) > hand_force)||(grip_number < 2)){
      grip_number++;
      hand_degree++;

      katana->movDegrees(5, hand_degree);
      Sleep(1000);

      std::cout<<"getForce(6) = "<<katana->getForce(6)<<", ";
      std::cout<<"hand degree = "<<hand_degree<<std::endl;
  //		katana->freezeMotor(5);
  //		std::cout<<"freeze"<<std::endl;
  //		Sleep(10000);
      if(hand_degree > 15){
        printf("hand_degree is over 15¥n");
        exit(1);
      }
    }

    Sleep(2000);

    katana->moveRobotTo(pose_fst);

  */
  katana->moveRobotTo(pose_init);

SPIN:
  ros::spin();

  return 0;
}
