// include for ARIA
#include <math.h>
#include <Aria.h>

// include for ROS
#include <ros/ros.h>
#include <unistd.h>
#include <boost/date_time/local_time/local_time.hpp>

#include <std_srvs/Empty.h>
//自分の位置座標を情報をメッセージとして投げる
#include <tms_msg_db/tmsdb_data.h>
#include <tms_msg_ss/fss_object_data.h>
#include <tms_msg_db/TmsdbGetData.h>

#include <tms_msg_rc/tms_rc_ppose.h>
#include <tms_msg_rc/tms_rc_pmove.h>
#include <tms_msg_rc/tms_rc_pparam.h>

#define POS_ABER 135

ros::ServiceClient get_data_client;

// connection class
class ConnHandler
{
public:
  // Constructor
  ConnHandler(ArRobot *robot);
  // Destructor, its just empty
  ~ConnHandler(void)
  {
  }
  // to be called if the connection was made
  void connected(void);
  // to call if the connection failed
  void connFail(void);
  // to be called if the connection was lost
  void disconnected(void);

protected:
  // robot pointer
  ArRobot *myRobot;
  // the functor callbacks
  ArFunctorC< ConnHandler > myConnectedCB;
  ArFunctorC< ConnHandler > myConnFailCB;
  ArFunctorC< ConnHandler > myDisconnectedCB;
};

ConnHandler::ConnHandler(ArRobot *robot)
  : myConnectedCB(this, &ConnHandler::connected)
  , myConnFailCB(this, &ConnHandler::connFail)
  , myDisconnectedCB(this, &ConnHandler::disconnected)

{
  myRobot = robot;
  myRobot->addConnectCB(&myConnectedCB, ArListPos::FIRST);
  myRobot->addFailedConnectCB(&myConnFailCB, ArListPos::FIRST);
  myRobot->addDisconnectNormallyCB(&myDisconnectedCB, ArListPos::FIRST);
  myRobot->addDisconnectOnErrorCB(&myDisconnectedCB, ArListPos::FIRST);
}

// just exit if the connection failed
void ConnHandler::connFail(void)
{
  printf("directMotionDemo connection handler: Failed to connect.\n");
  myRobot->stopRunning();
  Aria::shutdown();
  return;
}

// turn on motors, and off sonar, and off amigobot sounds, when connected
void ConnHandler::connected(void)
{
  printf("directMotionDemo connection handler: Connected\n");
  myRobot->comInt(ArCommands::SONAR, 0);
  myRobot->comInt(ArCommands::ENABLE, 1);
  myRobot->comInt(ArCommands::SOUNDTOG, 0);
}

// lost connection, so just exit
void ConnHandler::disconnected(void)
{
  printf("directMotionDemo connection handler: Lost connection, exiting program.\n");
  exit(0);
}

//ロボットのオブジェクトを生成(コールバック関数のために大域定義)
ArRobot robot;

//デバッグのためにposeを大域定義
ArPose pose;
///////////////////////

ros::Publisher publish1;

//ロボット内部のオドメトリ値をViconの値で変更
bool Vicon_Psetodom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  // getViconData
  tms_msg_db::TmsdbGetData getRobotData;
  getRobotData.request.tmsdb.id = 2006;      // KXP ID
  getRobotData.request.tmsdb.sensor = 3001;  // Vicon ID
  if (get_data_client.call(getRobotData))
  {
    ROS_INFO("Get info of robot ID: %d\n", getRobotData.request.tmsdb.id);
  }
  else
  {
    ROS_INFO("Failed to call service getRobotData ID: %d\n", getRobotData.request.tmsdb.id);
    return false;
  }
  if (!getRobotData.response.tmsdb.empty())
  {
    if (getRobotData.response.tmsdb[0].x != 0 && getRobotData.response.tmsdb[0].y != 0)
    {
      pose.setPose(0.0, 0.0, 0.0);  //関数突入時に大域変数pose初期化
      pose.setPose(getRobotData.response.tmsdb[0].x, getRobotData.response.tmsdb[0].y,
                   getRobotData.response.tmsdb[0].ry);  // set robot odometry
      robot.lock();
      robot.moveTo(pose, false);
      robot.unlock();
    }
  }
  return true;
}

//ロボット内部のオドメトリ値を変更
bool Psetodom(tms_msg_rc::tms_rc_ppose::Request &req, tms_msg_rc::tms_rc_ppose::Response &res)
{
  pose.setPose(0.0, 0.0, 0.0);                                    //関数突入時に大域変数pose初期化
  pose.setPose((double)req.px, (double)req.py, (double)req.pth);  // set robot odometry "pose"

  robot.lock();
  robot.moveTo(pose, false);
  robot.unlock();

  return true;
}

//前進・後退・回転行動
bool Pmove(tms_msg_rc::tms_rc_pmove::Request &req, tms_msg_rc::tms_rc_pmove::Response &res)
{
  int add_command = 1;

  ArPose befpose;
  ArPose aftpose;
  ArPose spose;

  double befx, befy, befth = 0.0;
  double befxs, befys, befths = 0.0;
  double aftxs, aftys, aftths = 0.0;
  double aftx, afty, aftth = 0.0;
  double spin = 0.0;

  switch (req.command)
  {
    //前進・後退(mm)
    case 1:
      ROS_INFO("go ahead : %f mm", req.pdist);
      robot.lock();
      robot.move((double)req.pdist);
      robot.unlock();

      if (add_command)
      {
        while (1)
        {
          robot.lock();
          if (robot.isMoveDone())
          {
            robot.unlock();
            break;
          }
          robot.unlock();
          sleep(1);
        }
      }

      break;

    //回転(相対角度degree)
    case 2:
      ROS_INFO("spin : %f degree", req.pangle);
      robot.lock();
      befpose = robot.getPose();
      robot.unlock();

      //移動前のロボット中心における姿勢を取得
      befx = befpose.getX();
      befy = befpose.getY();
      befth = befpose.getTh();
      ROS_INFO("before robot pose = (%lf,%lf,%lf)", befx, befy, befth);

      befxs = befx + POS_ABER * cos((befth * M_PI) / 180);
      befys = befy + POS_ABER * sin((befth * M_PI) / 180);
      befths = befth;
      ROS_INFO("before spin pose = (%lf,%lf,%lf)", befxs, befys, befths);

      aftxs = befxs;
      aftys = befys;
      if ((befths + req.pangle) > 180)
        aftths = (befths + req.pangle) - 360;
      else if ((befths + req.pangle) < -180)
        aftths = (befths + req.pangle) + 360;
      else
        aftths = befths + (double)req.pangle;
      ROS_INFO("after spin pose = (%lf,%lf,%lf)", aftxs, aftys, aftths);

      robot.lock();
      robot.setDeltaHeading((double)req.pangle);
      robot.unlock();

      if (add_command)
      {
        while (1)
        {
          robot.lock();
          if (robot.isHeadingDone())
          {
            robot.unlock();
            break;
          }
          robot.unlock();
          sleep(1);
        }
      }

      robot.lock();
      aftpose = robot.getPose();
      robot.unlock();
      ROS_INFO("after robot pose(not modified) = (%lf,%lf,%lf)", aftpose.getX(), aftpose.getY(), aftpose.getTh());

      //ここで回転中心のズレを修正するための計算を行う
      aftx = aftxs - POS_ABER * cos((aftths * M_PI) / 180);
      afty = aftys - POS_ABER * sin((aftths * M_PI) / 180);
      aftth = aftths;
      ROS_INFO("after robot pose(modified) = (%lf,%lf,%lf)", aftx, afty, aftth);

      spose.setPose(aftx, afty, aftth);
      ROS_INFO("spose = (%lf,%lf,%lf)", spose.getX(), spose.getY(), spose.getTh());

      //中心座標を回転ズレに対応した姿勢であるsposeへ変換
      robot.lock();
      robot.moveTo(spose, false);
      robot.unlock();

      ROS_INFO("spose = (%lf,%lf,%lf)", spose.getX(), spose.getY(), spose.getTh());

      break;

    //その他の場合はエラー
    default:
      ROS_INFO("pmove command error");
      break;
  }

  return true;
}

//パラメータ取得
//主に「前の命令を終了したかどうか？」を判定するのに利用する。
//今は「動作終了までメッセージを返さない」仕様に変更されているため、これを使うことはない。
bool Pparam(tms_msg_rc::tms_rc_pparam::Request &req, tms_msg_rc::tms_rc_pparam::Response &res)
{
  switch (req.command)
  {
    //直進してる？
    //直進してればis_pmove = 1
    //止まっていればis_pmove = 0
    // ispspinは2で固定値
    case 1:
      robot.lock();
      if (robot.isMoveDone())
      {
        res.ispmove = 0;
        res.ispspin = 2;
      }
      else
      {
        res.ispmove = 1;
        res.ispspin = 2;
      }
      robot.unlock();
      break;

    //回転してる？
    //回転してればis_pspin = 1
    //止まっていればis_pmove = 0
    // ispmoveは2で固定値
    case 2:
      robot.lock();
      if (robot.isHeadingDone())
      {
        res.ispspin = 0;
        res.ispmove = 2;
      }
      else
      {
        res.ispspin = 1;
        res.ispmove = 2;
      }
      robot.unlock();
      break;

    //動いてる？(直進/回転の各パラメータを返却)
    //動いていれば対応する返答パラメータ=1
    //止まっていれば対応する返答パラメータ=0
    defalut:

      ROS_INFO("command error");

      break;
  }

  return true;
}

void publishodom(const ros::TimerEvent &event)
{
  ArPose temp_pose;
  tms_msg_ss::fss_object_data msg;
  tms_msg_db::tmsdb_data msg_temp1;

  robot.lock();
  temp_pose = robot.getPose();  // poseを取得
  robot.unlock();

  msg_temp1.tMeasuredTime = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;
  msg_temp1.iType = 1;
  msg_temp1.iID = 5;
  msg_temp1.fX = temp_pose.getX();
  msg_temp1.fY = temp_pose.getY();
  msg_temp1.fZ = 0;
  msg_temp1.fTheta = temp_pose.getTh();
  msg_temp1.iPlace = 17;
  msg_temp1.iState = 1;

  msg.msgTMSInfo.push_back(msg_temp1);
  publish1.publish(msg);
}

////////////////////////  main  ///////////////////////////
//初期化に失敗して初期位置がおかしくなる場合があるので注意！///
int main(int argc, char **argv)
{
  // ROS init//
  ros::init(argc, argv, "test1_move");
  ros::NodeHandle n;
  ROS_INFO("test1_move : init");
  ros::Rate loop_rate(10);
  ros::MultiThreadedSpinner spinner(2);

  ros::Timer timer = n.createTimer(ros::Duration(0.1), publishodom);

  publish1 = n.advertise< tms_msg_ss::fss_object_data >("kkp_current_data", 10);

  ros::ServiceServer service1 = n.advertiseService("psetodom", Psetodom);
  ros::ServiceServer service2 = n.advertiseService("pmove", Pmove);
  ros::ServiceServer service3 = n.advertiseService("pparam", Pparam);
  ros::ServiceServer service4 = n.advertiseService("vicon_psetodom", Vicon_Psetodom);

  get_data_client = n.serviceClient< tms_msg_db::TmsdbGetData >("/tms_db_reader/dbreader");

  // Aria init//
  Aria::init();

  ArArgumentParser argParser(&argc, argv);
  ArSimpleConnector con(&argParser);

  // the connection handler from above
  ConnHandler ch(&robot);

  if (!Aria::parseArgs())
  {
    Aria::logOptions();
    Aria::shutdown();
    return 1;
  }

  if (!con.connectRobot(&robot))
  {
    ArLog::log(ArLog::Normal, "directMotionExample: Could not connect to the robot. Exiting.");
    return 1;
  }

  ArLog::log(ArLog::Normal, "directMotionExample: Connected.");

  //キーボード接続
  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);
  robot.attachKeyHandler(&keyHandler);
  ROS_INFO("You may press escape to exit\n");

  // Aria内の時間計測用
  ArTime start;

  // Run the robot processing cycle in its own thread. Note that after starting this
  // thread, we must lock and unlock the ArRobot object before and after accessing it.
  robot.runAsync(false);

  printf("TEST PROGRAM START : Turn and Ahead\n");

  printf("MaxRotAccel = %lf\n", robot.getAbsoluteMaxRotAccel());
  printf("MaxRotDecel = %lf\n", robot.getAbsoluteMaxRotDecel());
  printf("MaxRotVel = %lf\n", robot.getAbsoluteMaxRotVel());
  printf("MaxTransAccel = %lf\n", robot.getAbsoluteMaxTransAccel());
  printf("MaxTransDecel = %lf\n", robot.getAbsoluteMaxTransDecel());
  printf("MaxTransVel = %lf\n\n", robot.getAbsoluteMaxTransVel());

  // initialize Robot's odometry using vicon data
  tms_msg_db::TmsdbGetData getRobotData;
  getRobotData.request.tmsdb.id = 2006;      // KXP ID
  getRobotData.request.tmsdb.sensor = 3001;  // Vicon ID
  if (get_data_client.call(getRobotData))
  {
    ROS_INFO("Get info of robot ID: %d\n", getRobotData.request.tmsdb.id);
  }
  else
  {
    ROS_INFO("Failed to call service getRobotData ID: %d\n", getRobotData.request.tmsdb.id);
    return false;
  }
  if (!getRobotData.response.tmsdb.empty())
  {
    if (getRobotData.response.tmsdb[0].x != 0 && getRobotData.response.tmsdb[0].y != 0)
    {
      pose.setPose(0.0, 0.0, 0.0);  //関数突入時に大域変数pose初期化
      pose.setPose(getRobotData.response.tmsdb[0].x, getRobotData.response.tmsdb[0].y,
                   getRobotData.response.tmsdb[0].ry);  // set robot odometry
      robot.lock();
      robot.moveTo(pose, false);
      robot.unlock();
    }
    printf("setPose for (%lf, %lf, %lf)\n", getRobotData.response.tmsdb[0].x, getRobotData.response.tmsdb[0].y,
           getRobotData.response.tmsdb[0].ry);
  }

  spinner.spin();  //オドメトリ情報を外部に投げるメッセージ用…spin(1)
                   //外部からのサービス要求を受け入れる用…spin(2)

  printf("TEST PROGRAM : Actions finished, shutting down Aria and exiting.\n");
  Aria::shutdown();

  return 0;
}
