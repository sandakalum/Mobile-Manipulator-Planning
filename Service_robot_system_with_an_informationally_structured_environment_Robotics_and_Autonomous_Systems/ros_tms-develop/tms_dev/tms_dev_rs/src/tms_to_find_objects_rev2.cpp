// include for ROS
#include <ros/ros.h>
#include <unistd.h>
#include <boost/date_time/local_time/local_time.hpp>

// include for PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// TMSDB_srv
#include <tms_msg_db/tmsdb_get_objects_info.h>
#include <tms_msg_db/tmsdb_get_person_info.h>
#include <tms_msg_db/tmsdb_get_furnitures_info.h>
#include <tms_msg_db/tmsdb_get_robots_info.h>
#include <tms_msg_db/tmsdb_get_pcd_info.h>
#include <tms_msg_db/tmsdb_file_conservation.h>
#include <tms_msg_db/tmsdb_objects_data.h>

#include <tms_msg_rp/tms_ts_find_objects.h>
#include <tms_msg_sa/tms_to_activate_service.h>
#include <tms_msg_ss/ods_change_detection.h>
#include <tms_msg_rc/tag_data.h>

#include <tms_msg_rp/rps_position.h>
#include <tms_msg_rp/rps_route.h>
#include <tms_msg_rp/rps_goal_planning.h>
#include <tms_msg_rp/rps_path_planning.h>
#include <tms_msg_rp/rps_robot_drive.h>
#include <tms_msg_rp/rps_command.h>
#include <tms_msg_rp/rps_command_array.h>
#include <tms_msg_rp/rps_command_to_move_smartpal.h>
#include <tms_msg_rp/rps_arm_drive.h>
#include <tms_msg_rc/smartpal_control.h>

#include <tms_msg_rc/tag_data.h>

struct FURNITURES_TOTAL
{
  int id;
  long long unsigned int total_time;
};

//大域変数
ros::Time tMeasuredTime_spfm;
ros::Time tMeasuredTime_spbm;
ros::Time tMeasuredTime_splh;
ros::Time tMeasuredTime_sprh;
unsigned int uiObjectNum_spfm;
unsigned int uiObjectNum_spbm;
unsigned int uiObjectNum_splh;
unsigned int uiObjectNum_sprh;
unsigned int uiTagDataSize_spfm;
unsigned int uiTagDataSize_spbm;
unsigned int uiTagDataSize_splh;
unsigned int uiTagDataSize_sprh;
std::vector< uint8_t > ucTagData_spfm;
std::vector< uint8_t > ucTagData_spbm;
std::vector< uint8_t > ucTagData_splh;
std::vector< uint8_t > ucTagData_sprh;

tms_msg_rp::tms_ts_find_objects::Request task_find_objects_information;

//これで探し回る家具等を管理する
//基本的にDB_readerより持ってきた情報を保持する

ros::ServiceServer service1;

// client0X -> tmsdb_writer, tmsdb_readerとのやりとり
ros::ServiceClient client01;  // furnitures_info from TMSDB
ros::ServiceClient client02;  // robots_info from TMSDB
ros::ServiceClient client03;  // pcd_info(furniture) form TMSDB
ros::ServiceClient client04;  // INSERT FILE to TMSDB	(pcd_file)
ros::ServiceClient client05;  // INSERT OBJECT DATA to TMSDB (RFID_READ @ robot)

// clientX -> SKILL partとのやりとり
ros::ServiceClient client1;  // goal planning(candidate)
ros::ServiceClient client2;  // path planning
ros::ServiceClient client3;  // robot drive
ros::ServiceClient client4;  // kinect call
ros::ServiceClient client5;  // READ可能位置への計画
ros::ServiceClient client6;  // READ可能位置への移動
ros::ServiceClient client7;  // READ可能な手の位置への計画
ros::ServiceClient client8;  // READ可能な手の位置への姿勢変更
// ros::ServiceClient client9;		//tag_READ

ros::ServiceClient client10;  // rps_command_array
ros::ServiceClient client11;  // robot_control_command

void spfm_tag_data_callback(const tms_msg_rc::tag_data::ConstPtr& msg)
{
  ucTagData_spfm.clear();
  tMeasuredTime_spfm = msg->tMeasuredTime;
  uiObjectNum_spfm = msg->uiObjectNum;
  uiTagDataSize_spfm = msg->uiTagDataSize;

  if (msg->uiObjectNum != 0)
  {
    ucTagData_spfm = msg->ucTagData;
  }
}

void spbm_tag_data_callback(const tms_msg_rc::tag_data::ConstPtr& msg)
{
  ucTagData_spbm.clear();
  tMeasuredTime_spbm = msg->tMeasuredTime;
  uiObjectNum_spbm = msg->uiObjectNum;
  uiTagDataSize_spbm = msg->uiTagDataSize;

  if (msg->uiObjectNum != 0)
  {
    ucTagData_spbm = msg->ucTagData;
  }
}

void splh_tag_data_callback(const tms_msg_rc::tag_data::ConstPtr& msg)
{
  ucTagData_splh.clear();
  tMeasuredTime_splh = msg->tMeasuredTime;
  uiObjectNum_splh = msg->uiObjectNum;
  uiTagDataSize_splh = msg->uiTagDataSize;

  if (msg->uiObjectNum != 0)
  {
    ucTagData_splh = msg->ucTagData;
  }
}

void sprh_tag_data_callback(const tms_msg_rc::tag_data::ConstPtr& msg)
{
  ucTagData_sprh.clear();
  tMeasuredTime_sprh = msg->tMeasuredTime;
  uiObjectNum_sprh = msg->uiObjectNum;
  uiTagDataSize_sprh = msg->uiObjectNum;

  if (msg->uiObjectNum != 0)
  {
    ucTagData_sprh = msg->ucTagData;
  }
}

bool tms_to_find_objects_func(tms_msg_rp::tms_ts_find_objects::Request& req,
                              tms_msg_rp::tms_ts_find_objects::Response& res)
{
  ROS_INFO("tms_to_find_objects_func");

  ros::Rate r(10);

  tms_msg_db::tmsdb_get_furnitures_info srv01;  // get furnitures info
  tms_msg_db::tmsdb_get_robots_info srv02;      // get robots info
  tms_msg_db::tmsdb_get_pcd_info srv03;         // get pcd info
  tms_msg_db::tmsdb_file_conservation srv04;    // insert file to TMSDB
  tms_msg_db::tmsdb_objects_data srv05;         // INSERT OBJECT_DATA

  tms_msg_rp::rps_goal_planning srv1;
  tms_msg_rp::rps_path_planning srv2;
  tms_msg_rp::rps_robot_drive srv3;
  //	tms_utility::ods_change_dt srv4;
  tms_msg_ss::ods_change_detection srv4;

  //	tms_utility::rps_arm_drive srv8;
  tms_msg_rc::tag_data srv9;

  //	tms_msg_rp::rps_command_to_move_smartpal srv10;
  tms_msg_rc::smartpal_control srv11;

  int all_success_flag = 0;
  int dummy_scanf = 0;

  //タスクの情報を保持する
  task_find_objects_information = req;

  //今は命令先のロボットをSmartpalに固定
  srv1.request.robot_id = 1;
  srv2.request.robot_id = 1;
  //	srv3.request.robot_id = 1;

  //サービスのidはNo.1: find_object
  srv1.request.task_id = 1;

  //ロボット初期位置設定
  printf("ロボット初期位置設定: 1000, 1000, 0\n");
  srv11.request.unit = 1;
  srv11.request.cmd = 9;
  srv11.request.arg.push_back(1000.0);
  srv11.request.arg.push_back(1000.0);
  srv11.request.arg.push_back(0.0);
  if (client11.call(srv11))
  {
  }
  else
  {
    ROS_ERROR("Failed to call service smartpal_control_vehicle");
  }
  srv11.request.arg.clear();

  printf("ロボット速度設定: 150, 10\n");
  srv11.request.unit = 1;
  srv11.request.cmd = 10;
  srv11.request.arg.push_back(200.0);
  srv11.request.arg.push_back(10.0);
  if (client11.call(srv11))
  {
  }
  else
  {
    ROS_ERROR("Failed to call service smartpal_control_vehicle");
  }
  srv11.request.arg.clear();

  // missing_objectsの探索開始
  for (unsigned int i = 0; i < req.search_furnitures_id.size(); i++)
  {
    unsigned int ret = 1;
    tms_msg_rp::rps_command temp_command;

    double phi = 0.00;
    double L = 0.00;

    double object_x = 0.00;
    double object_z = 0.00;

    double temp_a = 0.00;
    double temp_b = 0.00;
    double temp_x = 0.00;
    double temp_y = 0.00;
    double d1 = 150.00;
    double d2 = 15.00;

    printf("家具ID取得待機中.");
    sleep(1);
    printf(".");
    sleep(1);
    printf(".\n");
    sleep(1);
    printf("次の家具を検索します: %d\n", req.search_furnitures_id[i]);

    // activate goal_planning(家具IDに対する変化箇所検出位置の決定)
    // 1箇所ずつ投げることに気をつける
    if (req.search_furnitures_id[i] == 12)
    {
      ROS_INFO("Bed search program is not exist");
      all_success_flag = 12;
      continue;
    }
    else if (req.search_furnitures_id[i] == 13)
    {
      ROS_INFO("desk search program is not exist");
      all_success_flag = 13;
      continue;
    }
    else if (req.search_furnitures_id[i] == 14)
    {
      srv2.request.rps_goal_candidate.rps_route.resize(1);
      srv2.request.rps_goal_candidate.rps_route.front().x = 3000;
      srv2.request.rps_goal_candidate.rps_route.front().y = 2000;
      srv2.request.rps_goal_candidate.rps_route.front().th = 180;
    }
    else
    {
      ROS_INFO("invalid search furnitures_id");
      break;
    }

    //		ROS_INFO("GOTO");
    //		goto SIN;

    // activate path_planning(目標とする位置に対する移動計画を行う)
    if (client2.call(srv2))
    {
      if (srv2.response.success == 0)
      {
        ROS_INFO("success = %d", srv2.response.success);
      }
      else
      {
        ROS_INFO("failed_ID = %d: '%s'", srv2.response.success, srv2.response.message.c_str());
        all_success_flag = 2;
      }
    }
    else
    {
      ROS_ERROR("Failed to call service rps_path_planning");
      all_success_flag = 102;
    }

    // activate robot_drive(ロボットを対象家具の付近、変化箇所検出位置に移動させる)
    for (unsigned int j = 0; j < srv2.response.rps_path.front().rps_route.size(); j++)
    {
      srv11.request.unit = 1;
      srv11.request.cmd = 15;
      srv11.request.arg.push_back(srv2.response.rps_path.front().rps_route[j].x);
      srv11.request.arg.push_back(srv2.response.rps_path.front().rps_route[j].y);
      srv11.request.arg.push_back(srv2.response.rps_path.front().rps_route[j].th);

      if (client11.call(srv11))
      {
      }
      else
      {
        ROS_ERROR("Failed to call service smartpal_control");
        all_success_flag = 103;
      }
      //移動中はBUSYとして停止する
      while (true)
      {
        srv11.request.arg.clear();
        srv11.request.unit = 1;
        srv11.request.cmd = 7;
        if (client11.call(srv11))
        {
          if (srv11.response.result != 16)
          {
            ROS_INFO("Smartpal BUSY %d", srv11.response.result);
            sleep(1);
          }
          else
          {
            break;
          }
        }
        else
        {
          ROS_ERROR("Failed to call service smartpal_control_get_state");
        }
      }
    }

    // commandの起動(把持姿勢：把持可能位置への姿勢変更)
    //		ROS_INFO("smartpal_arm_drive");
    //
    //		srv11.request.arg.clear();
    //		srv11.request.unit = 2;
    //		srv11.request.cmd = 15;
    //		srv11.request.arg.push_back(0.0);	//1
    //		srv11.request.arg.push_back(-20.0);	//2
    //		srv11.request.arg.push_back(0.0);	//3
    //		srv11.request.arg.push_back(0.0);	//4
    //		srv11.request.arg.push_back(0.0);	//5
    //		srv11.request.arg.push_back(0.0);	//6
    //		srv11.request.arg.push_back(0.0);	//7
    //		srv11.request.arg.push_back(10.0);	//vel
    //		if (client11.call(srv11))
    //			ROS_INFO("result: %d", srv11.response.result);
    //		else{
    //			ROS_ERROR("Failed to call service sp4_control");
    //			all_success_flag = 106;
    //		}
    //		srv11.request.arg.clear();
    //		srv11.request.unit = 2;
    //		srv11.request.cmd = 15;
    //		srv11.request.arg.push_back(20.0);	//1
    //		srv11.request.arg.push_back(-4.0);	//2
    //		srv11.request.arg.push_back(15.0);	//3
    //		srv11.request.arg.push_back(30.0);	//4
    //		srv11.request.arg.push_back(89.0);	//5
    //		srv11.request.arg.push_back(36.0);	//6
    //		srv11.request.arg.push_back(0.0);	//7
    //		srv11.request.arg.push_back(10.0);	//vel
    //		if (client11.call(srv11))
    //			ROS_INFO("result: %d", srv11.response.result);
    //		else{
    //			ROS_ERROR("Failed to call service sp4_control");
    //			all_success_flag = 106;
    //		}

    //		while(true){
    //			srv11.request.unit = 2;
    //			srv11.request.cmd = 0;	//clear Alarm
    //			if (client11.call(srv11)){
    //				//ROS_INFO("result: %d", srv11.response.result);
    //			}else{
    //				ROS_ERROR("Failed to call service sp4_control");
    //				all_success_flag = 106;
    //			}
    //
    //			srv11.request.unit = 2;
    //			srv11.request.cmd = 7;
    //			if (client11.call(srv11))
    //				ROS_INFO("result: %d", srv11.response.result);
    //			else{
    //				ROS_ERROR("Failed to call service sp4_control");
    //				all_success_flag = 106;
    //			}
    //
    //			sleep(1);
    //		}
    //
    //		ROS_INFO("sleep");
    //		sleep(1000);

    //		while(true){
    //			srv11.request.unit = 2;
    //			srv11.request.cmd = 7;
    //
    //			if (client11.call(srv11))
    //				ROS_INFO("result_arm R: %d", srv11.response.result);
    //			else{
    //				ROS_ERROR("Failed to call service sp4_control");
    //				all_success_flag = 106;
    //			}
    //			srv11.request.unit = 3;
    //			srv11.request.cmd = 7;
    //
    //			if (client11.call(srv11))
    //				ROS_INFO("result_arm L: %d", srv11.response.result);
    //			else{
    //				ROS_ERROR("Failed to call service sp4_control");
    //				all_success_flag = 106;
    //			}
    //
    //			srv11.request.arg.clear();
    //			srv11.request.unit = 2;
    //			srv11.request.cmd = 15;
    //			srv11.request.arg.push_back(0.0);	//1
    //			srv11.request.arg.push_back(-20.0);	//2
    //			srv11.request.arg.push_back(0.0);	//3
    //			srv11.request.arg.push_back(0.0);	//4
    //			srv11.request.arg.push_back(0.0);	//5
    //			srv11.request.arg.push_back(0.0);	//6
    //			srv11.request.arg.push_back(0.0);	//7
    //			srv11.request.arg.push_back(10.0);	//vel
    //			if (client11.call(srv11))
    //				ROS_INFO("result: %d", srv11.response.result);
    //			else{
    //				ROS_ERROR("Failed to call service sp4_control");
    //				all_success_flag = 106;
    //			}
    //			srv11.request.arg.clear();
    //			srv11.request.unit = 3;
    //			srv11.request.cmd = 15;
    //			srv11.request.arg.push_back(0.0);	//1
    //			srv11.request.arg.push_back(-20.0);	//2
    //			srv11.request.arg.push_back(0.0);	//3
    //			srv11.request.arg.push_back(0.0);	//4
    //			srv11.request.arg.push_back(0.0);	//5
    //			srv11.request.arg.push_back(0.0);	//6
    //			srv11.request.arg.push_back(0.0);	//7
    //			srv11.request.arg.push_back(10.0);	//vel
    //			if (client11.call(srv11))
    //				ROS_INFO("result: %d", srv11.response.result);
    //			else{
    //				ROS_ERROR("Failed to call service sp4_control");
    //				all_success_flag = 106;
    //			}
    //
    //			srv11.request.unit = 2;
    //			srv11.request.cmd = 7;
    //			if (client11.call(srv11))
    //				ROS_INFO("result_arm R: %d", srv11.response.result);
    //			else{
    //				ROS_ERROR("Failed to call service sp4_control");
    //				all_success_flag = 106;
    //			}
    //			srv11.request.unit = 3;
    //			srv11.request.cmd = 7;
    //			if (client11.call(srv11))
    //				ROS_INFO("result_arm L: %d", srv11.response.result);
    //			else{
    //				ROS_ERROR("Failed to call service sp4_control");
    //				all_success_flag = 106;
    //			}
    //
    //			sleep(3);
    //
    //			srv11.request.arg.clear();
    //			srv11.request.unit = 2;
    //			srv11.request.cmd = 15;
    //			srv11.request.arg.push_back(20.0);	//1
    //			srv11.request.arg.push_back(-4.0);	//2
    //			srv11.request.arg.push_back(15.0);	//3
    //			srv11.request.arg.push_back(30.0);	//4
    //			srv11.request.arg.push_back(89.0);	//5
    //			srv11.request.arg.push_back(36.0);	//6
    //			srv11.request.arg.push_back(0.0);	//7
    //			srv11.request.arg.push_back(10.0);	//vel
    //			if (client11.call(srv11))
    //				ROS_INFO("result: %d", srv11.response.result);
    //			else{
    //				ROS_ERROR("Failed to call service sp4_control");
    //				all_success_flag = 106;
    //			}
    //			srv11.request.arg.clear();
    //			srv11.request.unit = 3;
    //			srv11.request.cmd = 15;
    //			srv11.request.arg.push_back(20.0);	//1
    //			srv11.request.arg.push_back(-4.0);	//2
    //			srv11.request.arg.push_back(15.0);	//3
    //			srv11.request.arg.push_back(30.0);	//4
    //			srv11.request.arg.push_back(89.0);	//5
    //			srv11.request.arg.push_back(36.0);	//6
    //			srv11.request.arg.push_back(0.0);	//7
    //			srv11.request.arg.push_back(10.0);	//vel
    //
    //			if (client11.call(srv11))
    //				ROS_INFO("result: %d", srv11.response.result);
    //			else{
    //				ROS_ERROR("Failed to call service sp4_control");
    //				all_success_flag = 106;
    //			}
    //
    //			printf("\n");
    //			sleep(3);
    //		}

    // SIN:
    //本来はfssや視覚によるロボット座標の修正が必要
    // fssの測定結果をwriterを通じてTMS_DB,smartpalのオドメトリデータの両方を書き換える必要がある
    //この2つが一致しないと,目標座標とロボット保持オドメトリに食い違いが生じて移動誤差が発生する
    //※(現在のロボットの位置・アームの位置・物品の絶対座標・移動距離 =>
    //「TMS_DBのロボット座標」とKinectで得た情報より変換…ここで出るのは相対座標基準)
    //※(移動命令 => 「ロボット内部」に保持しているオドメトリの値で指示)
    //あるいは、ロボットに「x,y方向の移動量」を与えれば一致する必要はないが…

    // Change_detectionのための環境情報の取得
    // client01 -> get furnitures info(家具の位置情報の取得)
    // client02 -> get robots info(ロボットの現在位置情報の取得)
    // client03 -> get pcd info(目標家具の最新pcdの取得)
    srv01.request.furnitures_id = req.search_furnitures_id[i];
    if (client01.call(srv01))
    {
      ROS_INFO("[ID=%d] x = %f, y =%f, z=%f", srv01.request.furnitures_id, srv01.response.furniture_x,
               srv01.response.furniture_y, srv01.response.furniture_z);
    }

    srv02.request.robots_id = 1;
    if (client02.call(srv02))
    {
      ROS_INFO("[ID=%d] x = %f, y =%f, z=%f", srv02.request.robots_id, srv02.response.robots_x, srv02.response.robots_y,
               srv02.response.robots_theta);
    }

    srv03.request.id = req.search_furnitures_id[i];
    if (client03.call(srv03))
    {
      ROS_INFO("[ID=%d] rostime = %lu, pcl_file = %s, get_x = %f, get_y =%f, get_z=%f, get_theta = %f",
               srv03.request.id, srv03.response.rostime, srv03.response.pcd_file.c_str(), srv03.response.get_x,
               srv03.response.get_y, srv03.response.get_z, srv03.response.get_theta);
    }
    // commandの起動(kinect動作...変化箇所検出)
    //出力：変化箇所の絶対座標系[x,y]、取得時間、更新されたPCD、
    srv4.request.cloud = srv03.response.cloud;
    srv4.request.furniture_id = req.search_furnitures_id[i];
    srv4.request.furniture.position.x = srv01.response.furniture_x;
    srv4.request.furniture.position.y = srv01.response.furniture_y;
    srv4.request.furniture.position.z = srv01.response.furniture_z;
    srv4.request.robot.x = srv02.response.robots_x;
    srv4.request.robot.y = srv02.response.robots_y;
    srv4.request.robot.theta = srv02.response.robots_theta;

    //変化箇所は配列として返却されるので
    //どこに対して把持計画を使うかを考える必要がある
    if (client4.call(srv4))
    {
      for (unsigned int j = 0; j < srv4.response.objects.poses.size(); j++)
      {
        ROS_INFO("Success![%d] x = %f, y = %f, z = %f", j, srv4.response.objects.poses[j].position.x,
                 srv4.response.objects.poses[j].position.y, srv4.response.objects.poses[j].position.z);
      }
    }
    else
    {
      ROS_ERROR("Failed to call service ods_change_dt");
      all_success_flag = 104;
    }

    if (srv4.response.objects.poses.size() != 1)
    {
      ROS_INFO("srv4.response.objects.poses.size != 1");
    }

    if (srv4.response.objects.poses.empty() != true)
    {
      //結果を書き込むためにwriterへ
      // 1. 家具に対するPCDファイル、取得時点のロボット位置・姿勢・場所
      srv04.request.cloud = srv4.response.cloud;
      srv04.request.rostime = srv4.response.tMeasuredTime.toNSec();
      srv04.request.id = srv4.request.furniture_id;
      char temp_filename[256];
      sprintf(temp_filename, "'/home/pcl_data/%lu_%d_robot%d.pcd'", srv04.request.rostime, srv04.request.id,
              srv1.request.robot_id);
      srv04.request.filename = std::string(temp_filename);
      std::cout << srv04.request.filename << std::endl;
      srv04.request.get_x = srv4.request.robot.x;
      srv04.request.get_y = srv4.request.robot.y;
      srv04.request.get_z = 0;  //床の上のため
      srv04.request.get_theta = srv4.request.robot.theta;
      srv04.request.get_place = 912;  //利用していない

      //			if(client04.call(srv04)){
      //				ROS_INFO("Success![%d] %s",srv04.response.success , srv04.response.message.c_str());
      //			}
      //			else{
      //				ROS_ERROR("Failed to call service tmsdb_file_conservation");
      //				all_success_flag = 105;
      //			}

      // commandの起動(経路計画：RFIDリーダによる認識が可能な位置へ，ベース経路の出力)
      //現状，これはRPSの接触判定に当たるためできない．直進で進む？
      L = sqrt(((621.554) * (621.554)) + ((-149.082) * (-149.082)));
      phi = atan2(-149.082, 621.554);

      object_x = srv4.response.objects.poses.front().position.x - d2;
      object_z = srv4.response.objects.poses.front().position.z * cos(0.380506377) + d1;

      ROS_INFO("move_x = %lf", L * cos(srv02.response.robots_theta * (3.141592 / 180) + phi) -
                                   (sqrt(pow(object_x, 2) + pow(object_z, 2))) *
                                       cos(srv02.response.robots_theta * (3.141592 / 180) - atan2(object_x, object_z)));
      ROS_INFO("move_y = %lf", L * sin(srv02.response.robots_theta * (3.141592 / 180) + phi) -
                                   (sqrt(pow(object_x, 2) + pow(object_z, 2))) *
                                       sin(srv02.response.robots_theta * (3.141592 / 180) - atan2(object_x, object_z)));

      //移動先x座標
      temp_command.x = srv02.response.robots_x -
                       (L * cos(srv02.response.robots_theta * (3.141592 / 180) + phi) -
                        (sqrt(pow(object_x, 2) + pow(object_z, 2))) *
                            cos(srv02.response.robots_theta * (3.141592 / 180) - atan2(object_x, object_z)));
      //移動先y座標
      temp_command.y = srv02.response.robots_y -
                       (L * sin(srv02.response.robots_theta * (3.141592 / 180) + phi) -
                        (sqrt(pow(object_x, 2) + pow(object_z, 2))) *
                            sin(srv02.response.robots_theta * (3.141592 / 180) - atan2(object_x, object_z)));
      //移動先θ
      temp_command.th = srv02.response.robots_theta;

      ROS_INFO("[robot_now]x,y,th,[calc_temp]L,phi,[robot_to_object]object_x,object_z,[Robot_move]X,Y,TH)");
      ROS_INFO("([robot_now]%lf,%lf,%lf,[calc_temp]%lf,%lf,[robot_to_object]%lf,%lf,[Robot_move]%lf,%lf,%lf)",
               srv02.response.robots_x, srv02.response.robots_y, srv02.response.robots_theta, L, phi, object_x,
               object_z, temp_command.x, temp_command.y, temp_command.th);

      // commandの起動(経路移動：把持可能位置への移動)
      srv11.request.arg.clear();
      srv11.request.unit = 1;
      srv11.request.cmd = 15;
      srv11.request.arg.push_back(temp_command.x);
      srv11.request.arg.push_back(temp_command.y);
      srv11.request.arg.push_back(temp_command.th);
      if (client11.call(srv11))
      {
      }
      else
      {
        ROS_ERROR("Failed to call service smartpal_control_vehicle");
        all_success_flag = 106;
      }
      while (true)
      {
        srv11.request.arg.clear();
        srv11.request.unit = 1;
        srv11.request.cmd = 7;
        if (client11.call(srv11))
        {
          if (srv11.response.result != 16)
          {
            ROS_INFO("Smartpal BUSY %d", srv11.response.result);
            sleep(1);
          }
          else
          {
            break;
          }
        }
        else
        {
          ROS_ERROR("Failed to call service smartpal_control_get_state");
        }
      }

      // commandの起動(把持計画：RFIDリーダによる認識が可能な姿勢の経路出力)

      // commandの起動(把持計画：RFIDリーダによる認識が可能な姿勢へ変更)
      srv11.request.arg.clear();
      srv11.request.unit = 2;
      srv11.request.cmd = 15;
      srv11.request.arg.push_back(-1.51573);  // 1
      srv11.request.arg.push_back(-4.0);      // 2
      srv11.request.arg.push_back(6.4605);    // 3
      srv11.request.arg.push_back(1.09681);   // 4
      srv11.request.arg.push_back(0.0);       // 5
      srv11.request.arg.push_back(8.23093);   // 6
      srv11.request.arg.push_back(-1.56232);  // 7
      srv11.request.arg.push_back(30.0);      // vel
      if (client11.call(srv11))
        ROS_INFO("result: %d", srv11.response.result);
      else
      {
        ROS_ERROR("Failed to call service sp4_control");
        all_success_flag = 106;
      }
      sleep(2);
      srv11.request.arg.clear();
      srv11.request.unit = 2;
      srv11.request.cmd = 15;
      srv11.request.arg.push_back(-40.0);     // 1
      srv11.request.arg.push_back(-4.0);      // 2
      srv11.request.arg.push_back(80.0);      // 3
      srv11.request.arg.push_back(75.0);      // 4
      srv11.request.arg.push_back(0.0);       // 5
      srv11.request.arg.push_back(8.23093);   // 6
      srv11.request.arg.push_back(-1.56232);  // 7
      srv11.request.arg.push_back(30.0);      // vel
      if (client11.call(srv11))
        ROS_INFO("result: %d", srv11.response.result);
      else
      {
        ROS_ERROR("Failed to call service sp4_control");
        all_success_flag = 106;
      }
      sleep(3);
      srv11.request.arg.clear();
      srv11.request.unit = 2;
      srv11.request.cmd = 15;
      srv11.request.arg.push_back(-35.1028);  // 1
      srv11.request.arg.push_back(-3.32541);  // 2
      srv11.request.arg.push_back(55.6545);   // 3
      srv11.request.arg.push_back(81.5873);   // 4
      srv11.request.arg.push_back(23.4591);   // 5
      srv11.request.arg.push_back(13.8435);   // 6
      srv11.request.arg.push_back(0.211368);  // 7
      srv11.request.arg.push_back(30.0);      // vel
      if (client11.call(srv11))
        ROS_INFO("result: %d", srv11.response.result);
      else
      {
        ROS_ERROR("Failed to call service sp4_control");
        all_success_flag = 106;
      }
      sleep(2);
      srv11.request.arg.clear();
      srv11.request.unit = 2;
      srv11.request.cmd = 15;
      srv11.request.arg.push_back(-32.988);   // 1
      srv11.request.arg.push_back(-3.07647);  // 2
      srv11.request.arg.push_back(44.2093);   // 3
      srv11.request.arg.push_back(84.4797);   // 4
      srv11.request.arg.push_back(34.854);    // 5
      srv11.request.arg.push_back(20.0595);   // 6
      srv11.request.arg.push_back(0.289368);  // 7
      srv11.request.arg.push_back(30.0);      // vel
      if (client11.call(srv11))
        ROS_INFO("result: %d", srv11.response.result);
      else
      {
        ROS_ERROR("Failed to call service sp4_control");
        all_success_flag = 106;
      }
      sleep(2);
      srv11.request.arg.clear();
      srv11.request.unit = 2;
      srv11.request.cmd = 15;
      srv11.request.arg.push_back(-32.5664);  // 1
      srv11.request.arg.push_back(-3.13708);  // 2
      srv11.request.arg.push_back(39.5044);   // 3
      srv11.request.arg.push_back(85.1802);   // 4
      srv11.request.arg.push_back(40.4139);   // 5
      srv11.request.arg.push_back(21.9173);   // 6
      srv11.request.arg.push_back(0.270378);  // 7
      srv11.request.arg.push_back(30.0);      // vel
      if (client11.call(srv11))
        ROS_INFO("result: %d", srv11.response.result);
      else
      {
        ROS_ERROR("Failed to call service sp4_control");
        all_success_flag = 106;
      }
      sleep(2);
      srv11.request.arg.clear();
      srv11.request.unit = 2;
      srv11.request.cmd = 15;
      srv11.request.arg.push_back(-30.0);     // 1
      srv11.request.arg.push_back(-4.0);      // 2
      srv11.request.arg.push_back(6.4605);    // 3
      srv11.request.arg.push_back(90.0);      // 4
      srv11.request.arg.push_back(89.0);      // 5
      srv11.request.arg.push_back(36.0);      // 6
      srv11.request.arg.push_back(-1.56232);  // 7
      srv11.request.arg.push_back(30.0);      // vel
      if (client11.call(srv11))
        ROS_INFO("result: %d", srv11.response.result);
      else
      {
        ROS_ERROR("Failed to call service sp4_control");
        all_success_flag = 106;
      }
      sleep(2);
      srv11.request.arg.clear();
      srv11.request.unit = 2;
      srv11.request.cmd = 15;
      srv11.request.arg.push_back(20.0);      // 1
      srv11.request.arg.push_back(-4.0);      // 2
      srv11.request.arg.push_back(15.0);      // 3
      srv11.request.arg.push_back(30.0);      // 4
      srv11.request.arg.push_back(89.0);      // 5
      srv11.request.arg.push_back(36.0);      // 6
      srv11.request.arg.push_back(-1.56232);  // 7
      srv11.request.arg.push_back(30.0);      // vel
      if (client11.call(srv11))
        ROS_INFO("result: %d", srv11.response.result);
      else
      {
        ROS_ERROR("Failed to call service sp4_control");
        all_success_flag = 106;
      }

      sleep(5);

      // Reader読み取り
      for (unsigned int j = 0; j < 3; j++)
      {
        //ここでSpinOnce
        ROS_INFO("ros::spinOnce(10)");  //発行側レートに合わせると、最新1発分だけ取ってこれるようだ

        // 30回分待機して最初に取ってきたデータを利用する
        for (unsigned int k = 0; k < 30; k++)
        {
          ros::spinOnce();
          r.sleep();
          ROS_INFO("rfid_object_size = %d", uiObjectNum_sprh);
        }

        if (ucTagData_sprh.empty() != true)
        {
          for (unsigned int i_tmp; i_tmp < ucTagData_sprh.size(); i_tmp++)
          {
            printf("%u ", ucTagData_sprh[i_tmp]);
          }
          //情報が取得できて入ればそこでforループを抜ける
          break;
        }
        else
        {
          ROS_INFO("ucTagData_sprh is empty.");
          //少し前進
          // for条件により5mm*3=15mmまで前進可能
          srv11.request.arg.clear();
          srv11.request.unit = 1;
          srv11.request.cmd = 16;
          srv11.request.arg.push_back(5.0);
          srv11.request.arg.push_back(0.0);
          srv11.request.arg.push_back(0.0);
          if (client11.call(srv11))
          {
          }
          else
          {
            ROS_ERROR("Failed to call service smartpal_control_vehicle");
          }
        }
      }

      //認識結果を書き込むためにwriterへ(objects_history_INSERT)
      // 2. RFIDリーダを用いた物品の特定結果(=id)、移動後の日用品の位置(=change_dtによる絶対座標系)
      //本来ここはforですべて書き込む
      srv05.request.rostime = tMeasuredTime_sprh.toNSec();

      if (uiObjectNum_sprh == 1)
      {
        //			srv05.request.id = ucTagData_sprh[7];
        if (ucTagData_sprh[7] == 200)
          srv05.request.id = 51;
        else if (ucTagData_sprh[7] == 201)
          srv05.request.id = 52;
        else if (ucTagData_sprh[7] == 202)
          srv05.request.id = 53;
        else if (ucTagData_sprh[7] == 203)
          srv05.request.id = 54;
        else if (ucTagData_sprh[7] == 204)
          srv05.request.id = 55;
        else if (ucTagData_sprh[7] == 205)
          srv05.request.id = 56;
        else if (ucTagData_sprh[7] == 206)
          srv05.request.id = 57;
        else if (ucTagData_sprh[7] == 207)
          srv05.request.id = 58;
        else if (ucTagData_sprh[7] == 208)
          srv05.request.id = 59;
        else if (ucTagData_sprh[7] == 216)
          srv05.request.id = 60;
        else
          srv05.request.id = 200;
      }
      else
        srv05.request.id = 999999;

      ROS_INFO("object_x_abs = %f", srv02.response.robots_x +
                                        object_z * cos(srv02.response.robots_theta * (3.141592 / 180)) -
                                        object_x * sin((-1) * (srv02.response.robots_theta * (3.141592 / 180))));
      ROS_INFO("object_y_abs = %f", srv02.response.robots_y +
                                        object_z * sin(srv02.response.robots_theta * (3.141592 / 180)) -
                                        object_x * cos((-1) * (srv02.response.robots_theta * (3.141592 / 180))));

      srv05.request.x = srv4.response.objects.poses[0].position.x;  //本来ここは検索した場所のIDで
      srv05.request.y = srv4.response.objects.poses[0].position.y;
      srv05.request.z = srv4.response.objects.poses[0].position.z;
      srv05.request.theta = 0.0;                          //これはどうしよう…
      srv05.request.state = 2;                            // 2=found
      srv05.request.place = req.search_furnitures_id[i];  //探した家具のIDを突っ込む

      ROS_INFO("OBJECTS_DATA_INSERT");
      if (client05.call(srv05))
      {
        ROS_INFO("success?[%d], mes=%s", srv05.response.success, srv05.response.message.c_str());
      }
    }

    srv11.request.arg.clear();
    srv11.request.unit = 2;
    srv11.request.cmd = 15;
    srv11.request.arg.push_back(-30.0);     // 1
    srv11.request.arg.push_back(-4.0);      // 2
    srv11.request.arg.push_back(6.4605);    // 3
    srv11.request.arg.push_back(90.0);      // 4
    srv11.request.arg.push_back(89.0);      // 5
    srv11.request.arg.push_back(36.0);      // 6
    srv11.request.arg.push_back(-1.56232);  // 7
    srv11.request.arg.push_back(30.0);      // vel
    if (client11.call(srv11))
      ROS_INFO("result: %d", srv11.response.result);
    else
    {
      ROS_ERROR("Failed to call service sp4_control");
      all_success_flag = 106;
    }
    sleep(2);
    srv11.request.arg.clear();
    srv11.request.unit = 2;
    srv11.request.cmd = 15;
    srv11.request.arg.push_back(-32.5664);  // 1
    srv11.request.arg.push_back(-3.13708);  // 2
    srv11.request.arg.push_back(39.5044);   // 3
    srv11.request.arg.push_back(85.1802);   // 4
    srv11.request.arg.push_back(40.4139);   // 5
    srv11.request.arg.push_back(21.9173);   // 6
    srv11.request.arg.push_back(0.270378);  // 7
    srv11.request.arg.push_back(30.0);      // vel
    if (client11.call(srv11))
      ROS_INFO("result: %d", srv11.response.result);
    else
    {
      ROS_ERROR("Failed to call service sp4_control");
      all_success_flag = 106;
    }
    sleep(2);
    srv11.request.arg.clear();
    srv11.request.unit = 2;
    srv11.request.cmd = 15;
    srv11.request.arg.push_back(-32.988);   // 1
    srv11.request.arg.push_back(-3.07647);  // 2
    srv11.request.arg.push_back(44.2093);   // 3
    srv11.request.arg.push_back(84.4797);   // 4
    srv11.request.arg.push_back(34.854);    // 5
    srv11.request.arg.push_back(20.0595);   // 6
    srv11.request.arg.push_back(0.289368);  // 7
    srv11.request.arg.push_back(30.0);      // vel
    if (client11.call(srv11))
      ROS_INFO("result: %d", srv11.response.result);
    else
    {
      ROS_ERROR("Failed to call service sp4_control");
      all_success_flag = 106;
    }
    sleep(2);
    srv11.request.arg.clear();
    srv11.request.unit = 2;
    srv11.request.cmd = 15;
    srv11.request.arg.push_back(-35.1028);  // 1
    srv11.request.arg.push_back(-3.32541);  // 2
    srv11.request.arg.push_back(55.6545);   // 3
    srv11.request.arg.push_back(81.5873);   // 4
    srv11.request.arg.push_back(23.4591);   // 5
    srv11.request.arg.push_back(13.8435);   // 6
    srv11.request.arg.push_back(0.211368);  // 7
    srv11.request.arg.push_back(30.0);      // vel
    if (client11.call(srv11))
      ROS_INFO("result: %d", srv11.response.result);
    else
    {
      ROS_ERROR("Failed to call service sp4_control");
      all_success_flag = 106;
    }
    sleep(2);
    srv11.request.arg.clear();
    srv11.request.unit = 2;
    srv11.request.cmd = 15;
    srv11.request.arg.push_back(-40.0);     // 1
    srv11.request.arg.push_back(-4.0);      // 2
    srv11.request.arg.push_back(80.0);      // 3
    srv11.request.arg.push_back(75.0);      // 4
    srv11.request.arg.push_back(0.0);       // 5
    srv11.request.arg.push_back(8.23093);   // 6
    srv11.request.arg.push_back(-1.56232);  // 7
    srv11.request.arg.push_back(30.0);      // vel
    if (client11.call(srv11))
      ROS_INFO("result: %d", srv11.response.result);
    else
    {
      ROS_ERROR("Failed to call service sp4_control");
      all_success_flag = 106;
    }
    sleep(3);
    srv11.request.arg.clear();
    srv11.request.unit = 2;
    srv11.request.cmd = 15;
    srv11.request.arg.push_back(-1.51573);  // 1
    srv11.request.arg.push_back(-4.0);      // 2
    srv11.request.arg.push_back(6.4605);    // 3
    srv11.request.arg.push_back(1.09681);   // 4
    srv11.request.arg.push_back(0.0);       // 5
    srv11.request.arg.push_back(8.23093);   // 6
    srv11.request.arg.push_back(-1.56232);  // 7
    srv11.request.arg.push_back(30.0);      // vel
    if (client11.call(srv11))
      ROS_INFO("result: %d", srv11.response.result);
    else
    {
      ROS_ERROR("Failed to call service sp4_control");
      all_success_flag = 106;
    }

    sleep(5);

    //変化検出箇所へ戻す
    temp_command.x = 3000;
    temp_command.y = 2000;
    temp_command.th = 180;

    srv11.request.arg.clear();
    srv11.request.unit = 1;
    srv11.request.cmd = 15;
    srv11.request.arg.push_back(temp_command.x);
    srv11.request.arg.push_back(temp_command.y);
    srv11.request.arg.push_back(temp_command.th);
    if (client11.call(srv11))
    {
    }
    else
    {
      ROS_ERROR("Failed to call service smartpal_control_vehicle");
      all_success_flag = 107;
    }
    while (true)
    {
      srv11.request.arg.clear();
      srv11.request.unit = 1;
      srv11.request.cmd = 7;
      if (client11.call(srv11))
      {
        if (srv11.response.result != 16)
        {
          ROS_INFO("Smartpal BUSY %d", srv11.response.result);
          sleep(1);
        }
        else
        {
          break;
        }
      }
      else
      {
        ROS_ERROR("Failed to call service smartpal_control_get_state");
      }
    }

    //腕の位置を戻す
    //		srv11.request.arg.clear();
    //		srv11.request.unit = 2;
    //		srv11.request.cmd = 15;
    //		srv11.request.arg.push_back(20.0);	//1
    //		srv11.request.arg.push_back(-4.0);	//2
    //		srv11.request.arg.push_back(15.0);	//3
    //		srv11.request.arg.push_back(30.0);	//4
    //		srv11.request.arg.push_back(89.0);	//5
    //		srv11.request.arg.push_back(36.0);	//6
    //		srv11.request.arg.push_back(0.0);	//7
    //		srv11.request.arg.push_back(10.0);	//vel
    //
    //		if (client11.call(srv11))
    //			ROS_INFO("result: %d", srv11.response.result);
    //		else{
    //			ROS_ERROR("Failed to call service sp4_control");
    //			all_success_flag = 108;
    //		}
    //		srv11.request.arg.clear();
    //		srv11.request.unit = 2;
    //		srv11.request.cmd = 15;
    //		srv11.request.arg.push_back(0.0);	//1
    //		srv11.request.arg.push_back(-20.0);	//2
    //		srv11.request.arg.push_back(0.0);	//3
    //		srv11.request.arg.push_back(0.0);	//4
    //		srv11.request.arg.push_back(0.0);	//5
    //		srv11.request.arg.push_back(0.0);	//6
    //		srv11.request.arg.push_back(0.0);	//7
    //		srv11.request.arg.push_back(10.0);	//vel
    //
    //		if (client11.call(srv11))
    //			ROS_INFO("result: %d", srv11.response.result);
    //		else{
    //			ROS_ERROR("Failed to call service sp4_control");
    //			all_success_flag = 109;
    //		}

    //返却するのは見つかった物品のIDと見つかった家具ID
    //これらから起こった事象を推定する(人が持っていった、それが行われた時刻等を分析部分へ返す)
    ucTagData_sprh.resize(8);
    ucTagData_sprh[7] = 0;
    res.object_id.push_back(ucTagData_sprh[7]);
    res.furnitures_id.push_back(req.search_furnitures_id[i]);
  }

  // srvレスポンスの挿入
  //全て成功していればall_success_flagは初期値=0のまま
  if (all_success_flag == 0)
    res.success = 0;  // 0のときにsuccess
  //もしどこかで失敗してROS_ERRORが表示されて入れば
  // all_success_flagは初期値意外のデバッグ値になっている
  else
    res.success = all_success_flag;

  res.message = "ALL SEARCH WAS FINISHED";

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tms_to_find_objects_rev2");
  ros::NodeHandle n;
  ROS_INFO("tms_to_find_objects_rev2 : init");

  //	ros::Subscriber sub1 = n.subscribe("spfm_tag_data", 100, spfm_tag_data_callback);
  //	ros::Subscriber sub2 = n.subscribe("spbm_tag_data", 100, spbm_tag_data_callback);
  //	ros::Subscriber sub3 = n.subscribe("splh_tag_data", 100, splh_tag_data_callback);
  ros::Subscriber sub4 = n.subscribe("sprh_tag_data", 100, sprh_tag_data_callback);

  //入力タスク
  service1 = n.advertiseService("tms_ts_find_objects", tms_to_find_objects_func);

  // 0X ... TMSDBと
  client01 = n.serviceClient< tms_msg_db::tmsdb_get_furnitures_info >("tmsdb_get_furnitures_info");
  client02 = n.serviceClient< tms_msg_db::tmsdb_get_robots_info >("tmsdb_get_robots_info");
  client03 = n.serviceClient< tms_msg_db::tmsdb_get_pcd_info >("tmsdb_get_pcd_info");
  client04 = n.serviceClient< tms_msg_db::tmsdb_file_conservation >("tmsdb_file_conservation");
  client05 = n.serviceClient< tms_msg_db::tmsdb_objects_data >("tmsdb_objects_data_1");

  client1 = n.serviceClient< tms_msg_rp::rps_goal_planning >("rps_goal_planning");
  client2 = n.serviceClient< tms_msg_rp::rps_path_planning >("rps_path_planning");
  client3 = n.serviceClient< tms_msg_rp::rps_robot_drive >("rps_robot_drive");
  client4 = n.serviceClient< tms_msg_ss::ods_change_detection >("ods_change_dt");

  client8 = n.serviceClient< tms_msg_rp::rps_arm_drive >("rps_arm_drive");
  //	client2 = n.serviceClient<tmsdb::tmsdb_modify_person_behavior>("tmsdb_modify_person_behavior");
  //	client3 = n.serviceClient<tmsdb::tmsdb_modify_missing_objects>("tmsdb_modify_missing_objects");
  //	client9 = n.serviceClient<tms_msg_rc::tag_data>("rts_read_tag");

  client10 = n.serviceClient< tms_msg_rp::rps_command_to_move_smartpal >("rps_smartpal_path");
  client11 = n.serviceClient< tms_msg_rc::smartpal_control >("sp4_control");

  ROS_INFO("tms_to_find_objects_rev2 : wait");

  ros::spin();

  return 0;
}
