// include for ROS
#include <ros/ros.h>
#include <unistd.h>
#include <boost/date_time/local_time/local_time.hpp>

// include for TMSDB
#include <tms_msg_db/tmsdb_get_robots_info.h>

#include <tms_msg_sa/tms_to_activate_service.h>

// include for planning
#include <tms_msg_rp/rps_position.h>
#include <tms_msg_rp/rps_route.h>
#include <tms_msg_rp/rps_path_planning.h>

// include for vision
#include <tms_msg_ss/ods_person_detection.h>
#include <tms_msg_ss/ods_skincolor_extraction.h>
#include <tms_msg_ss/ods_face_detection.h>
#include <capture.h>

// include for pioneer3-AT
#include <tms_msg_rc/tms_rc_ppose.h>
#include <tms_msg_rc/tms_rc_pmove.h>
#include <tms_msg_rc/tms_rc_pparam.h>

//#define kangle_deg 28.5
// 22
#define kangle_deg 28.5
#define safe_dist 500.0
#define body_width 300.0

#define POS_ABER_XTION 225

ros::ServiceServer service1;

// client0X -> tmsdb_writer, tmsdb_readerとのやりとり
ros::ServiceClient client01;
ros::ServiceClient client02;
ros::ServiceClient client03;
ros::ServiceClient client04;
ros::ServiceClient client05;

// clientX -> SKILL partとのやりとり
ros::ServiceClient client1;
ros::ServiceClient client2;
ros::ServiceClient client3;
ros::ServiceClient client4;
ros::ServiceClient client5;
ros::ServiceClient client6;
ros::ServiceClient client7;
ros::ServiceClient client9;
ros::ServiceClient client10;
ros::ServiceClient client11;

typedef struct lpose
{
  double x;
  double y;
  double th;
};

double pinitx = 1000;
double pinity = 1000;
double pinitth = 0;

//一定サイズ以上のクラスタ1つに対して発動
//クラスタの端点でxが小さい方を(x1,y1)、大きい方を(x2,y2)とする
// goal_array[G1,G2,G3,G4,G5,G6]
// G1,G4が全身用、G1に対してG2とG3が顔認識用、G4に対してG5とG6が顔認識用
// LRFに近い方がG1,遠い方がG4
// G1,G4各々について、人に向かって左がG2,G5、向かって右がG3,G6
// goal_array[0~5]にG1~G6に対するロボットの座標と姿勢を入れる
void calc_goal_pos(double x1, double y1, double x2, double y2, lpose goal_array[])
{
  FILE *fp;
  bool fileflag = true;

  double th_radian = 0.0;
  double th_degree = 0.0;
  double clen = 0.0;
  double kangle = 0.0;

  if ((fp = fopen("goal_point.txt", "a")) == NULL)
  {
    ROS_INFO("FILE OPEN ERROR");
    fileflag = false;
  }

  th_radian = atan2(y2 - y1, x2 - x1);
  th_degree = (180.0 * th_radian) / M_PI;

  clen = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
  kangle = (M_PI * kangle_deg) / 180;

  // if(th_degree > 0.0){	//傾き正の場合
  goal_array[0].x = (x1 + x2) / 2 + (clen / (2 * tan(kangle))) * cos(th_radian - (M_PI / 2));  // G1.x
  ROS_INFO("G1.x=%lf", goal_array[0].x);
  goal_array[0].y = (y1 + y2) / 2 + (clen / (2 * tan(kangle))) * sin(th_radian - (M_PI / 2));  // G1.y
  ROS_INFO("G1.y=%lf", goal_array[0].y);
  goal_array[1].x = x1 + safe_dist * cos(th_radian - (M_PI / 2));  // G2.x
  ROS_INFO("G2.x=%lf", goal_array[1].x);
  goal_array[1].y = y1 + safe_dist * sin(th_radian - (M_PI / 2));  // G2.y
  ROS_INFO("G2.y=%lf", goal_array[1].y);
  goal_array[2].x = x2 + safe_dist * cos(th_radian - (M_PI / 2));  // G3.x
  ROS_INFO("G3.x=%lf", goal_array[2].x);
  goal_array[2].y = y2 + safe_dist * sin(th_radian - (M_PI / 2));  // G3.y
  ROS_INFO("G3.y=%lf", goal_array[2].y);
  goal_array[0].th = goal_array[1].th = goal_array[2].th = th_degree + 90.0;
  ROS_INFO("G1~G3.th=%lf", goal_array[0].th);

  goal_array[3].x = (x1 + x2) / 2 + ((clen / (2 * tan(kangle))) + body_width) * cos(th_radian + (M_PI / 2));
  ROS_INFO("G4.x=%lf", goal_array[3].x);
  goal_array[3].y = (y1 + y2) / 2 + ((clen / (2 * tan(kangle))) + body_width) * sin(th_radian + (M_PI / 2));
  ROS_INFO("G4.y=%lf", goal_array[3].y);
  goal_array[5].x = x1 + (safe_dist + body_width) * cos(th_radian + (M_PI / 2));
  ROS_INFO("G5.x=%lf", goal_array[5].x);
  goal_array[5].y = y1 + (safe_dist + body_width) * sin(th_radian + (M_PI / 2));
  ROS_INFO("G5.y=%lf", goal_array[5].y);
  goal_array[4].x = x2 + (safe_dist + body_width) * cos(th_radian + (M_PI / 2));
  ROS_INFO("G6.x=%lf", goal_array[4].x);
  goal_array[4].y = y2 + (safe_dist + body_width) * sin(th_radian + (M_PI / 2));
  ROS_INFO("G6.y=%lf", goal_array[4].y);
  goal_array[3].th = goal_array[4].th = goal_array[5].th = th_degree - 90.0;
  ROS_INFO("G4~G6.th=%lf", goal_array[3].th);
  //}

  if (fileflag)
  {
    fprintf(fp, "%lf,%lf,%lf\n"
                "%lf,%lf,%lf\n"
                "%lf,%lf,%lf\n"
                "%lf,%lf,%lf\n"
                "%lf,%lf,%lf\n"
                "%lf,%lf,%lf\n\n",
            goal_array[0].x, goal_array[0].y, goal_array[0].th, goal_array[1].x, goal_array[1].y, goal_array[1].th,
            goal_array[2].x, goal_array[2].y, goal_array[2].th, goal_array[3].x, goal_array[3].y, goal_array[3].th,
            goal_array[4].x, goal_array[4].y, goal_array[4].th, goal_array[5].x, goal_array[5].y, goal_array[5].th);
    fclose(fp);
  }
}

bool tms_rs_fall_detection_func(tms_msg_sa::tms_to_activate_service::Request &req,
                                tms_msg_sa::tms_to_activate_service::Response &res)
{
  tms_msg_db::tmsdb_get_robots_info srv01;  // get robots info

  tms_msg_rc::tms_rc_pmove srv1;
  tms_msg_rc::tms_rc_pparam srv2;

  tms_msg_rp::rps_path_planning srv3;
  tms_msg_ss::ods_person_detection srv4;
  tms_msg_ss::ods_skincolor_extraction srv5;
  tms_msg_ss::ods_face_detection srv6;

  // tms_msg_ss::ods_face_detection srv_temp;

  double cluster_central_x, cluster_central_y, cluster_central_th = 0.0;

  double x1, y1, x2, y2;
  lpose pose[6];

  tms_msg_rc::tms_rc_pmove::Request pmove;
  cv_bridge::CvImagePtr cv_ptr;  // OpenCVを用いてメッセージを画像化する

  bool find_person = false;
  bool directmove = true;  //今は必ずdirectmoveになるようにしています
  int directmove_int = 0;

  //////////GLOBAL座標系での転倒した人の端点の設定//////////

  //クラスタの概方向を取得する
  //現在座標(pinitx,pinity,pinitth)、クラスタ座標(cx,cy)とすると…global角度を求めて現在角度から変換
  //ここは本来TMS_SAにおいて分析されているため，DBから情報を取得するだけでよい
  ROS_INFO("input cluster's central position");
  scanf("%lf %lf %lf", &cluster_central_x, &cluster_central_y, &cluster_central_th);

  //ロボット座標をsrv01として取得
  srv01.request.robots_id = 5;
  if (client01.call(srv01))
  {
    ROS_INFO("robotid=%u:(%f,%f,%f),state=%u,place=%u", srv01.request.robots_id, srv01.response.robots_x,
             srv01.response.robots_y, srv01.response.robots_theta, srv01.response.robots_state,
             srv01.response.robots_place);
  }
  else
  {
    ROS_ERROR("Failed to call service tmsdb_get_robots_info");
  }

  //回転動作:人っぽいクラスタのだいたいの方向を向く
  //場所によっては回転だけで視野に入らない場合もあるので，その場合は動作計画->視野に入る場所への移動が必要
  srv1.request.command = 2;
  srv1.request.pangle =
      ((atan2(cluster_central_y - srv01.response.robots_y, cluster_central_x - srv01.response.robots_x) * 180) / M_PI) -
      srv01.response.robots_theta;

  if (srv1.request.pangle > 180)
    srv1.request.pangle = srv1.request.pangle - 360;
  if (srv1.request.pangle < -180)
    ROS_INFO("spin robot : %f", srv1.request.pangle);
  srv1.request.pangle = srv1.request.pangle + 360;

  if (client1.call(srv1))
  {
    ROS_INFO("Spin : end");
  }
  else
  {
    ROS_ERROR("Failed to call service pioneer_pmove");
  }

  sleep(2);  // DB書き換えまで少々待つ

  //ロボット座標をsrv01として取得
  srv01.request.robots_id = 5;
  if (client01.call(srv01))
  {
    ROS_INFO("robotid=%u:(%f,%f,%f),state=%u,place=%u", srv01.request.robots_id, srv01.response.robots_x,
             srv01.response.robots_y, srv01.response.robots_theta, srv01.response.robots_state,
             srv01.response.robots_place);
  }
  else
  {
    ROS_ERROR("Failed to call service tmsdb_get_robots_info");
  }

  ROS_INFO("sleep 3 sec.");
  sleep(3);

  //	goto CALC_ROBOT_GOAL;

  // Xtionを利用
  //端点の相対座標を出力
  if (client4.call(srv4))
  {
    ROS_INFO("p1(%f,%f,%f), p2(%f,%f,%f)", srv4.response.p1_x, srv4.response.p1_y, srv4.response.p1_z,
             srv4.response.p2_x, srv4.response.p2_y, srv4.response.p2_z);
    std::cout << srv4.response.image.encoding << std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service ods_person_detection");
  }

  // goto RESULT;

  //端点のGLOBAL座標を計算

  // XtionのZ軸をロボットのX軸上におく
  // XtionのX軸はロボットのX軸からPOS_ABER_XTION離れているとすると…

  x1 = srv01.response.robots_x +
       (POS_ABER_XTION + srv4.response.p1_z) * cos((M_PI * srv01.response.robots_theta) / 180) -
       (-1 * (srv4.response.p1_x)) * sin((M_PI * srv01.response.robots_theta) / 180);
  y1 = srv01.response.robots_y +
       (POS_ABER_XTION + srv4.response.p1_z) * sin((M_PI * srv01.response.robots_theta) / 180) +
       (-1 * (srv4.response.p1_x)) * cos((M_PI * srv01.response.robots_theta) / 180);

  x2 = srv01.response.robots_x +
       (POS_ABER_XTION + srv4.response.p2_z) * cos((M_PI * srv01.response.robots_theta) / 180) -
       (-1 * (srv4.response.p2_x)) * sin((M_PI * srv01.response.robots_theta) / 180);
  y2 = srv01.response.robots_y +
       (POS_ABER_XTION + srv4.response.p2_z) * sin((M_PI * srv01.response.robots_theta) / 180) +
       (-1 * (srv4.response.p2_x)) * cos((M_PI * srv01.response.robots_theta) / 180);

  ROS_INFO("p1(%lf,%lf),p2(%lf,%lf)", x1, y1, x2, y2);

CALC_ROBOT_GOAL:

  //////////GLOBAL座標系での転倒した人の端点の設定ここまで//////////

  //	ROS_INFO("input x1 y1 x2 y2");
  //	scanf("%lf %lf %lf %lf", &x1,&y1,&x2,&y2);

  /////////端点から目標位置の決定/////////
  calc_goal_pos(x1, y1, x2, y2, pose);

  //ここでG1とG4のどちらへ向かうかを決定する
  //ロボットの現在位置に近い方から向かう
  //もし逆になっているときは、G1<->G4、G2<->G5、G3<->G6と入れ替える処理を挿入
  //すなわち、temp = pose[0];pose[3] = pose[0];pose[0] = temp;
  //こうすることで探索順が必ずpose[0]からになる
  //ただし、現状ではXtionで端点を発見する都合上、G4よりもG1の方が近い(ロボット:G1:人:G4)の順番になる
  //すなわち、このif文に入るのはodd patternである
  if (sqrt(((pose[0].x - srv01.response.robots_x) * (pose[0].x - srv01.response.robots_x)) +
           ((pose[0].y - srv01.response.robots_y) * (pose[0].y - srv01.response.robots_y))) >
      sqrt(((pose[3].x - srv01.response.robots_x) * (pose[3].x - srv01.response.robots_x)) +
           ((pose[3].y - srv01.response.robots_y) * (pose[3].y - srv01.response.robots_y))))
  {
    ROS_INFO("odd pattern was occer");
    lpose temp;
    temp = pose[0];
    pose[0] = pose[3];
    pose[3] = temp;
    temp = pose[1];
    pose[1] = pose[4];
    pose[4] = temp;
    temp = pose[2];
    pose[2] = pose[5];
    pose[5] = temp;
  }

  //	goto BREAK;		//現状ここでブレーク(動作確認用)
  //	ROS_INFO("goto SKINCOLOR_A");
  //	goto SKINCOLOR_A;

  /////////////////////////ここから移動////////////////////////////////

  // robot_id = "KKP"
  srv3.request.robot_id = 5;
  srv3.request.rps_goal_candidate.rps_route.resize(1);

SKINCOLOR_A:

  for (unsigned int G_i = 0; G_i < 2; G_i = G_i + 3)
  {
    //		ROS_INFO("goto SKINCOLOR_B");
    //		goto SKINCOLOR_B;
    // pose[0]へ移動 -> pose[3]へ移動
    srv3.request.rps_goal_candidate.rps_route.front().x = pose[G_i].x;
    srv3.request.rps_goal_candidate.rps_route.front().y = pose[G_i].y;
    srv3.request.rps_goal_candidate.rps_route.front().th = pose[G_i].th;

    if (client3.call(srv3))
    {
      if (srv3.response.success == 1)
      {
        ROS_INFO("success = %d", srv3.response.success);
      }
      else
      {
        ROS_INFO("failed_ID = %d: '%s'", srv3.response.success, srv3.response.message.c_str());
      }
    }
    else
    {
      ROS_ERROR("Failed to call service rps_path_planning");
    }

    // pose[0]へ移動できない場合は移動不可能
    //もう片方のpose[3]へ移動する：Goto Greverse
    if (srv3.response.success == 0)
    {
      ROS_INFO("continue");
      continue;  // G_i = G_i+3;
    }

  //移動：Pioneerを全身撮影用目標地点(G1orG4)へ動作させる
  //まずは直進回転動作への変換を行い、次にそれに沿って動作を行う
  SKINCOLOR_B:
    //		for(unsigned int i=0;i<9-1;i++){
    for (unsigned int i = 0; i < srv3.response.rps_path.front().rps_route.size() - 1; i++)
    {
      //			ROS_INFO("goto SKINCOLOR_Cclea");
      //			goto SKINCOLOR_C;
      if (abs((srv3.response.rps_path.front().rps_route[i + 1].th - srv3.response.rps_path.front().rps_route[i].th)) <
          0.01)
      {                            //回転角0.01度未満であれば直進動作用とみなす
        srv1.request.command = 1;  //直進
        srv1.request.pdist = sqrt(
            (srv3.response.rps_path.front().rps_route[i + 1].x - srv3.response.rps_path.front().rps_route[i].x) *
                (srv3.response.rps_path.front().rps_route[i + 1].x - srv3.response.rps_path.front().rps_route[i].x) +
            ((srv3.response.rps_path.front().rps_route[i + 1].y - srv3.response.rps_path.front().rps_route[i].y) *
             (srv3.response.rps_path.front().rps_route[i + 1].y - srv3.response.rps_path.front().rps_route[i].y)));

        //移動
        if (client1.call(srv1))
        {
        }
        else
        {
          ROS_ERROR("Failed to call service pioneer_pmove");
        }
        //				srv2.request.command = 1;
        //				while(1){
        //					if(client2.call(srv2)){
        //						//ROS_INFO("is_pmove = %u, is_pspin = %u", srv2.response.ispmove, srv2.response.ispspin);
        //						if(srv2.response.ispmove == 0)
        //							break;
        //					}
        //					sleep(1);
        //				}
      }
      else
      {
        srv1.request.command = 2;  //回転
        srv1.request.pangle =
            srv3.response.rps_path.front().rps_route[i + 1].th - srv3.response.rps_path.front().rps_route[i].th;

        if (client1.call(srv1))
        {
        }
        else
        {
          ROS_ERROR("Failed to call service pioneer_pmove");
        }
        //				srv2.request.command = 2;
        //				while(1){
        //					if(client2.call(srv2)){
        //						//ROS_INFO("is_pmove = %u, is_pspin = %u", srv2.response.ispmove, srv2.response.ispspin);
        //						if(srv2.response.ispspin == 0)
        //							break;
        //					}
        //					sleep(1);
        //				}
      }
    }

  //		goto BREAK;

  SKINCOLOR_C:

    //肌色検出：
    // G2かG3へ移動 / G4かG5へ移動
    if (client5.call(srv5))
    {
    }
    else
    {
      ROS_ERROR("Failed to call service pioneer_pmove");
    }

    //		goto FACE_DETECT;

    if (srv5.response.direction == 2)
    {  //肌色検出が左
      ROS_INFO("face color: left");

      srv3.request.rps_goal_candidate.rps_route.front().x = pose[G_i + 1].x;
      srv3.request.rps_goal_candidate.rps_route.front().y = pose[G_i + 1].y;
      srv3.request.rps_goal_candidate.rps_route.front().th = pose[G_i + 1].th;

      if (client3.call(srv3))
      {
        if (srv3.response.success == 1)
        {
          ROS_INFO("success = %d", srv3.response.success);
        }
        else
        {
          ROS_INFO("failed_ID = %d: '%s'", srv3.response.success, srv3.response.message.c_str());
          directmove = true;
        }
      }
      else
      {
        ROS_ERROR("Failed to call service rps_path_planning");
        directmove = true;
      }

      if (directmove == false)
      {
        //移動：Pioneerを顔撮影用目標地点(G2/G5)へ動作させる
        //まずは直進回転動作への変換を行い、次にそれに沿って動作を行う
        for (unsigned int i = 0; i < srv3.response.rps_path.front().rps_route.size() - 1; i++)
        {
          if (abs((srv3.response.rps_path.front().rps_route[i + 1].th -
                   srv3.response.rps_path.front().rps_route[i].th)) < 0.01)
          {                            //回転角0.01度未満であれば直進動作用とみなす
            srv1.request.command = 1;  //直進
            srv1.request.pdist = sqrt(
                (srv3.response.rps_path.front().rps_route[i + 1].x - srv3.response.rps_path.front().rps_route[i].x) *
                    (srv3.response.rps_path.front().rps_route[i + 1].x -
                     srv3.response.rps_path.front().rps_route[i].x) +
                ((srv3.response.rps_path.front().rps_route[i + 1].y - srv3.response.rps_path.front().rps_route[i].y) *
                 (srv3.response.rps_path.front().rps_route[i + 1].y - srv3.response.rps_path.front().rps_route[i].y)));

            //移動
            if (client1.call(srv1))
            {
            }
            else
            {
              ROS_ERROR("Failed to call service pioneer_pmove");
            }
          }
          else
          {
            srv1.request.command = 2;  //回転
            srv1.request.pangle =
                srv3.response.rps_path.front().rps_route[i + 1].th - srv3.response.rps_path.front().rps_route[i].th;

            if (client1.call(srv1))
            {
            }
            else
            {
              ROS_ERROR("Failed to call service pioneer_pmove");
            }
          }
        }
      }
      else
      {
        ROS_INFO("DIRECT MOVE? / 0:NOT MOVE else:MOVE");
        scanf("%d", &directmove_int);
        if (directmove != 0)
        {
          srv01.request.robots_id = 5;
          if (client01.call(srv01))
          {
            ROS_INFO("robotid=%u:(%f,%f,%f),state=%u,place=%u", srv01.request.robots_id, srv01.response.robots_x,
                     srv01.response.robots_y, srv01.response.robots_theta, srv01.response.robots_state,
                     srv01.response.robots_place);
          }
          else
          {
            ROS_ERROR("Failed to call service tmsdb_get_robots_info");
          }

          srv1.request.command = 2;
          srv1.request.pangle = ((atan2(pose[G_i + 1].y - pose[G_i].y, pose[G_i + 1].x - pose[G_i].x) * 180) / M_PI) -
                                srv01.response.robots_theta;

          if (srv1.request.pangle > 180)
            srv1.request.pangle = srv1.request.pangle - 360;
          if (srv1.request.pangle < -180)
            srv1.request.pangle = srv1.request.pangle + 360;

          ROS_INFO("spin robot : %f", srv1.request.pangle);
          if (client1.call(srv1))
          {
          }
          else
          {
            ROS_ERROR("Failed to call service pioneer_pmove");
          }

          srv01.request.robots_id = 5;
          if (client01.call(srv01))
          {
            ROS_INFO("robotid=%u:(%f,%f,%f),state=%u,place=%u", srv01.request.robots_id, srv01.response.robots_x,
                     srv01.response.robots_y, srv01.response.robots_theta, srv01.response.robots_state,
                     srv01.response.robots_place);
          }
          else
          {
            ROS_ERROR("Failed to call service tmsdb_get_robots_info");
          }

          srv1.request.command = 1;
          srv1.request.pdist = sqrt(pow(pose[G_i + 1].x - pose[G_i].x, 2) + pow(pose[G_i + 1].y - pose[G_i].y, 2));
          ROS_INFO("go robot : %f", srv1.request.pdist);
          if (client1.call(srv1))
          {
          }
          else
          {
            ROS_ERROR("Failed to call service pioneer_pmove");
          }

          srv01.request.robots_id = 5;
          if (client01.call(srv01))
          {
            ROS_INFO("robotid=%u:(%f,%f,%f),state=%u,place=%u", srv01.request.robots_id, srv01.response.robots_x,
                     srv01.response.robots_y, srv01.response.robots_theta, srv01.response.robots_state,
                     srv01.response.robots_place);
          }
          else
          {
            ROS_ERROR("Failed to call service tmsdb_get_robots_info");
          }

          srv1.request.command = 2;
          srv1.request.pangle = pose[G_i + 1].th - srv01.response.robots_theta;

          if (srv1.request.pangle > 180)
            srv1.request.pangle = srv1.request.pangle - 360;
          if (srv1.request.pangle < -180)
            srv1.request.pangle = srv1.request.pangle + 360;

          ROS_INFO("spin robot : %f", srv1.request.pangle);
          if (client1.call(srv1))
          {
          }
          else
          {
            ROS_ERROR("Failed to call service pioneer_pmove");
          }
        }
        //				directmove = false;
        directmove_int = 0;
      }

    FACE_DETECT:
      //			unsigned int temp;
      //			scanf("%u", &temp);

      //顔検出モジュール起動
      if (client6.call(srv6))
      {
      }
      else
      {
        ROS_ERROR("Failed to call service ods_face_detection");
      }

      if (srv6.response.result == 1)
      {  //顔検出：顔が見つかった場合
        ROS_INFO("face detected.(G1-G2/G4-G5);pose[%d]", G_i + 1);
        find_person = true;
        break;
      }
      else
      {  //顔検出：顔が見つからなかった場合

        ROS_INFO("face not detected.(G1-G2-/G4-G5-);pose[%d]", G_i + 1);

        //右へ移動する
        srv3.request.rps_goal_candidate.rps_route.front().x = pose[G_i + 2].x;
        srv3.request.rps_goal_candidate.rps_route.front().y = pose[G_i + 2].y;
        srv3.request.rps_goal_candidate.rps_route.front().th = pose[G_i + 2].th;

        if (client3.call(srv3))
        {
          if (srv3.response.success == 1)
          {
            ROS_INFO("success = %d", srv3.response.success);
          }
          else
          {
            ROS_INFO("failed_ID = %d: '%s'", srv3.response.success, srv3.response.message.c_str());
            directmove = true;
          }
        }
        else
        {
          ROS_ERROR("Failed to call service rps_path_planning");
          directmove = true;
        }

        if (directmove == false)
        {
          //移動：Pioneerをもう一方の顔撮影用目標地点(G3/G6)へ動作させる
          //まずは直進回転動作への変換を行い、次にそれに沿って動作を行う
          for (unsigned int i = 0; i < srv3.response.rps_path.front().rps_route.size() - 1; i++)
          {
            if (abs((srv3.response.rps_path.front().rps_route[i + 1].th -
                     srv3.response.rps_path.front().rps_route[i].th)) < 0.01)
            {                            //回転角0.01度未満であれば直進動作用とみなす
              srv1.request.command = 1;  //直進
              srv1.request.pdist = sqrt(
                  (srv3.response.rps_path.front().rps_route[i + 1].x - srv3.response.rps_path.front().rps_route[i].x) *
                      (srv3.response.rps_path.front().rps_route[i + 1].x -
                       srv3.response.rps_path.front().rps_route[i].x) +
                  ((srv3.response.rps_path.front().rps_route[i + 1].y - srv3.response.rps_path.front().rps_route[i].y) *
                   (srv3.response.rps_path.front().rps_route[i + 1].y - srv3.response.rps_path.front().rps_route[i].y)));

              //移動
              if (client1.call(srv1))
              {
              }
              else
              {
                ROS_ERROR("Failed to call service pioneer_pmove");
              }
            }
            else
            {
              srv1.request.command = 2;  //回転
              srv1.request.pangle =
                  srv3.response.rps_path.front().rps_route[i + 1].th - srv3.response.rps_path.front().rps_route[i].th;

              if (client1.call(srv1))
              {
              }
              else
              {
                ROS_ERROR("Failed to call service pioneer_pmove");
              }
            }
          }
        }
        else
        {
          ROS_INFO("DIRECT MOVE? / 0:NOT MOVE else:MOVE");
          scanf("%d", &directmove_int);
          if (directmove_int != 0)
          {
            srv01.request.robots_id = 5;
            if (client01.call(srv01))
            {
              ROS_INFO("robotid=%u:(%f,%f,%f),state=%u,place=%u", srv01.request.robots_id, srv01.response.robots_x,
                       srv01.response.robots_y, srv01.response.robots_theta, srv01.response.robots_state,
                       srv01.response.robots_place);
            }
            else
            {
              ROS_ERROR("Failed to call service tmsdb_get_robots_info");
            }

            srv1.request.command = 2;
            srv1.request.pangle =
                ((atan2(pose[G_i + 2].y - pose[G_i + 1].y, pose[G_i + 2].x - pose[G_i + 1].x) * 180) / M_PI) -
                srv01.response.robots_theta;

            if (srv1.request.pangle > 180)
              srv1.request.pangle = srv1.request.pangle - 360;
            if (srv1.request.pangle < -180)
              srv1.request.pangle = srv1.request.pangle + 360;

            ROS_INFO("spin robot : %f", srv1.request.pangle);
            if (client1.call(srv1))
            {
            }
            else
            {
              ROS_ERROR("Failed to call service pioneer_pmove");
            }

            srv01.request.robots_id = 5;
            if (client01.call(srv01))
            {
              ROS_INFO("robotid=%u:(%f,%f,%f),state=%u,place=%u", srv01.request.robots_id, srv01.response.robots_x,
                       srv01.response.robots_y, srv01.response.robots_theta, srv01.response.robots_state,
                       srv01.response.robots_place);
            }
            else
            {
              ROS_ERROR("Failed to call service tmsdb_get_robots_info");
            }

            srv1.request.command = 1;
            srv1.request.pdist =
                sqrt(pow(pose[G_i + 2].x - pose[G_i + 1].x, 2) + pow(pose[G_i + 2].y - pose[G_i + 1].y, 2));
            ROS_INFO("go robot : %f", srv1.request.pdist);
            if (client1.call(srv1))
            {
            }
            else
            {
              ROS_ERROR("Failed to call service pioneer_pmove");
            }

            srv01.request.robots_id = 5;
            if (client01.call(srv01))
            {
              ROS_INFO("robotid=%u:(%f,%f,%f),state=%u,place=%u", srv01.request.robots_id, srv01.response.robots_x,
                       srv01.response.robots_y, srv01.response.robots_theta, srv01.response.robots_state,
                       srv01.response.robots_place);
            }
            else
            {
              ROS_ERROR("Failed to call service tmsdb_get_robots_info");
            }

            srv1.request.command = 2;
            srv1.request.pangle = pose[G_i + 2].th - srv01.response.robots_theta;

            if (srv1.request.pangle > 180)
              srv1.request.pangle = srv1.request.pangle - 360;
            if (srv1.request.pangle < -180)
              srv1.request.pangle = srv1.request.pangle + 360;

            ROS_INFO("spin robot : %f", srv1.request.pangle);
            if (client1.call(srv1))
            {
            }
            else
            {
              ROS_ERROR("Failed to call service pioneer_pmove");
            }
          }
          directmove = false;
          directmove_int = 0;
        }

        if (client6.call(srv6))
        {
        }
        else
        {
          ROS_ERROR("Failed to call service ods_face_detection");
        }

        if (srv6.response.result == 1)
        {  //顔検出：顔が見つかった場合
          ROS_INFO("face detected.(G1-G2-G3/G4-G5-G6);pose[%d]", G_i + 2);
          find_person = true;
          break;
        }
        else
        {  //顔検出：顔が見つからなかった場合
          ROS_INFO("face not detected.(G1-G2-G3-/G4-G5-G6-);pose[%d],pose[%d]", G_i + 1, G_i + 2);
        }
      }
    }
    else
    {  //肌色検出が右(srv5.response.direction == 1)
      ROS_INFO("face color: right");

      srv3.request.rps_goal_candidate.rps_route.front().x = pose[G_i + 2].x;
      srv3.request.rps_goal_candidate.rps_route.front().y = pose[G_i + 2].y;
      srv3.request.rps_goal_candidate.rps_route.front().th = pose[G_i + 2].th;

      if (client3.call(srv3))
      {
        if (srv3.response.success == 1)
        {
          ROS_INFO("success = %d", srv3.response.success);
        }
        else
        {
          ROS_INFO("failed_ID = %d: '%s'", srv3.response.success, srv3.response.message.c_str());
          directmove = true;
        }
      }
      else
      {
        ROS_ERROR("Failed to call service rps_path_planning");
        directmove = true;
      }

      if (directmove == false)
      {
        //移動：Pioneerを顔撮影用目標地点(G3/G6)へ動作させる
        //まずは直進回転動作への変換を行い、次にそれに沿って動作を行う
        for (unsigned int i = 0; i < srv3.response.rps_path.front().rps_route.size() - 1; i++)
        {
          if (abs((srv3.response.rps_path.front().rps_route[i + 1].th -
                   srv3.response.rps_path.front().rps_route[i].th)) < 0.01)
          {                            //回転角0.01度未満であれば直進動作用とみなす
            srv1.request.command = 1;  //直進
            srv1.request.pdist = sqrt(
                (srv3.response.rps_path.front().rps_route[i + 1].x - srv3.response.rps_path.front().rps_route[i].x) *
                    (srv3.response.rps_path.front().rps_route[i + 1].x -
                     srv3.response.rps_path.front().rps_route[i].x) +
                ((srv3.response.rps_path.front().rps_route[i + 1].y - srv3.response.rps_path.front().rps_route[i].y) *
                 (srv3.response.rps_path.front().rps_route[i + 1].y - srv3.response.rps_path.front().rps_route[i].y)));

            //移動
            if (client1.call(srv1))
            {
            }
            else
            {
              ROS_ERROR("Failed to call service pioneer_pmove");
            }
          }
          else
          {
            srv1.request.command = 2;  //回転
            srv1.request.pangle =
                srv3.response.rps_path.front().rps_route[i + 1].th - srv3.response.rps_path.front().rps_route[i].th;

            if (client1.call(srv1))
            {
            }
            else
            {
              ROS_ERROR("Failed to call service pioneer_pmove");
            }
          }
        }
      }
      else
      {
        ROS_INFO("DIRECT MOVE? / 0:NOT MOVE else:MOVE");
        scanf("%d", &directmove_int);
        if (directmove_int != 0)
        {
          srv01.request.robots_id = 5;
          if (client01.call(srv01))
          {
            ROS_INFO("robotid=%u:(%f,%f,%f),state=%u,place=%u", srv01.request.robots_id, srv01.response.robots_x,
                     srv01.response.robots_y, srv01.response.robots_theta, srv01.response.robots_state,
                     srv01.response.robots_place);
          }
          else
          {
            ROS_ERROR("Failed to call service tmsdb_get_robots_info");
          }

          srv1.request.command = 2;
          srv1.request.pangle = ((atan2(pose[G_i + 2].y - pose[G_i].y, pose[G_i + 2].x - pose[G_i].x) * 180) / M_PI) -
                                srv01.response.robots_theta;

          if (srv1.request.pangle > 180)
            srv1.request.pangle = srv1.request.pangle - 360;
          if (srv1.request.pangle < -180)
            srv1.request.pangle = srv1.request.pangle + 360;

          ROS_INFO("spin robot : %f", srv1.request.pangle);
          if (client1.call(srv1))
          {
          }
          else
          {
            ROS_ERROR("Failed to call service pioneer_pmove");
          }

          srv01.request.robots_id = 5;
          if (client01.call(srv01))
          {
            ROS_INFO("robotid=%u:(%f,%f,%f),state=%u,place=%u", srv01.request.robots_id, srv01.response.robots_x,
                     srv01.response.robots_y, srv01.response.robots_theta, srv01.response.robots_state,
                     srv01.response.robots_place);
          }
          else
          {
            ROS_ERROR("Failed to call service tmsdb_get_robots_info");
          }

          srv1.request.command = 1;
          srv1.request.pdist = sqrt(pow(pose[G_i + 2].x - pose[G_i].x, 2) + pow(pose[G_i + 2].y - pose[G_i].y, 2));
          ROS_INFO("go robot : %f", srv1.request.pdist);
          if (client1.call(srv1))
          {
          }
          else
          {
            ROS_ERROR("Failed to call service pioneer_pmove");
          }

          srv01.request.robots_id = 5;
          if (client01.call(srv01))
          {
            ROS_INFO("robotid=%u:(%f,%f,%f),state=%u,place=%u", srv01.request.robots_id, srv01.response.robots_x,
                     srv01.response.robots_y, srv01.response.robots_theta, srv01.response.robots_state,
                     srv01.response.robots_place);
          }
          else
          {
            ROS_ERROR("Failed to call service tmsdb_get_robots_info");
          }

          srv1.request.command = 2;
          srv1.request.pangle = pose[G_i + 2].th - srv01.response.robots_theta;

          if (srv1.request.pangle > 180)
            srv1.request.pangle = srv1.request.pangle - 360;
          if (srv1.request.pangle < -180)
            srv1.request.pangle = srv1.request.pangle + 360;

          ROS_INFO("spin robot : %f", srv1.request.pangle);
          if (client1.call(srv1))
          {
          }
          else
          {
            ROS_ERROR("Failed to call service pioneer_pmove");
          }
        }
        directmove = false;
        directmove_int = 0;
      }

      //顔検出RIGHT側
      if (client6.call(srv6))
      {
      }
      else
      {
        ROS_ERROR("Failed to call service ods_face_detection");
      }

      if (srv6.response.result == 1)
      {  //顔検出：顔が見つかった場合
        ROS_INFO("face detected.(G1-G3/G4-G6);pose[%d]", G_i + 2);
        find_person = true;
        break;
      }
      else
      {  //顔検出：顔が見つからなかった場合

        ROS_INFO("face not detected.(G1-G3-/G4-G6-);pose[%d]", G_i + 2);

        //左へ移動する
        srv3.request.rps_goal_candidate.rps_route.front().x = pose[G_i + 1].x;
        srv3.request.rps_goal_candidate.rps_route.front().y = pose[G_i + 1].y;
        srv3.request.rps_goal_candidate.rps_route.front().th = pose[G_i + 1].th;

        if (client3.call(srv3))
        {
          if (srv3.response.success == 1)
          {
            ROS_INFO("success = %d", srv3.response.success);
          }
          else
          {
            ROS_INFO("failed_ID = %d: '%s'", srv3.response.success, srv3.response.message.c_str());
            directmove = true;
          }
        }
        else
        {
          ROS_ERROR("Failed to call service rps_path_planning");
          directmove = true;
        }

        if (directmove == false)
        {
          //移動：Pioneerをもう一方の顔撮影用目標地点(G2/G5)へ動作させる
          //まずは直進回転動作への変換を行い、次にそれに沿って動作を行う
          for (unsigned int i = 0; i < srv3.response.rps_path.front().rps_route.size() - 1; i++)
          {
            if (abs((srv3.response.rps_path.front().rps_route[i + 1].th -
                     srv3.response.rps_path.front().rps_route[i].th)) < 0.01)
            {                            //回転角0.01度未満であれば直進動作用とみなす
              srv1.request.command = 1;  //直進
              srv1.request.pdist = sqrt(
                  (srv3.response.rps_path.front().rps_route[i + 1].x - srv3.response.rps_path.front().rps_route[i].x) *
                      (srv3.response.rps_path.front().rps_route[i + 1].x -
                       srv3.response.rps_path.front().rps_route[i].x) +
                  ((srv3.response.rps_path.front().rps_route[i + 1].y - srv3.response.rps_path.front().rps_route[i].y) *
                   (srv3.response.rps_path.front().rps_route[i + 1].y - srv3.response.rps_path.front().rps_route[i].y)));

              //移動
              if (client1.call(srv1))
              {
              }
              else
              {
                ROS_ERROR("Failed to call service pioneer_pmove");
              }
            }
            else
            {
              srv1.request.command = 2;  //回転
              srv1.request.pangle =
                  srv3.response.rps_path.front().rps_route[i + 1].th - srv3.response.rps_path.front().rps_route[i].th;

              if (client1.call(srv1))
              {
              }
              else
              {
                ROS_ERROR("Failed to call service pioneer_pmove");
              }
            }
          }
        }
        else
        {
          ROS_INFO("DIRECT MOVE? / 0:NOT MOVE else:MOVE");
          scanf("%d", &directmove_int);
          if (directmove != 0)
          {
            srv01.request.robots_id = 5;
            if (client01.call(srv01))
            {
              ROS_INFO("robotid=%u:(%f,%f,%f),state=%u,place=%u", srv01.request.robots_id, srv01.response.robots_x,
                       srv01.response.robots_y, srv01.response.robots_theta, srv01.response.robots_state,
                       srv01.response.robots_place);
            }
            else
            {
              ROS_ERROR("Failed to call service tmsdb_get_robots_info");
            }

            srv1.request.command = 2;
            srv1.request.pangle =
                ((atan2(pose[G_i + 1].y - pose[G_i + 2].y, pose[G_i + 1].x - pose[G_i + 2].x) * 180) / M_PI) -
                srv01.response.robots_theta;

            if (srv1.request.pangle > 180)
              srv1.request.pangle = srv1.request.pangle - 360;
            if (srv1.request.pangle < -180)
              srv1.request.pangle = srv1.request.pangle + 360;

            ROS_INFO("spin robot : %f", srv1.request.pangle);
            if (client1.call(srv1))
            {
            }
            else
            {
              ROS_ERROR("Failed to call service pioneer_pmove");
            }

            srv01.request.robots_id = 5;
            if (client01.call(srv01))
            {
              ROS_INFO("robotid=%u:(%f,%f,%f),state=%u,place=%u", srv01.request.robots_id, srv01.response.robots_x,
                       srv01.response.robots_y, srv01.response.robots_theta, srv01.response.robots_state,
                       srv01.response.robots_place);
            }
            else
            {
              ROS_ERROR("Failed to call service tmsdb_get_robots_info");
            }

            srv1.request.command = 1;
            srv1.request.pdist =
                sqrt(pow(pose[G_i + 1].x - pose[G_i + 2].x, 2) + pow(pose[G_i + 1].y - pose[G_i + 2].y, 2));
            ROS_INFO("go robot : %f", srv1.request.pdist);
            if (client1.call(srv1))
            {
            }
            else
            {
              ROS_ERROR("Failed to call service pioneer_pmove");
            }

            srv01.request.robots_id = 5;
            if (client01.call(srv01))
            {
              ROS_INFO("robotid=%u:(%f,%f,%f),state=%u,place=%u", srv01.request.robots_id, srv01.response.robots_x,
                       srv01.response.robots_y, srv01.response.robots_theta, srv01.response.robots_state,
                       srv01.response.robots_place);
            }
            else
            {
              ROS_ERROR("Failed to call service tmsdb_get_robots_info");
            }

            srv1.request.command = 2;
            srv1.request.pangle = pose[G_i + 1].th - srv01.response.robots_theta;

            if (srv1.request.pangle > 180)
              srv1.request.pangle = srv1.request.pangle - 360;
            if (srv1.request.pangle < -180)
              srv1.request.pangle = srv1.request.pangle + 360;

            ROS_INFO("spin robot : %f", srv1.request.pangle);
            if (client1.call(srv1))
            {
            }
            else
            {
              ROS_ERROR("Failed to call service pioneer_pmove");
            }
          }
          //					directmove = false;
          directmove_int = 0;
        }
        //顔検出LEFT側
        if (client6.call(srv6))
        {
        }
        else
        {
          ROS_ERROR("Failed to call service ods_face_detection");
        }

        if (srv6.response.result == 1)
        {  //顔検出：顔が見つかった場合
          ROS_INFO("face detected.(G1-G3-G2/G4-G6-G5);pose[%d]", G_i + 1);
          find_person = true;
          break;
        }
        else
        {  //顔検出：顔が見つからなかった場合
          ROS_INFO("face not detected.(G1-G3-G2-/G4-G6-G5-);pose[%d],pose[%d]", G_i + 2, G_i + 1);
        }
      }
    }  // <- 肌色検出による左右決定のif/else
  }    // <- G1,G4選択のfor

RESULT:

  //	if(find_person){
  ROS_INFO("face was found. Send it TMS.");
  if (srv6.response.image.data.size() != 0)
  {
    cv_ptr = cv_bridge::toCvCopy(srv6.response.image, srv6.response.image.encoding);
    cv::imwrite("rgb_image_face.png", cv_ptr->image);
  }
  //	}
  //	else{
  ROS_INFO("face was not found. Send person body?/big object? TMS");
  if (srv5.response.image.data.size() != 0)
  {
    cv_ptr = cv_bridge::toCvCopy(srv5.response.image, srv5.response.image.encoding);
    cv::imwrite("rgb_image_body.png", cv_ptr->image);
  }
  if (srv4.response.image.data.size() != 0)
  {
    cv_ptr = cv_bridge::toCvCopy(srv4.response.image, srv4.response.image.encoding);
    cv::imwrite("rgb_image_first.png", cv_ptr->image);
  }
//	}

BREAK:

  return true;
}

int main(int argc, char **argv)
{
  //	ROS_INFO("%f",180*atan2(2000,500)/M_PI);
  //	ROS_INFO("%f",180*atan2(-2000,500)/M_PI);
  //	ROS_INFO("%f",180*atan2(2000,-500)/M_PI);
  //	ROS_INFO("%f",180*atan2(-2000,-500)/M_PI);
  //	sleep(1000);

  // ROS init//
  ros::init(argc, argv, "tms_rs_fall_detection");
  ros::NodeHandle n;
  ROS_INFO("tms_rs_fall_detection : init");

  service1 = n.advertiseService("tms_rs_fall_detection", tms_rs_fall_detection_func);

  // 0X ... TMSDBと
  client01 = n.serviceClient< tms_msg_db::tmsdb_get_robots_info >("tmsdb_get_robots_info");
  //	client02 = n.serviceClient<tms_msg_db::tmsdb_get_furnitures_info>("tmsdb_get_furnitures_info");
  //	client03 = n.serviceClient<tms_msg_db::tmsdb_get_pcd_info>("tmsdb_get_pcd_info");
  //	client04 = n.serviceClient<tms_msg_db::tmsdb_file_conservation>("tmsdb_file_conservation");
  //	client05 = n.serviceClient<tms_msg_db::tmsdb_objects_data>("tmsdb_objects_data_1");

  //こちらはskill部と
  client1 = n.serviceClient< tms_msg_rc::tms_rc_pmove >("pmove");  //直進・回転命令を指定
  client2 = n.serviceClient< tms_msg_rc::tms_rc_pparam >("pparam");  // DirectMotionが完了しているかどうかを判定
  client3 = n.serviceClient< tms_msg_rp::rps_path_planning >("rps_path_planning");
  client4 = n.serviceClient< tms_msg_ss::ods_person_detection >("ods_person_detection");
  client5 = n.serviceClient< tms_msg_ss::ods_skincolor_extraction >("ods_skin_color");
  client6 = n.serviceClient< tms_msg_ss::ods_face_detection >("ods_face_detection");

  ros::spin();

  return 0;
}
