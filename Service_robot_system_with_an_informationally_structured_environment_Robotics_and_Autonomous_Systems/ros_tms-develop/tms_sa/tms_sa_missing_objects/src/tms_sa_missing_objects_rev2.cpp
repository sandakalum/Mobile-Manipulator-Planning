/*
 * tmsdb_analyze_missing_object.cpp
 *
 *  Created on: 2012/12/11
 *      Author: an
 */

// include for ROS
#include "ros/ros.h"
#include <unistd.h>
#include <boost/date_time/local_time/local_time.hpp>

#include "tms_msg_db/tmsdb_get_objects_info.h"
#include "tms_msg_db/tmsdb_get_person_info.h"
#include "tms_msg_db/tmsdb_get_person_behavior_info.h"
#include "tms_msg_sa/tms_to_activate_service.h"
#include "tms_msg_ts/tms_sa_find_objects.h"
//#include "tms_utility/ods_change_dt.h"

#include "tms_msg_sa/tms_to_modify_missing_objects.h"

#define table14_x 1400
#define table14_y 1900
#define desk13_x 4000
#define desk13_y 2000
#define bed12_x 0
#define bed12_y 0
#define threshold_furnitures 1000  //家具中心からの閾値設定
#define furnitures_start_number 11
#define objects_start_number 51

#define bed12_xld 2330
#define bed12_yld 3020
#define bed12_xlu 2330
#define bed12_ylu 3990
#define bed12_xrd 4490
#define bed12_yrd 3020
#define bed12_xru 4490
#define bed12_yru 3990

#define desk13_xld 3950
#define desk13_yld 1550
#define desk13_xlu 3950
#define desk13_ylu 2950
#define desk13_xrd 4490
#define desk13_yrd 1550
#define desk13_xru 4490
#define desk13_yru 2950

#define table14_xld 1000
#define table14_yld 1500
#define table14_xlu 1000
#define table14_ylu 2300
#define table14_xrd 1800
#define table14_yrd 1500
#define table14_xru 1800
#define table14_yru 2300

#define block_size 10       // mm
#define environment_x 4500  // mm
#define environment_y 4000  // mm
#define near_aria 500       // mm

// mapに関するFILE POINTER
FILE *fp;
// MISSING TABLE に関するFILE POINTER
FILE *fp_missing_table;

struct PERSON_HISTORY
{
  long unsigned int rostime;
  //	int id;
  float x;
  float y;
};

struct FURNITURES
{
  // std::vector<long unsigned int> near_time;	//数回近づいた場合を考慮，基本は[0]のみ利用する
  //.empty=0の場合は接近した家具がないため例外処理へ
  long unsigned int near_time;
  // std::vector<long unsigned int> left_time;	//数回近づいた場合を考慮
  long unsigned int left_time;
  // std::vector<int> id;	//furnitures_id
  int id;
};

struct FURNITURES_TOTAL
{
  int id;
  long unsigned int total_time;
};

//これで探し回る家具等を管理する
//基本的にDB_readerより持ってきた情報を保持する
struct MISSING_OBJECTS_P
{
  int missing_object_id;                         //消失物品のid
  bool missing_object_id_valid;                  //既に見つかっている場合はfalseになる
  long unsigned int missing_object_time;         //物品消失時間の記録
  std::vector< PERSON_HISTORY > person_history;  //人の歩行履歴(物品がなくなった後より)
  std::vector< FURNITURES > near_furnitures;  //近づいた家具のidと接近開始時間と離脱時間の保持(分離型)
  std::vector< FURNITURES_TOTAL > near_furnitures_total;
  int find_furnitures_id;  //物品を発見した家具のid
};

std::vector< MISSING_OBJECTS_P > missing_objects_list;

ros::ServiceServer service1;
ros::ServiceServer service2;

// callback1内 -> tmsdb
ros::ServiceClient client01;
ros::ServiceClient client02;
ros::ServiceClient client03;
ros::ServiceClient client04;
ros::ServiceClient client05;

// callback2内 -> tmsdb
ros::ServiceClient client001;

// callback1 -> tmsdb以外
ros::ServiceClient client1;
ros::ServiceClient client2;
ros::ServiceClient client3;
ros::ServiceClient client4;
ros::ServiceClient client5;

std::vector< std::vector< long unsigned int > > missing_table;
bool first_flag = true;

std::vector< unsigned long long int > ana_time;

//ここまで分析したよ、という時刻(分析時点での人の最後の歩行軌跡の時刻に一致する)
unsigned long int ana_time_scr;

//接近家具を分析するためのmap要素
struct FURNITURES_MAP_ELEMENT
{
  //マップの場所がその家具自体であるかどうかのフラグ
  bool bed;
  bool desk;
  bool table;

  //マップの場所と家具との距離計算
  //ここで言う距離とは、ブロック化後の距離、つまりブロック化10cmなら11cm離れている場所は距離2
  int bed_distance;
  int desk_distance;
  int table_distance;

  // sqrt(2)として斜め方向を計算するための要素
  double bed_distance_d;
  double desk_distance_d;
  double table_distance_d;

  //マップの場所がその家具に近いかどうかのフラグ
  //これを参照することで足の座標が家具に近いかどうかを判定する
  bool near_bed_flag;
  bool near_desk_flag;
  bool near_table_flag;
};

//接近家具を分析するためのmap
std::vector< std::vector< FURNITURES_MAP_ELEMENT > > map;

// sort降順比較用関数
static bool compare(const FURNITURES_TOTAL &c1, const FURNITURES_TOTAL &c2)
{
  return c1.total_time > c2.total_time;
}

bool tms_utility_analyze_missing_objects_func(tms_msg_sa::tms_to_activate_service::Request &req,
                                              tms_msg_sa::tms_to_activate_service::Response &res)
{
  printf("activate = %d\n", req.activation);

  tms_msg_db::tmsdb_get_objects_info srv1;
  tms_msg_db::tmsdb_get_objects_info srv2;
  tms_msg_db::tmsdb_get_person_info srv3;
  tms_msg_ts::tms_sa_find_objects srv4;
  tms_msg_sa::tms_to_modify_missing_objects srv5;

  MISSING_OBJECTS_P temp_missing_objects_list;
  PERSON_HISTORY temp_person_history;
  FURNITURES temp_furniture;

  FURNITURES_TOTAL temp_furnitures_total;

  std::vector< FURNITURES_TOTAL > furnitures_order(32);
  for (unsigned int i = 0; i < 32; i++)
  {
    furnitures_order[i].id = furnitures_start_number + i;  // idをfurnitures_numberから32個割り振っていく
    furnitures_order[i].total_time = 0;
  }

  //なくなった物品リスト(missing_objects_list)の管理用カウンタ
  unsigned int while_i = 0;
  //人の歩行軌跡用カウンタ
  unsigned int while_j = 0;
  // desk_total_time
  long unsigned int desk13_sum = 0;
  // table_total_time
  long unsigned int table14_sum = 0;

  // presentよりstate=0(ICSより消失した物品)のrostime~idを全て取得
  if (client1.call(srv1))
  {
    for (unsigned int i = 0; i < srv1.response.id.size(); i++)
    {
      ROS_INFO("Success! [%d] rostime=%lu,id=%d, x=%f, y=%f, z=%f, weight=%f, state=%d, place=%d", i,
               srv1.response.rostime[i], srv1.response.id[i], srv1.response.x[i], srv1.response.y[i],
               srv1.response.z[i], srv1.response.weight[i], srv1.response.state[i], srv1.response.place[i]);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service tmsdb_get_objects_info_present");
  }

  // objects_historyより，上で取得したidの最新状態を取得する
  // srv2.response.id[]が,所在不明の物品idリスト
  for (unsigned int i = 0; i < srv1.response.id.size(); i++)
  {
    srv2.request.id.push_back(srv1.response.id[i]);  // presentのstate=0のidのobjects_idを挿入
  }
  if (client2.call(srv2))
  {
    for (unsigned int i = 0; i < srv2.response.id.size(); i++)
    {
      ROS_INFO("Success! [%d] rostime=%lu,id=%d,weight=%f,state=%d,place=%d", i, srv2.response.rostime[i],
               srv2.response.id[i], srv2.response.weight[i], srv2.response.state[i], srv2.response.place[i]);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service tmsdb_get_objects_info_history");
  }

  // missing_objects_listのid欄を埋める
  /////////ここでplace==0の要素がない場合は後にmissing_objects_listの要素にアクセスした場合にSegmentationFaultが起こるので注意！！！！！////////////
  for (unsigned int i = 0; i < srv2.response.id.size(); i++)
  {
    printf("%d\n", i);
    if (srv2.response.place[i] == 0)
    {  //物品の最新状態が0(=紛失)の場合、missing_listへ挿入
      temp_missing_objects_list.missing_object_id = srv2.response.id[i];
      temp_missing_objects_list.missing_object_id_valid = true;
      temp_missing_objects_list.missing_object_time = srv2.response.rostime[i];  //物品の紛失時刻を記録
      missing_objects_list.push_back(temp_missing_objects_list);
    }
  }

  for (unsigned int i = 0; i < missing_objects_list.size(); i++)
  {
    printf("missing_objects_id=%lu, missing_objects_id=%d\n", missing_objects_list[i].missing_object_time,
           missing_objects_list[i].missing_object_id);
  }

  //ここからは巨大なループ構造を作成
  // while(missing_objects_list == null)でループを抜ける
  //内部での処理は
  // missing_objects_listにidが入っているとき、それが知的収納庫から紛失した後の人の動きを抽出する
  //なお、ロボットが動かしたことが分かっている場合、その情報はobjects_historyに入っているのでmissing_objects_listに入らない！

  for (while_i = 0; while_i < missing_objects_list.size(); while_i++)
  {
    if (missing_objects_list[while_i].missing_object_id_valid == true)
    {  //他の物品探索の結果、ついでに見つかった物品はfalseになる
      ROS_INFO("34");
      //物品紛失直後の人の移動履歴をreaderから取得
      // get_person_info(std::vector)を用いる。WHERE条件はmissing_objects_list.missing_time以降
      srv3.request.rostime.push_back(missing_objects_list[while_i].missing_object_time);

      // srv3.request.rostime[0] = missing_objects_list[while_i].missing_object_time;
      // missing_objects_listへ挿入
      if (client3.call(srv3))
      {
        for (unsigned int i = 0; i < srv3.response.id.size(); i++)
        {
          ROS_INFO("Success! [%d] rostime=%lu,id=%d,x=%f,y=%f,z=%f,theta=%f,state=%d,place=%d", i,
                   srv3.response.rostime[i], srv3.response.id[i], srv3.response.x[i], srv3.response.y[i],
                   srv3.response.z[i], srv3.response.theta[i], srv3.response.state[i], srv3.response.place[i]);
          temp_person_history.rostime = srv3.response.rostime[i];
          temp_person_history.x = srv3.response.x[i];
          temp_person_history.y = srv3.response.y[i];
          missing_objects_list[while_i].person_history.push_back(temp_person_history);
          //					sleep(3);
          //					printf("get : end\n");

          //					for(unsigned int j=0;j<missing_objects_list[i].person_history.size();j++){
          //						ROS_INFO("rostime=%lu, x=%f,y=%f\n",
          //								missing_objects_list[i].person_history[j].rostime,
          //								missing_objects_list[i].person_history[j].x,
          //								missing_objects_list[i].person_history[j].y);
          //					}
        }
      }
      else
      {
        ROS_ERROR("Failed to call service tmsdb_get_person_info");
      }

      // goto LEFT;

      //近づいた家具(足の位置+-手の長さ+ α)のidを抽出、接近時間を保持
      //家具の現在位置を取得する get_furnitures_infoが必要
      //ここで取得した数の配列(vectorではなく)を生成した方が楽かもしれない

      // table14_x = 1500, table14_y = 2000
      // desk13_x = 4000, desk13_y = 2000
      // bed12_x = 0, bed12_y = 0

      //物品紛失後に人の足が観測されていれば
      for (while_j = 0; while_j < missing_objects_list[while_i].person_history.size(); while_j++)
      {
        ROS_INFO("1");
        //              //bed12に近づいていればセット
        //				ROS_INFO("desk12?:%lf",sqrt(pow((missing_objects_list[while_i].person_history[while_j].x - bed12_x),2)
        //+ pow((missing_objects_list[while_i].person_history[while_j].y - bed12_y),2)));
        //				if(sqrt(pow((missing_objects_list[while_i].person_history[while_j].x - bed12_x),2) +
        // pow((missing_objects_list[while_i].person_history[while_j].y - bed12_y),2))
        //						< threshold_furnitures){
        //					ROS_INFO("2:while_j = %d",while_j);
        //					temp_furniture.id = 12;
        //					temp_furniture.near_time = missing_objects_list[while_i].person_history[while_j].rostime;
        //					if(while_j < missing_objects_list[while_i].person_history.size()-1){
        //						ROS_INFO("1-1");
        //						temp_furniture.left_time = missing_objects_list[while_i].person_history[while_j+1].rostime;
        //					}
        //					//歩行履歴の端点の場合は滞在時間が不明になるので到達時間+1秒(=1*1000(usec)*1000(msec)*1000(sec))を加える
        //					else{
        //						ROS_INFO("1-2");
        //						temp_furniture.left_time =
        // missing_objects_list[while_i].person_history[while_j].rostime+(1*1000*1000*1000);
        //					}
        //					missing_objects_list[while_i].near_furnitures.push_back(temp_furniture);
        //				}

        // desk13に近づいていればセット
        ROS_INFO("desk13?:%lf", sqrt(pow((missing_objects_list[while_i].person_history[while_j].x - desk13_x), 2) +
                                     pow((missing_objects_list[while_i].person_history[while_j].y - desk13_y), 2)));
        if (sqrt(pow((missing_objects_list[while_i].person_history[while_j].x - desk13_x), 2) +
                 pow((missing_objects_list[while_i].person_history[while_j].y - desk13_y), 2)) < threshold_furnitures)
        {
          ROS_INFO("2:while_j = %d", while_j);
          temp_furniture.id = 13;
          temp_furniture.near_time = missing_objects_list[while_i].person_history[while_j].rostime;
          if (while_j < missing_objects_list[while_i].person_history.size() - 1)
          {
            ROS_INFO("2-1");
            temp_furniture.left_time = missing_objects_list[while_i].person_history[while_j + 1].rostime;
          }
          //歩行履歴の端点の場合は滞在時間が不明になるので到達時間+1秒(=1*1000(usec)*1000(msec)*1000(sec))を加える
          else
          {
            ROS_INFO("2-2");
            temp_furniture.left_time =
                missing_objects_list[while_i].person_history[while_j].rostime + (1 * 1000 * 1000 * 1000);
          }
          missing_objects_list[while_i].near_furnitures.push_back(temp_furniture);
        }
        // table14に近づいていればセット
        ROS_INFO("table14?:%lf", sqrt(pow((missing_objects_list[while_i].person_history[while_j].x - table14_x), 2) +
                                      pow((missing_objects_list[while_i].person_history[while_j].y - table14_y), 2)));
        if (sqrt(pow((missing_objects_list[while_i].person_history[while_j].x - table14_x), 2) +
                 pow((missing_objects_list[while_i].person_history[while_j].y - table14_y), 2)) < threshold_furnitures)
        {
          ROS_INFO("3:while_j = %d", while_j);
          temp_furniture.id = 14;
          temp_furniture.near_time = missing_objects_list[while_i].person_history[while_j].rostime;
          if (while_j < missing_objects_list[while_i].person_history.size() - 1)
          {
            ROS_INFO("3-1");
            temp_furniture.left_time = missing_objects_list[while_i].person_history[while_j + 1].rostime;
          }
          //歩行履歴の端点の場合は滞在時間が不明になるので到達時間+1秒(=1*1000(usec)*1000(msec)*1000(sec))を加える
          else
          {
            ROS_INFO("3-2");
            temp_furniture.left_time =
                missing_objects_list[while_i].person_history[while_j].rostime + (1 * 1000 * 1000 * 1000);
          }
          missing_objects_list[while_i].near_furnitures.push_back(temp_furniture);
        }
      }
      ROS_INFO("4");
      // FURNITURES_TOTALの計算
      for (unsigned int i = 0; i < missing_objects_list[while_i].near_furnitures.size(); i++)
      {
        if (missing_objects_list[while_i].near_furnitures[i].id == 13)
        {
          desk13_sum += missing_objects_list[while_i].near_furnitures[i].left_time -
                        missing_objects_list[while_i].near_furnitures[i].near_time;
        }
        if (missing_objects_list[while_i].near_furnitures[i].id == 14)
        {
          table14_sum += missing_objects_list[while_i].near_furnitures[i].left_time -
                         missing_objects_list[while_i].near_furnitures[i].near_time;
        }
      }
      printf("desk13_sum = %lu\n", desk13_sum);
      printf("table14_sum = %lu\n", table14_sum);

      // missing_objects_listへ挿入

      for (unsigned int i = 0; i < 2; i++)
      {
        temp_furnitures_total.id = 13 + i;

        if (i == 0)
          temp_furnitures_total.total_time = desk13_sum;
        else
          temp_furnitures_total.total_time = table14_sum;

        missing_objects_list[while_i].near_furnitures_total.push_back(temp_furnitures_total);
      }
    }

    srv3.request.rostime.clear();
    desk13_sum = 0;
    table14_sum = 0;
  }

  //全ての物品に対する家具周辺の滞在時間の総時間を集計
  //すなわち、紛失物品基準の歩行履歴の時間から家具主体への歩行履歴の時間へ変換する
  for (unsigned int i = 0; i < missing_objects_list.size(); i++)
  {
    for (unsigned int j = 0; j < missing_objects_list[i].near_furnitures_total.size(); j++)
    {
      for (unsigned int k = 0; k < furnitures_order.size(); k++)
      {
        //ここではmissing_objects_listの各紛失物体に対して、歩行履歴中の接近"時間"(時刻ではない)を引っ張ってくる(i,j)
        //それがfurnitures_order[k].total_timeへ加算される
        if (furnitures_order[k].id == missing_objects_list[i].near_furnitures[j].id)
        {
          furnitures_order[k].total_time += missing_objects_list[i].near_furnitures[j].left_time -
                                            missing_objects_list[i].near_furnitures[j].near_time;
        }
      }
    }
  }

  //見に行く場所(家具)の順序を確定
  //降順ソート(移動順序(家具IDの配列)決定)
  std::sort(furnitures_order.begin(), furnitures_order.end(), compare);
  for (unsigned int i = 0; i < furnitures_order.size(); i++)
  {
    printf("id = %d, total_sum = %lu\n", furnitures_order[i].id, furnitures_order[i].total_time);
  }
  ROS_INFO("furnitures_order.size: %ld", furnitures_order.size());

  // srvへ挿入(furnitures_orderのうちtotal_time == 0のものは除く) <-
  // これは近づいていない、すなわち探す対象から省かれるため
  for (unsigned int i = 0; i < furnitures_order.size(); i++)
  {
    if (furnitures_order[i].total_time != 0)
    {
      srv4.request.search_furnitures_id.push_back(furnitures_order[i].id);
    }
  }

  // taskをTaskSchedulerに投げる[-> tms_utility_task_find_objects_T]
  if (client4.call(srv4))
  {
    ROS_INFO("task_result_id : %d", srv4.response.success);
    if (srv4.response.message.empty() != true)
      ROS_INFO("task_message : %s", srv4.response.message.c_str());

    //タスクの結果、object_idとfurnitures_idの組が返ってきたのであれば
    //情報をmissing_objects_listから引き出してきて、writerへ投げて書き込む
    //※物品を発見した時刻を書き込むか、人が家具に接近した時刻を書き込むかは議論の余地あり
    //とりあえず現在は物品を発見した時刻を書き込むことにする
    if (srv4.response.object_id.empty() != true)
    {
      for (unsigned int j = 0; j < srv4.response.object_id.size(); j++)
      {
        printf("object_id = %d, place = %d\n", srv4.response.object_id[j], srv4.response.furnitures_id[j]);
      }
    }
    else
    {
      ROS_INFO("NOT FOUND");
    }
  }
  else
  {
    ROS_ERROR("Failed to call service robot_task_find_objects");
  }

  res.success = 1;

  // missing_objects_list.clear();

  return true;
}

void callback1(const ros::TimerEvent &)
{
  ROS_INFO("Callback 1 triggered");

  //分析パートのための変数
  tms_msg_db::tmsdb_get_objects_info srv01;
  tms_msg_db::tmsdb_get_objects_info srv02;
  tms_msg_db::tmsdb_get_person_info srv03;

  bool temp_object_flag;  //知的収納庫にある物品IDに関するフラグ
                          //このフラグがtrueの場合はmissing_listの該当行を消去しない．

  //タスク送信パートのための変数
  std::vector< FURNITURES_TOTAL > furnitures_order;  // TSに渡すID列
  FURNITURES_TOTAL temp_furnitures;
  temp_furnitures.total_time = 0;
  tms_msg_ts::tms_sa_find_objects srv1;

  /////////////////// 分析パート ///////////////////
  temp_object_flag = false;  //基本的に消しにかかる

  // presentよりstate=0(ICSより消失した物品)のrostime~idを全て取得する
  // presentからはあくまでも"知的収納庫に"あるか、ないかしか分からない。
  if (client01.call(srv01))
  {
    for (unsigned int i = 0; i < srv01.response.id.size(); i++)
    {
      ROS_INFO("Success! [%d] rostime=%lu,id=%d, x=%f, y=%f, z=%f, weight=%f, state=%d, place=%d", i,
               srv01.response.rostime[i], srv01.response.id[i], srv01.response.x[i], srv01.response.y[i],
               srv01.response.z[i], srv01.response.weight[i], srv01.response.state[i], srv01.response.place[i]);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service tmsdb_get_objects_info_present");
  }

  //収納庫内に物品が戻された場合にmissing_tableの該当物品ID列の総TIMEを0に戻す
  // vector.clearは0すら残さないので0を代入することに気をつける
  for (unsigned int i = 0; i < missing_table.size(); i++)
  {
    for (unsigned int j = 0; j < srv01.response.id.size(); j++)
    {
      if (srv01.response.id[j] - objects_start_number == i)
      {
        temp_object_flag = true;
      }
    }
    if (!temp_object_flag)
    {
      for (unsigned int k = 0; k < missing_table[i].size(); k++)
      {
        missing_table[i][k] = 0;
      }
      ROS_INFO("missing_table[%d] : CLEAR", i);
    }
    else
    {
      ROS_INFO("missing_table[%d] : NOT CLEAR", i);
    }
    temp_object_flag = false;
  }

  // objects_historyより，上で取得したidの最新状態を取得する
  //ここで得られるのはシステム全体として見失った物品のIDとその時刻
  // srv2.response.id[]が,所在不明の物品idリスト
  for (unsigned int i = 0; i < srv01.response.id.size(); i++)
  {
    srv02.request.id.push_back(srv01.response.id[i]);  // presentのstate=0のidのobjects_idを挿入
  }
  if (client02.call(srv02))
  {
    for (unsigned int i = 0; i < srv02.response.id.size(); i++)
    {
      ROS_INFO("Success! [%d] rostime=%lu,id=%d,weight=%f,state=%d,place=%d", i, srv02.response.rostime[i],
               srv02.response.id[i], srv02.response.weight[i], srv02.response.state[i], srv02.response.place[i]);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service tmsdb_get_objects_info_history");
  }

  //ここで、「いつまで」分析したかの配列ana_time[]を参照して、人の歩行軌跡を引っ張り出す
  //ただし、02responce次第で抜ける
  if (srv02.response.id.empty() == false)
  {
    if (first_flag)
    {
      ROS_INFO("FIRST Analysis");

      //			ana_time[0] = 物品ID[51]の紛失時刻が初期値として入る
      //			以降、ana_time[1] = 物品ID[52]の紛失時刻...
      //			for(unsigned int i=0;i<srv02.response.id.size();i++){
      //				for(unsigned int j=0;j<10;j++){
      //					if((srv02.response.id[i] - objects_start_number) == j){
      //						ana_time[j] = srv02.response.rostime[i];
      //					}
      //				}
      //			}
      ana_time_scr = 0;  //

      // missing_tableの更新(初回)
      //『iに対応するのはobjects_presentで紛失している物品のid』
      //すなわち物品IDごとに以下の処理を行う
      for (unsigned int i = 0; i < srv02.response.id.size(); i++)
      {
        //もし，objects_historyの最新状態も紛失であれば以下の処理を行う
        if (srv02.response.state[i] == 0)
        {
          //物品紛失時刻以降の人の歩行軌跡情報列(time,id,x,y)[]を引っ張り出す
          srv03.request.rostime.push_back(srv02.response.rostime[i]);
          if (client03.call(srv03))
          {
            // missing_table更新
            //ここは本来「ある座標もしくはある家具」と「それをセットするmissing_table[物品ID][！ここ！]の対応関係を書いた配列」を定義しておくべき？
            //「ある座標もしくはある家具」の数が増えれば、それに対応させてmissing_tableの！ここ！を増やす
            //『jに対応するのは取得した歩行軌跡のどこを指すか』
            for (unsigned int j = 0; j < srv03.response.x.size(); j++)
            {
              // bed12に近づいていればセット
              if (map[srv03.response.x[j] / block_size][srv03.response.y[j] / block_size].near_bed_flag)
              {
                //							ROS_INFO("1");
                if ((srv03.response.x.size() - j) != 1)
                {
                  missing_table[srv02.response.id[i] - objects_start_number][0] +=
                      (srv03.response.rostime[j + 1] - srv03.response.rostime[j]);
                  //								ROS_INFO("1-1");
                }
                //最後の歩行軌跡の場合は滞在時間が分からない
                //これは次の歩行軌跡が得られた場合(すなわち次の分析タイミングで歩行軌跡が新たに取得された場合)滞在時間として取得される
                //ただし、退出してしまった場合は滞在時間でなくなるので注意！
                //(「部屋の端で足が取得された後」に退出し、「部屋の端で足が取得されてから部屋内を動く」のであれば、家具接近条件に当たらず、分析対象にならないので現状問題なし)
                //もし部屋の端にある場合は、その時刻を保持する必要がある。次回入室時にrostimeをキーとして省く作業をしないと、退出時間がすべて接近に該当してしまう
                else
                {
                  //								missing_table[srv02.response.id[i]-objects_start_number][0] += 1*1000*1000*1000;
                  //								ROS_INFO("1-2");
                }
              }

              // desk13に近づいていればセット
              if (map[srv03.response.x[j] / block_size][srv03.response.y[j] / block_size].near_desk_flag)
              {
                //							ROS_INFO("2");
                if ((srv03.response.x.size() - j) != 1)
                {
                  missing_table[srv02.response.id[i] - objects_start_number][1] +=
                      (srv03.response.rostime[j + 1] - srv03.response.rostime[j]);
                  //								ROS_INFO("2-1");
                }
                //最後の歩行軌跡の場合は滞在時間が分からないので1秒を加える
                //ただし、現在の家具の位置では、家具に近づいている条件に多分ヒットしないため、ここに入ることはまずない
                else
                {
                  //								missing_table[srv02.response.id[i]-objects_start_number][1] += 1*1000*1000*1000;
                  //								ROS_INFO("2-2");
                }
              }

              // table14に近づいていればセット
              if (map[srv03.response.x[j] / block_size][srv03.response.y[j] / block_size].near_table_flag)
              {
                //							ROS_INFO("3");
                if ((srv03.response.x.size() - j) != 1)
                {
                  missing_table[srv02.response.id[i] - objects_start_number][2] +=
                      (srv03.response.rostime[j + 1] - srv03.response.rostime[j]);
                  //								ROS_INFO("3-1");
                }
                else
                {
                  //								missing_table[srv02.response.id[i]-objects_start_number][2] += 1*1000*1000*1000;
                  //								ROS_INFO("3-2");
                }
              }

              //....に近づいていればセット
            }
          }
          else
          {
            ROS_ERROR("Failed to call service tmsdb_get_person_info_2");
          }

          //最後に取得した人の歩行軌跡のrostimeを利用して分析済時間の更新
          if (srv03.response.rostime.empty() == false)
          {
            first_flag = false;
            ROS_INFO("first_flag = false, response.rostime_push back");
            ana_time_scr = srv03.response.rostime.back();
          }

          srv03.request.rostime.clear();
          srv03.response.rostime.clear();
          srv03.response.id.clear();
          srv03.response.x.clear();
          srv03.response.y.clear();
        }
        else
        {
          // state != 0 すなわち知的収納庫にはないが，部屋内では見つかっている物品に対して処理
          // missing_tableのIDに対応する列をclearする
          for (unsigned int j = 0; j < missing_table[srv02.response.id[i] - objects_start_number].size(); j++)
          {
            missing_table[srv02.response.id[i] - objects_start_number][j] = 0;
          }
          ROS_INFO("CLEAR TABLE : missing_table[%d]", srv02.response.id[i] - objects_start_number);
        }
      }
    }

    else
    {
      //人の歩行軌跡を引っ張り出す
      ROS_INFO("SECOND Analysis...");

      // iに対応するのは紛失物品のID列
      for (unsigned int i = 0; i < srv02.response.id.size(); i++)
      {
        if (srv02.response.state[i] == 0)
        {
          //「前回分析にかけた最後の歩行軌跡を含め、それ以降の」人の歩行軌跡情報列(time,id,x,y)[]を引っ張り出す
          //前回の最後の歩行軌跡は滞在時間としては分析されていないため、重複はしない。
          srv03.request.rostime.push_back(ana_time_scr);
          if (client03.call(srv03))
          {
            //					for(unsigned int j=0;j<srv03.response.x.size();j++){
            //						ROS_INFO("(rostime,id,x,y)=(%lu, %d, %f, %f)",
            //								srv03.response.rostime[j], srv03.response.id[j], srv03.response.x[j],
            // srv03.response.y[j]);
            //					}

            // missing_table更新
            //ここは本来「ある座標もしくはある家具」と「それをセットするmissing_table[物品ID][！ここ！]の対応関係を書いた配列」を定義しておくべき？
            //「ある座標もしくはある家具」の数が増えれば、それに対応させてmissing_tableの！ここ！を増やす
            //『jに対応するのは取得した歩行軌跡のどこを指すか』

            // srv03.responce == 0 の場合は即座に抜ける
            for (unsigned int j = 0; j < srv03.response.x.size(); j++)
            {
              // bed12に近づいていればセット
              if (map[srv03.response.x[j] / block_size][srv03.response.y[j] / block_size].near_bed_flag)
              {
                //							ROS_INFO("1");
                //歩行軌跡の最後の1つは次回分析に回すため滞在時間を計算しない(というか最新なのでできない)
                if ((srv03.response.x.size() - j) != 1)
                {
                  missing_table[srv02.response.id[i] - objects_start_number][0] +=
                      (srv03.response.rostime[j + 1] - srv03.response.rostime[j]);
                  //								ROS_INFO("1-1");
                }
                //最後の歩行軌跡の場合は滞在時間が分からない
                //これは次の歩行軌跡が得られた場合(すなわち次の分析タイミングで歩行軌跡が新たに取得された場合)滞在時間として取得される
                //ただし、退出してしまった場合は滞在時間でなくなるので注意！
                //(「部屋の端で足が取得された後」に退出し、「部屋の端で足が取得されてから部屋内を動く」のであれば、家具接近条件に当たらず、分析対象にならないので現状問題なし)
                //もし部屋の端にある場合は、その時刻を保持する必要がある。次回入室時にrostimeをキーとして省く作業をしないと、退出時間がすべて接近に該当してしまう
                else
                {
                  //								missing_table[srv02.response.id[i]-objects_start_number][0] += 1*1000*1000*1000;
                  //								ROS_INFO("1-2");
                }
              }

              // desk13に近づいていればセット
              if (map[srv03.response.x[j] / block_size][srv03.response.y[j] / block_size].near_desk_flag)
              {
                //							ROS_INFO("2");
                if ((srv03.response.x.size() - j) != 1)
                {
                  missing_table[srv02.response.id[i] - objects_start_number][1] +=
                      (srv03.response.rostime[j + 1] - srv03.response.rostime[j]);
                  //								ROS_INFO("2-1");
                }
                //最後の歩行軌跡の場合は滞在時間が分からないので1秒を加える
                //ただし、現在の家具の位置では、家具に近づいている条件に多分ヒットしないため、ここに入ることはまずない
                else
                {
                  //								missing_table[srv02.response.id[i]-objects_start_number][1] += 1*1000*1000*1000;
                  //								ROS_INFO("2-2");
                }
              }
              // table14に近づいていればセット
              if (map[srv03.response.x[j] / block_size][srv03.response.y[j] / block_size].near_table_flag)
              {
                //							ROS_INFO("3");
                if ((srv03.response.x.size() - j) != 1)
                {
                  missing_table[srv02.response.id[i] - objects_start_number][2] +=
                      (srv03.response.rostime[j + 1] - srv03.response.rostime[j]);
                  //								ROS_INFO("3-1");
                }
                else
                {
                  //								missing_table[srv02.response.id[i]-objects_start_number][2] += 1*1000*1000*1000;
                  //								ROS_INFO("3-2");
                }
              }
              //....に近づいていればセット
            }
          }
          else
          {
            ROS_ERROR("Failed to call service tmsdb_get_person_info_2");
          }

          //最後に取得した人の歩行軌跡のrostimeを利用して分析済時間の更新
          if (srv03.response.rostime.empty() == false)
          {
            ana_time_scr = srv03.response.rostime.back();
          }

          srv03.request.rostime.clear();
          srv03.response.rostime.clear();
          srv03.response.id.clear();
          srv03.response.x.clear();
          srv03.response.y.clear();
        }

        else
        {
          for (unsigned int j = 0; j < missing_table[srv02.response.id[i] - objects_start_number].size(); j++)
          {
            missing_table[srv02.response.id[i] - objects_start_number][j] = 0;
          }
          ROS_INFO("CLEAR TABLE : missing_table[%d]", srv02.response.id[i] - objects_start_number);
        }
      }
    }
  }
  else
  {
    ROS_INFO("NON missing object");
  }

  // missing_table書き出し
  for (unsigned int i = 0; i < missing_table.size(); i++)
  {
    for (unsigned int j = 0; j < missing_table[i].size(); j++)
    {
      fprintf(fp_missing_table, "%lu, ", missing_table[i][j]);
    }
    fprintf(fp_missing_table, "\n");
  }
  fprintf(fp_missing_table, "\n");

  for (unsigned int i = 0; i < missing_table.size(); i++)
  {
    for (unsigned int j = 0; j < missing_table[i].size(); j++)
    {
      printf("%lu, ", missing_table[i][j]);
    }
    printf("\n");
  }

  ROS_INFO("ana_time_scr = %lu", ana_time_scr);

  /////////////////// タスク送信パート ///////////////////
  printf("\n");

  // iが物品IDに対応，jが家具IDに対応
  for (unsigned int j = 0; j < missing_table.front().size(); j++)
  {
    for (unsigned int i = 0; i < missing_table.size(); i++)
    {
      temp_furnitures.total_time += missing_table[i][j];
    }
    temp_furnitures.id = j + 12;
    furnitures_order.push_back(temp_furnitures);
    temp_furnitures.total_time = 0;
  }

  //総滞在時間の長い順にソートする(家具idと総滞在時間の組をクイックソート)
  std::sort(furnitures_order.begin(), furnitures_order.end(), compare);
  for (unsigned int i = 0; i < furnitures_order.size(); i++)
  {
    printf("(id, rostime) = (%d, %lu)\n", furnitures_order[i].id, furnitures_order[i].total_time);
  }
  for (unsigned int i = 0; i < furnitures_order.size(); i++)
  {
    //今のところ放置場所は最大10としているので0の部分も出てくる
    if (furnitures_order[i].total_time != 0)
    {
      srv1.request.search_furnitures_id.push_back(furnitures_order[i].id);
    }
  }
  if (client4.call(srv1))
  {
    ROS_INFO("task_result_id : %d", srv1.response.success);
    if (srv1.response.message.empty() != true)
      ROS_INFO("task_message : %s", srv1.response.message.c_str());
    else
      ROS_INFO("error");
  }
  else
  {
    ROS_ERROR("Failed to call service robot_task_find_objects");
  }

  printf("\n");
}

void callback2(const ros::TimerEvent &)
{
  //ここではmap[i][j]の集計及び見に行く固定家具の決定を行い、それをTSに投げる
  // MultiThreadSpinnerを利用しているため，ロボットサービスを行っている最中であってもmissing_tableは更新される(はず…)

  std::vector< FURNITURES_TOTAL > furnitures_order;  // TSに渡すID列
  FURNITURES_TOTAL temp_furnitures;

  temp_furnitures.total_time = 0;

  tms_msg_db::tmsdb_get_person_behavior_info srv01;
  tms_msg_ts::tms_sa_find_objects srv1;

  //まずは動作条件に当たるかどうか(人の現在の状態=退室？)を分析する
  //本来はTSに担わせるべき条件かな？
  if (client001.call(srv01))
  {
    ROS_INFO("%u, %u", srv01.response.id, srv01.response.behavior);
  }
  else
  {
    ROS_ERROR("Failed to call service tmsdb_get_person_behavior");
  }

  if (srv01.response.behavior == 0)
  {
    ROS_INFO("Trigger");

    // iが物品IDに対応，jが家具IDに対応
    for (unsigned int j = 0; j < missing_table.front().size(); j++)
    {
      for (unsigned int i = 0; i < missing_table.size(); i++)
      {
        temp_furnitures.total_time += missing_table[i][j];
      }
      temp_furnitures.id = j + 12;
      furnitures_order.push_back(temp_furnitures);
      temp_furnitures.total_time = 0;
    }

    //総滞在時間の長い順にソートする(家具idと総滞在時間の組をクイックソート)
    std::sort(furnitures_order.begin(), furnitures_order.end(), compare);
    for (unsigned int i = 0; i < furnitures_order.size(); i++)
    {
      printf("(id, rostime) = (%d, %lu)\n", furnitures_order[i].id, furnitures_order[i].total_time);
    }
    for (unsigned int i = 0; i < furnitures_order.size(); i++)
    {
      //今のところ放置場所は最大10としているので0の部分も出てくる
      if (furnitures_order[i].total_time != 0)
      {
        srv1.request.search_furnitures_id.push_back(furnitures_order[i].id);
      }
    }
    if (client4.call(srv1))
    {
      ROS_INFO("task_result_id : %d", srv1.response.success);
      if (srv1.response.message.empty() != true)
        ROS_INFO("task_message : %s", srv1.response.message.c_str());

      //タスクの結果、object_idとfurnitures_idの組が返ってきたのであれば
      //情報をmissing_objects_listから引き出してきて、writerへ投げて書き込む
      //※物品を発見した時刻を書き込むか、人が家具に接近した時刻を書き込むかは議論の余地あり
      //とりあえず現在は物品を発見した時刻を書き込むことにする
      if (srv1.response.object_id.empty() != true)
      {
        for (unsigned int j = 0; j < srv1.response.object_id.size(); j++)
        {
          printf("object_id = %d, place = %d\n", srv1.response.object_id[j], srv1.response.furnitures_id[j]);
        }
      }
      else
      {
        ROS_INFO("NOT FOUND");
      }
    }
    else
    {
      ROS_ERROR("Failed to call service robot_task_find_objects");
    }
  }
}

int main(int argc, char **argv)
{
  char csv_name[1024];

  if ((fp = fopen("/home/an/catkin_ws/src/tms_sa/src/map.csv", "w")) == NULL)
  {
    printf("file open error!!\n");
    exit(EXIT_FAILURE);
  }

  if ((fp_missing_table = fopen("/home/an/catkin_ws/src/tms_sa/src/missing_table.csv", "a")) == NULL)
  {
    printf("file open error!!\n");
    exit(EXIT_FAILURE);
  }

  ros::init(argc, argv, "tms_sa_missing_objects_rev2");
  ros::NodeHandle n;
  ROS_INFO("tms_sa_missing_objects_rev2 : init");

  // multithreadspinner
  ros::MultiThreadedSpinner spinner(2);

  //スケジューラからの起動の場合
  ros::Timer timer1 = n.createTimer(ros::Duration(5.0), callback1);
  //	ros::Timer timer2 = n.createTimer(ros::Duration(15.0), callback2);

  //人消失時の起動の場合
  service2 = n.advertiseService("tms_utility_analyze_missing_object", tms_utility_analyze_missing_objects_func);

  // client0X ... DBとのやりとり
  client01 = n.serviceClient< tms_msg_db::tmsdb_get_objects_info >("tmsdb_get_objects_info_present");
  client02 = n.serviceClient< tms_msg_db::tmsdb_get_objects_info >("tmsdb_get_objects_info_history");
  client03 = n.serviceClient< tms_msg_db::tmsdb_get_person_info >("tmsdb_get_person_info_2");

  client001 = n.serviceClient< tms_msg_db::tmsdb_get_person_behavior_info >("tmsdb_get_person_info_3");

  // clisent1X ... DB以外とのやりとり
  client4 = n.serviceClient< tms_msg_ts::tms_sa_find_objects >("tms_sa_find_objects_TS");  //"TS" = Task Scheduler
  client5 = n.serviceClient< tms_msg_sa::tms_to_modify_missing_objects >("tms_utility_modify_missing_objects");

  //	client5 = n.serviceClient<tmsdb::tmsdb_change_dt>("object_detection_test");
  //	client2 = n.serviceClient<tmsdb::tmsdb_modify_person_behavior>("tmsdb_modify_person_behavior");
  //	client3 = n.serviceClient<tmsdb::tmsdb_modify_missing_objects>("tmsdb_modify_missing_objects");

  // missing_tableの初期化
  //ここでは、各物品&各家具(もしくは場所)毎に物品放置度を保持している
  //扱いやすいようにresizeしたのちに利用する。
  // missing_table[物品(51-60)][家具(12,13,14,...)]
  // missing_tableでは放置度のみ扱うものとし、他の要素を組み合わせる場合(例えば物品紛失時刻管理)、別の配列を定義して扱う。

  //このリサイズは物品数で決めている
  missing_table.resize(10);
  for (unsigned int i = 0; i < 10; i++)
    missing_table[i].resize(10);

  for (unsigned int i = 0; i < 10; i++)
  {
    for (unsigned int j = 0; j < 10; j++)
    {
      missing_table[i][j] = 0;
    }
  }

  for (unsigned int i = 0; i < 10; i++)
  {
    for (unsigned int j = 0; j < 10; j++)
    {
      printf("%lu ", missing_table[i][j]);
    }
    printf("\n");
  }

  //このリサイズは物品数で決まる
  //今は51~60の合計10個
  ana_time.resize(10);

  map.resize((environment_x / block_size) + 1);  // 451作る

  for (unsigned int i = 0; i < (environment_x / block_size) + 1; i++)
  {
    map[i].resize((environment_y / block_size) + 1);  // yを400に戻す
    //即ちMAP_TABLEを[450]*[400]マスに変更
  }

  //ここから先はmap_init()で関数化したほうがいいかもしれぬ
  // mapの初期化(map_init(map))
  for (unsigned int i = 0; i < (environment_x / block_size) + 1; i++)
  {
    for (unsigned int j = 0; j < (environment_y / block_size) + 1; j++)
    {
      map[i][j].bed = false;
      map[i][j].desk = false;
      map[i][j].table = false;
      map[i][j].bed_distance = 100000;
      map[i][j].desk_distance = 100000;
      map[i][j].table_distance = 100000;
      map[i][j].bed_distance_d = 100000.0;
      map[i][j].desk_distance_d = 100000.0;
      map[i][j].table_distance_d = 100000.0;
      map[i][j].near_bed_flag = false;
      map[i][j].near_desk_flag = false;
      map[i][j].near_table_flag = false;
    }
  }
  ROS_INFO("%ld", map.size());
  ROS_INFO("%ld", map.front().size());

  sleep(1);

  // objectフラグを立てる
  // bed_flag
  for (unsigned int j = bed12_yld / block_size; j < bed12_ylu / block_size; j++)
  {
    map[bed12_xld / block_size][j].bed = true;
  }
  for (unsigned int i = bed12_xld / block_size; i < bed12_xrd / block_size; i++)
  {
    map[i][bed12_yld / block_size].bed = true;
  }
  for (unsigned int i = bed12_xlu / block_size; i < bed12_xru / block_size; i++)
  {
    map[i][bed12_ylu / block_size].bed = true;
  }
  for (unsigned int j = bed12_yrd / block_size; j < bed12_yru / block_size + 1; j++)
  {
    map[bed12_xrd / block_size][j].bed = true;
  }

  // desk_flag
  for (unsigned int j = desk13_yld / block_size; j < desk13_ylu / block_size; j++)
  {
    map[desk13_xld / block_size][j].desk = true;
  }
  for (unsigned int i = desk13_xld / block_size; i < desk13_xrd / block_size; i++)
  {
    map[i][desk13_yld / block_size].desk = true;
  }
  for (unsigned int i = desk13_xlu / block_size; i < desk13_xru / block_size; i++)
  {
    map[i][desk13_ylu / block_size].desk = true;
  }
  for (unsigned int j = desk13_yrd / block_size; j < desk13_yru / block_size + 1; j++)
  {
    map[desk13_xrd / block_size][j].desk = true;
  }
  // table_flag
  for (unsigned int j = table14_yld / block_size; j < table14_ylu / block_size; j++)
  {
    map[table14_xld / block_size][j].table = true;
  }
  for (unsigned int i = table14_xld / block_size; i < table14_xrd / block_size; i++)
  {
    map[i][table14_yld / block_size].table = true;
  }
  for (unsigned int i = table14_xlu / block_size; i < table14_xru / block_size; i++)
  {
    map[i][table14_ylu / block_size].table = true;
  }
  for (unsigned int j = table14_yrd / block_size; j < table14_yru / block_size + 1; j++)
  {
    map[table14_xrd / block_size][j].table = true;
  }

  // distance_init()
  for (unsigned int i = 0; i < map.size(); i++)
  {
    for (unsigned int j = 0; j < map[i].size(); j++)
    {
      if (map[i][j].bed == true)
      {
        map[i][j].bed_distance = 0;
        map[i][j].bed_distance_d = 0.0;
      }
      if (map[i][j].desk == true)
      {
        map[i][j].desk_distance = 0;
        map[i][j].desk_distance_d = 0.0;
      }
      if (map[i][j].table == true)
      {
        map[i][j].table_distance = 0;
        map[i][j].table_distance_d = 0.0;
      }
    }
  }
  //ここまでmap_init()で関数化する？

  //ここからmap_calc_raster()
  //距離計算(ラスタ走査"行き")
  ROS_INFO("calc raster distance int start");
  for (unsigned int i = 1; i < map.size() - 1; i++)
  {
    for (unsigned int j = 1; j < map[i].size() - 1; j++)
    {
      if (map[i][j].bed == false)
      {
        map[i][j].bed_distance =
            std::min(std::min(map[i - 1][j].bed_distance + block_size, map[i][j - 1].bed_distance + block_size),
                     map[i][j].bed_distance);
      }
      if (map[i][j].desk == false)
      {
        map[i][j].desk_distance =
            std::min(std::min(map[i - 1][j].desk_distance + block_size, map[i][j - 1].desk_distance + block_size),
                     map[i][j].desk_distance);
      }
      if (map[i][j].table == false)
      {
        map[i][j].table_distance =
            std::min(std::min(map[i - 1][j].table_distance + block_size, map[i][j - 1].table_distance + block_size),
                     map[i][j].table_distance);
      }
    }
  }
  //距離計算(ラスタ走査"帰り")
  for (unsigned int i = map.size() - 2; i > 0; i--)
  {
    for (unsigned int j = map[i].size() - 2; j > 0; j--)
    {
      if (map[i][j].bed == false)
      {
        map[i][j].bed_distance = std::min(std::min(map[i][j].bed_distance, map[i][j + 1].bed_distance + block_size),
                                          map[i + 1][j].bed_distance + block_size);
      }
      if (map[i][j].desk == false)
      {
        map[i][j].desk_distance = std::min(std::min(map[i][j].desk_distance, map[i][j + 1].desk_distance + block_size),
                                           map[i + 1][j].desk_distance + block_size);
      }
      if (map[i][j].table == false)
      {
        map[i][j].table_distance =
            std::min(std::min(map[i][j].table_distance, map[i][j + 1].table_distance + block_size),
                     map[i + 1][j].table_distance + block_size);
      }
    }
  }
  ROS_INFO("calc raster distance int finished");

  ROS_INFO("calc raster distance double start");
  for (unsigned int i = 1; i < map.size() - 1; i++)
  {
    for (unsigned int j = 1; j < map[i].size() - 1; j++)
    {
      if (!map[i][j].bed)
      {
        map[i][j].bed_distance_d = std::min(std::min(std::min(map[i - 1][j - 1].bed_distance_d + sqrt(2.0) * block_size,
                                                              map[i][j - 1].bed_distance_d + 1.0 * block_size),
                                                     std::min(map[i - 1][j + 1].bed_distance_d + sqrt(2.0) * block_size,
                                                              map[i - 1][j].bed_distance_d + 1.0 * block_size)),
                                            map[i][j].bed_distance_d);
      }
      if (!map[i][j].desk)
      {
        map[i][j].desk_distance_d =
            std::min(std::min(std::min(map[i - 1][j - 1].desk_distance_d + sqrt(2.0) * block_size,
                                       map[i][j - 1].desk_distance_d + 1.0 * block_size),
                              std::min(map[i - 1][j + 1].desk_distance_d + sqrt(2.0) * block_size,
                                       map[i - 1][j].desk_distance_d + 1.0 * block_size)),
                     map[i][j].desk_distance_d);
      }
      if (!map[i][j].table)
      {
        map[i][j].table_distance_d =
            std::min(std::min(std::min(map[i - 1][j - 1].table_distance_d + sqrt(2.0) * block_size,
                                       map[i][j - 1].table_distance_d + 1.0 * block_size),
                              std::min(map[i - 1][j + 1].table_distance_d + sqrt(2.0) * block_size,
                                       map[i - 1][j].table_distance_d + 1.0 * block_size)),
                     map[i][j].table_distance_d);
      }
    }
  }
  for (int i = map.size() - 2; i > 0; i--)
  {
    for (int j = map[i].size() - 2; j > 0; j--)
    {
      if (!map[i][j].bed)
      {
        map[i][j].bed_distance_d =
            std::min(std::min(std::min(map[i + 1][j].bed_distance_d + 1.0 * block_size,
                                       map[i + 1][j - 1].bed_distance_d + sqrt(2.0) * block_size),
                              std::min(map[i][j + 1].bed_distance_d + 1.0 * block_size,
                                       map[i + 1][j + 1].bed_distance_d + sqrt(2.0) * block_size)),
                     map[i][j].bed_distance_d);
      }
      if (!map[i][j].desk)
      {
        map[i][j].desk_distance_d =
            std::min(std::min(std::min(map[i + 1][j].desk_distance_d + 1.0 * block_size,
                                       map[i + 1][j - 1].desk_distance_d + sqrt(2.0) * block_size),
                              std::min(map[i][j + 1].desk_distance_d + 1.0 * block_size,
                                       map[i + 1][j + 1].desk_distance_d + sqrt(2.0) * block_size)),
                     map[i][j].desk_distance_d);
      }
      if (!map[i][j].table)
      {
        map[i][j].table_distance_d =
            std::min(std::min(std::min(map[i + 1][j].table_distance_d + 1.0 * block_size,
                                       map[i + 1][j - 1].table_distance_d + sqrt(2.0) * block_size),
                              std::min(map[i][j + 1].table_distance_d + 1.0 * block_size,
                                       map[i + 1][j + 1].table_distance_d + sqrt(2.0) * block_size)),
                     map[i][j].table_distance_d);
      }
    }
  }
  ROS_INFO("calc raster distance double finished");
  //ここまでmap_calc_raster()

  //ここからset_collision()
  for (unsigned int i = 0; i < map.size(); i++)
  {
    for (unsigned int j = 0; j < map[i].size(); j++)
    {
      if ((map[i][j].bed) || (map[i][j].bed_distance_d <= near_aria))
      {
        map[i][j].near_bed_flag = true;
      }
      if ((map[i][j].desk) || (map[i][j].desk_distance_d <= near_aria))
        map[i][j].near_desk_flag = true;
      if ((map[i][j].table) || (map[i][j].table_distance_d <= near_aria))
        map[i][j].near_table_flag = true;
    }
  }

  //ここまでset_collision()

  //	for(unsigned int i=0;i<map.size();i++){
  //		for(unsigned int j=0;j<map.front().size();j++){
  //			if(map[i][j].bed == true){
  //				fprintf(fp, "1, ");
  //			}
  //			else if(map[i][j].desk == true){
  //				fprintf(fp, "2, ");
  //			}
  //			else if(map[i][j].table == true){
  //				fprintf(fp, "3, ");
  //			}
  //			else{
  //				fprintf(fp, "0, ");
  //			}
  //		}
  //		fprintf(fp, "\n");
  //	}

  //	for(unsigned int i=0;i<map.size();i++){
  //		for(unsigned int j=0;j<map[i].size();j++){
  //			fprintf(fp, "%lf, ", map[i][j].bed_distance_d);
  //		}
  //		fprintf(fp, "\n");
  //	}

  for (unsigned int i = 0; i < map.size(); i++)
  {
    for (unsigned int j = 0; j < map[i].size(); j++)
    {
      if (map[i][j].bed || map[i][j].desk || map[i][j].table)
        fprintf(fp, "8, ");
      else if (map[i][j].near_bed_flag || map[i][j].near_desk_flag || map[i][j].near_table_flag)
      {
        fprintf(fp, "1, ");
      }
      else
        fprintf(fp, "0, ");
    }
    fprintf(fp, "\n");
  }

  ROS_INFO("tms_sa_missing_objects : wait");
  // ros::spin();
  spinner.spin();

  return 0;
}
