/*
 * tmsdb_analyze_missing_object.cpp
 *
 *  Created on: 2012/12/11
 *      Author: an
 */

// include for ROS
#include <ros/ros.h>
#include <unistd.h>
#include <boost/date_time/local_time/local_time.hpp>

#include <tms_msg_db/tmsdb_get_objects_info.h>
#include <tms_msg_db/tmsdb_get_person_info.h>

#include <tms_msg_sa/tms_to_activate_service.h>
#include <tms_msg_ts/tms_sa_find_objects.h>
//#include <tms_utility/ods_change_dt.h>

#include <tms_msg_sa/tms_to_modify_missing_objects.h>

#define table14_x 1500
#define table14_y 2000
#define desk13_x 4000
#define desk13_y 2000
#define bed12_x 0
#define bed12_y 0
#define threshold_furnitures 1000  //家具中心からの閾値設定
#define furnitures_start_number 11

struct PERSON_HISTORY
{
  long unsigned int rostime;
  //	int id;
  float x;
  float y;
};

struct FURNITURES
{
  // std::vector<long long unsigned int> near_time;	//数回近づいた場合を考慮，基本は[0]のみ利用する
  //.empty=0の場合は接近した家具がないため例外処理へ
  long unsigned int near_time;
  // std::vector<long long unsigned int> left_time;	//数回近づいた場合を考慮
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

ros::ServiceClient client1;
ros::ServiceClient client2;
ros::ServiceClient client3;
ros::ServiceClient client4;
ros::ServiceClient client5;

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
      //			srv5.request.object_id = srv4.response.object_id;
      //			srv5.request.furnitures_id = srv4.response.furnitures_id;
      //			srv5.request.object_x = srv4.response.object_x;
      //			srv5.request.object_y = srv4.response.object_y;
      //			srv5.request.object_z = srv4.response.object_z;
      //			srv5.request.placed_time.push_back(0);
      //			if(client5.call(srv5)){
      //				ROS_INFO("success:%d",srv5.response.success);
      //				if(srv5.response.message.empty() != true)
      //					ROS_INFO("message : %s",srv5.response.message.c_str());
      //			}
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tms_sa_missing_objects");
  ros::NodeHandle n;
  printf("tms_sa_missing_objects : init\n");

  //スケジューラからの起動の場合
  //	service1 = n.advertiseService("tmsdb_analyze_missing_object_S", tms_analyze_missing_objects_func);

  //人消失時の起動の場合
  service2 = n.advertiseService("tms_utility_analyze_missing_object", tms_utility_analyze_missing_objects_func);

  client1 = n.serviceClient< tms_msg_db::tmsdb_get_objects_info >("tmsdb_get_objects_info_present");
  client2 = n.serviceClient< tms_msg_db::tmsdb_get_objects_info >("tmsdb_get_objects_info_history");
  client3 = n.serviceClient< tms_msg_db::tmsdb_get_person_info >("tmsdb_get_person_info");
  client4 = n.serviceClient< tms_msg_ts::tms_sa_find_objects >("tms_sa_find_objects_TS");  //"TS" = Task Scheduler
  client5 = n.serviceClient< tms_msg_sa::tms_to_modify_missing_objects >("tms_utility_modify_missing_objects");

  //	client5 = n.serviceClient<tmsdb::tmsdb_change_dt>("object_detection_test");
  //	client2 = n.serviceClient<tmsdb::tmsdb_modify_person_behavior>("tmsdb_modify_person_behavior");
  //	client3 = n.serviceClient<tmsdb::tmsdb_modify_missing_objects>("tmsdb_modify_missing_objects");

  printf("tms_sa_missing_objects : wait\n");

  ros::spin();

  return 0;
}
