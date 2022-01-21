/* rps_give.cpp
 *
 *  Created on: 2014/05/21
 *      Author: hashiguchi
 *
 *      rps_goal_plannerを踏襲し，
 *      サブタスクgiveを実現するサービス(テスト実装) */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rps_voronoi.h"
#include <tms_msg_rp/rps_map_full.h>
#include <tms_msg_rp/rps_goal_planning.h>
#include <tms_msg_rp/rps_cnoid_grasp_obj_planning.h>
#include <tms_msg_rp/rps_select_path.h>
#include <tms_msg_db/TmsdbGetData.h>
#include <tms_msg_db/Tmsdb.h>
#include <sstream>

#define USE_TMS_DB

using namespace std;

ros::ServiceClient get_data_client;
bool is_sim;

vector< vector< CollisionMapData > > sub_Map;
vector< ManipulabilityMapData > manip_Map;

void set_RPS_MAP(const tms_msg_rp::rps_map_full::ConstPtr& RPS_MAP)
{
  sub_Map.clear();

  x_llimit = RPS_MAP->x_llimit;
  x_ulimit = RPS_MAP->x_ulimit;
  y_llimit = RPS_MAP->y_llimit;
  y_ulimit = RPS_MAP->y_ulimit;
  cell_size = RPS_MAP->cell_size;

  vector< CollisionMapData > temp_map_line;
  CollisionMapData temp_map_d;

  for (unsigned int x = 0; x < RPS_MAP->rps_map_x.size(); x++)
  {
    temp_map_line.clear();
    for (unsigned int y = 0; y < RPS_MAP->rps_map_x[x].rps_map_y.size(); y++)
    {
      temp_map_d.object = RPS_MAP->rps_map_x[x].rps_map_y[y].object;
      temp_map_d.collision = RPS_MAP->rps_map_x[x].rps_map_y[y].object;
      temp_map_d.voronoi = RPS_MAP->rps_map_x[x].rps_map_y[y].voronoi;
      temp_map_d.dist_from_obj = RPS_MAP->rps_map_x[x].rps_map_y[y].dist_from_obj_f;

      temp_map_line.push_back(temp_map_d);
    }
    sub_Map.push_back(temp_map_line);
  }
}

void removeCollisionPos(vector< vector< CollisionMapData > > Map, vector< vector< double > > in_posList,
                        vector< vector< double > >& out_posList)
{
  out_posList.clear();

  int temp_x, temp_y;

  for (unsigned int i = 0; i < in_posList.size(); i++)
  {
    temp_x = (int)round((in_posList[i][0] - x_llimit) / cell_size);
    temp_y = (int)round((in_posList[i][1] - y_llimit) / cell_size);

    if ((temp_x < 0) || (temp_x > Map.size() - 1) || (temp_y < 0) || (temp_y > Map[0].size() - 1))
      continue;
    if (Map[temp_x][temp_y].collision)
      continue;
    else
      out_posList.push_back(in_posList[i]);
  }
}

void expandRobotObjDist(vector< double > objPos, double expand_dist, vector< vector< double > >& out_posList)
{
  if (out_posList.empty())
  {
    cout << "Error <expandRobotObjDist>: posList is not set" << endl;
    return;
  }

  double temp_th;
  for (unsigned int i = 0; i < out_posList.size(); i++)
  {
    temp_th = atan2(out_posList[i][1] - objPos[1], out_posList[i][0] - objPos[0]);
    out_posList[i][0] += expand_dist * cos(temp_th);
    out_posList[i][1] += expand_dist * sin(temp_th);
  }
}

// human_pos = face_center_pos(m)
// human_height = 170 or 160 or 150
bool unifyManipulabilityMap(vector< double > human_pos, double human_th, int human_height, char robot_LR,
                            bool robot_useWaist, char human_LR, bool human_useWaist, string& message)
{  // LR_flg:0=left, 1=right
  manip_Map.resize(human_height + 1);

  char home_dir[255];
  strcpy(home_dir, getenv("HOME"));

  string dirPath;
  dirPath = home_dir;
  dirPath += "/catkin_ws/src/ros_tms/tms_rp/tms_rp_rps/rps_planner/manipulabilityMAP/";

  FILE* fp_r, *fp_h;
  char fname_r[256], fname_h[256];
  string temp_name = dirPath;
  char* tp_r, *tp_h;
  char buff[4096];
  int CommaCount = 0;
  double temp_z = 0.0;

  double human_z_offset =
      (double)human_height - 10.0 - human_pos[2] * 100.0;  // 10.0:頭のてっぺんから目の高さまでの距離

  // initial manip_Map
  for (int i = 0; i < manip_Map.size(); i++)
  {
    manip_Map[i].reachable = false;
    manip_Map[i].robot_manipulability = 0.0;
    manip_Map[i].human_manipulability = 0.0;
    manip_Map[i].peak_manipulability = 0.0;
    manip_Map[i].total_manipulability = 0.0;

    manip_Map[i].obj_pos.resize(3);
    manip_Map[i].robot_palm_pos_rel.resize(3);

    manip_Map[i].dist_from_obj_to_robot = manip_Map[i].th_from_obj_to_robot = 0.0;

    manip_Map[i].human_joint_angle.clear();
    manip_Map[i].robot_joint_angle.clear();
  }

  ////////// read human manipulability map //////////
  int Human_numJoints = 34;

  stringstream ss;
  ss << dirPath << "human/Human_" << human_height;
  temp_name = ss.str();
  //~ temp_name += "human/Human" + hp->targetHuman->bodyItemHuman->name();
  if (human_useWaist)
    temp_name += "/withWaist";
  else
    temp_name += "/noWaist";

  temp_name += "/max_param.csv";
  sprintf(fname_h, temp_name.c_str());
  while (1)
  {
    if ((fp_h = fopen(fname_h, "r")) == NULL)
    {
      message = "human manipulability map file cannot open";
      ROS_ERROR(message.c_str());
      return false;
    }
    else
      break;
  }

  while (fgets(buff, 4096, fp_h) != NULL)
  {
    ManipulabilityMapData temp_map_data;
    temp_map_data.obj_pos.resize(3);
    temp_map_data.robot_palm_pos_rel.resize(3);
    temp_map_data.human_joint_angle.clear();
    temp_map_data.robot_joint_angle.clear();
    CommaCount = 0;
    tp_h = strtok(buff, ",");
    while (tp_h != NULL)
    {
      if ((CommaCount >= 0) && (CommaCount <= 2))
        temp_map_data.obj_pos[CommaCount] = atof(tp_h);
      else if (CommaCount == 6)
      {
        temp_map_data.human_manipulability = atof(tp_h);
        temp_map_data.peak_manipulability = atof(tp_h);
      }
      else if ((CommaCount >= 7) && (CommaCount < 7 + Human_numJoints))
        temp_map_data.human_joint_angle.push_back(deg2rad(atof(tp_h)));
      else if (CommaCount == 7 + Human_numJoints)
      {
        temp_map_data.total_manipulability = atof(tp_h);
      }
      else if (CommaCount == 7 + Human_numJoints + 1)
        break;
      tp_h = strtok(NULL, ",");
      CommaCount++;
    }
    temp_z = round(temp_map_data.obj_pos[2] * 100 - human_z_offset);
    if ((int)temp_z > 0)
    {
      temp_map_data.reachable = true;
      manip_Map[(int)temp_z] = temp_map_data;
      if (human_LR == 'L')
      {
        temp_map_data.obj_pos[1] *= -1.0;
        manip_Map[(int)temp_z].human_joint_angle[0] = -temp_map_data.human_joint_angle[0];
        for (unsigned int i = 0; i < 8; i++)
        {
          if ((i == 1) || (i == 2) || (i == 4) || (i == 6))
          {
            temp_map_data.human_joint_angle[2 + i] *= -1.0;
            temp_map_data.human_joint_angle[2 + i + 8] *= -1.0;
          }
          manip_Map[(int)temp_z].human_joint_angle[2 + i] = temp_map_data.human_joint_angle[2 + i + 8];
          manip_Map[(int)temp_z].human_joint_angle[2 + i + 8] = temp_map_data.human_joint_angle[2 + i];
        }
      }
      manip_Map[(int)temp_z].obj_pos[0] =
          human_pos[0] + (temp_map_data.obj_pos[0] * cos(human_th) - temp_map_data.obj_pos[1] * sin(human_th));
      manip_Map[(int)temp_z].obj_pos[1] =
          human_pos[1] + (temp_map_data.obj_pos[0] * sin(human_th) + temp_map_data.obj_pos[1] * cos(human_th));
      manip_Map[(int)temp_z].obj_pos[2] = temp_z / 100.0;
    }
  }
  fclose(fp_h);
  cout << "finish read " << fname_h << endl;

  //~ FILE *fp_out;
  //~ const char	*fname_out = "./manip_Map_test0.csv";
  //~ fp_out = fopen( fname_out, "w" );
  //~ for(unsigned int z=0;z<manip_Map.size();z++){
  //~ fprintf( fp_out, "%d,%f,%f,%f,%f\n", z, manip_Map[z].obj_pos[0], manip_Map[z].obj_pos[1],
  // manip_Map[z].dist_from_obj_to_robot, manip_Map[z].total_manipulability);
  //~ }
  //~ fclose( fp_out );
  //~ printf( "%sファイル書き込みが終わりました\n", fname_out );

  ////////// read robot manipulability map //////////
  int Robot_numJoints = 18;  // ManipulabilityMap ロボット特有のところ!!
  temp_name = dirPath;
  temp_name += "robot/SmartPal5";  // ManipulabilityMap ロボット特有のところ!!
  if (robot_useWaist)
    temp_name += "/withWaist";
  else
    temp_name += "/noWaist";

  temp_name += "/max_param.csv";
  sprintf(fname_r, temp_name.c_str());
  while (1)
  {
    if ((fp_r = fopen(fname_r, "r")) == NULL)
    {
      message = "robot manipulability map file cannot open";
      ROS_ERROR(message.c_str());
      return false;
    }
    else
      break;
  }

  while (fgets(buff, 4096, fp_r) != NULL)
  {
    ManipulabilityMapData temp_map_data;
    temp_map_data.obj_pos.resize(3);
    temp_map_data.robot_palm_pos_rel.resize(3);
    temp_map_data.human_joint_angle.clear();
    temp_map_data.robot_joint_angle.clear();
    CommaCount = 0;
    tp_r = strtok(buff, ",");
    while (tp_r != NULL)
    {
      if ((CommaCount >= 0) && (CommaCount <= 2))
        temp_map_data.obj_pos[CommaCount] = atof(tp_r);
      else if ((CommaCount >= 3) && (CommaCount <= 5))
        temp_map_data.robot_palm_pos_rel[CommaCount - 3] = atof(tp_r);
      else if (CommaCount == 6)
        temp_map_data.robot_manipulability = atof(tp_r);
      else if ((CommaCount >= 7) && (CommaCount < 7 + Robot_numJoints))
        temp_map_data.robot_joint_angle.push_back(deg2rad(atof(tp_r)));
      else if (CommaCount == 7 + Robot_numJoints)
      {
        temp_map_data.total_manipulability = atof(tp_r);
      }
      else if (CommaCount == 7 + Robot_numJoints + 1)
        break;
      tp_r = strtok(NULL, ",");
      CommaCount++;
    }

    temp_z = round(temp_map_data.obj_pos[2] * 100.0);
    if (!manip_Map[(int)temp_z].reachable)
      continue;
    manip_Map[(int)temp_z].robot_manipulability = temp_map_data.robot_manipulability;
    manip_Map[(int)temp_z].peak_manipulability += temp_map_data.robot_manipulability;
    manip_Map[(int)temp_z].robot_palm_pos_rel = temp_map_data.robot_palm_pos_rel;
    manip_Map[(int)temp_z].robot_joint_angle = temp_map_data.robot_joint_angle;
    if (robot_LR == 'L')
    {
      temp_map_data.obj_pos[1] *= -1.0;
      manip_Map[(int)temp_z].robot_palm_pos_rel[1] *= -1.0;
      for (unsigned int i = 0; i < 8; i++)
      {
        if ((i == 1) || (i == 2) || (i == 4) || (i == 6))
        {
          temp_map_data.robot_joint_angle[2 + i] *= -1.0;
          temp_map_data.robot_joint_angle[2 + i + 8] *= -1.0;
        }
        manip_Map[(int)temp_z].robot_joint_angle[2 + i] = temp_map_data.robot_joint_angle[2 + i + 8];
        manip_Map[(int)temp_z].robot_joint_angle[2 + i + 8] = temp_map_data.robot_joint_angle[2 + i];
      }
    }
    manip_Map[(int)temp_z].dist_from_obj_to_robot = sqrt((temp_map_data.obj_pos[0] * temp_map_data.obj_pos[0]) +
                                                         (temp_map_data.obj_pos[1] * temp_map_data.obj_pos[1]));
    manip_Map[(int)temp_z].th_from_obj_to_robot = atan2(temp_map_data.obj_pos[1], temp_map_data.obj_pos[0]);
    manip_Map[(int)temp_z].total_manipulability += temp_map_data.total_manipulability;
  }
  fclose(fp_r);
  printf("finish read %s\n", fname_r);

  for (unsigned int i = 0; i < manip_Map.size(); i++)
  {
    if ((manip_Map[i].human_manipulability == 0.0) || (manip_Map[i].robot_manipulability == 0.0))
      manip_Map[i].reachable = false;
  }

  //~ FILE *fp_out_h;
  //~ const char	*fname_out_h = "./manip_Map_test.csv";
  //~ fp_out_h = fopen( fname_out_h, "w" );
  //~ for(unsigned int z=0;z<manip_Map.size();z++){
  //~ fprintf( fp_out_h, "%d,%f,%f,%f,%f\n", z, manip_Map[z].obj_pos[0], manip_Map[z].obj_pos[1],
  // manip_Map[z].dist_from_obj_to_robot, manip_Map[z].total_manipulability);
  //~ }
  //~ fclose( fp_out_h );
  //~ printf( "%sファイル書き込みが終わりました\n", fname_out_h );

  return true;
}

void QSort_manip_map_peak(vector< ManipulabilityMapData >& m_Map, int left, int right)
{
  int i, j;
  double pivot;
  ManipulabilityMapData m_data;

  i = left;
  j = right;

  pivot = m_Map[(left + right) / 2].peak_manipulability;

  while (1)
  {
    while (m_Map[i].peak_manipulability > pivot)
      i++;
    while (pivot > m_Map[j].peak_manipulability)
      j--;
    if (i >= j)
      break;

    // swap
    m_data = m_Map[i];
    m_Map[i] = m_Map[j];
    m_Map[j] = m_data;

    i++;
    j--;
  }

  if (left < i - 1)
    QSort_manip_map_peak(m_Map, left, i - 1);
  if (j + 1 < right)
    QSort_manip_map_peak(m_Map, j + 1, right);
}

void QSort_manip_map_total(vector< ManipulabilityMapData >& m_Map, int left, int right)
{
  int i, j;
  double pivot;
  ManipulabilityMapData m_data;

  i = left;
  j = right;

  pivot = m_Map[(left + right) / 2].total_manipulability;

  while (1)
  {
    while (m_Map[i].total_manipulability > pivot)
      i++;
    while (pivot > m_Map[j].total_manipulability)
      j--;
    if (i >= j)
      break;

    // swap
    m_data = m_Map[i];
    m_Map[i] = m_Map[j];
    m_Map[j] = m_data;

    i++;
    j--;
  }

  if (left < i - 1)
    QSort_manip_map_total(m_Map, left, i - 1);
  if (j + 1 < right)
    QSort_manip_map_total(m_Map, j + 1, right);
}

void QSort_give_point(vector< GivePosData >& give_point, int left, int right)
{
  int i, j;
  double pivot;
  GivePosData gp_data;

  i = left;
  j = right;

  pivot = give_point[(left + right) / 2].evaluation;

  while (1)
  {
    while (give_point[i].evaluation > pivot)
      i++;
    while (pivot > give_point[j].evaluation)
      j--;
    if (i >= j)
      break;

    // swap
    gp_data = give_point[i];
    give_point[i] = give_point[j];
    give_point[j] = gp_data;

    i++;
    j--;
  }

  if (left < i - 1)
    QSort_give_point(give_point, left, i - 1);
  if (j + 1 < right)
    QSort_give_point(give_point, j + 1, right);
}

void QSort_path_Evaluation(vector< double >& path_E, vector< int >& rank, int left, int right)
{
  int i, j, i_temp;
  double pivot, temp;

  i = left;
  j = right;

  pivot = path_E[(left + right) / 2];

  while (1)
  {
    while (path_E[i] > pivot)
      i++;
    while (pivot > path_E[j])
      j--;
    if (i >= j)
      break;

    // swap
    temp = path_E[i];
    path_E[i] = path_E[j];
    path_E[j] = temp;

    i_temp = rank[i];
    rank[i] = rank[j];
    rank[j] = i_temp;

    i++;
    j--;
  }

  if (left < i - 1)
    QSort_path_Evaluation(path_E, rank, left, i - 1);
  if (j + 1 < right)
    QSort_path_Evaluation(path_E, rank, j + 1, right);
}

bool set_person_view(vector< vector< CollisionMapData > >& Map, double person_x, double person_y, double person_th)
{
  int i_person_x = (int)round(((person_x / 1000.0) - x_llimit) / cell_size);
  int i_person_y = (int)round(((person_y / 1000.0) - y_llimit) / cell_size);

  if ((i_person_x < 0) || (i_person_x > Map.size() - 1) || (i_person_y < 0) || (i_person_y > Map[0].size() - 1))
    return false;

  double dth, temp_th;

  if (person_th >= 0)
    person_th -= (int)(person_th / PI) * PI;
  else
    person_th += (int)(-person_th / PI) * PI;
  cout << person_th << endl;
  for (int x = 0; x < Map.size(); x++)
  {
    for (int y = 0; y < Map[x].size(); y++)
    {
      Map[x][y].person_view = false;
      if ((i_person_x != x) || (i_person_y != y))
      {
        temp_th = atan2((double)(y - i_person_y), (double)(x - i_person_x));
        dth = fabs(person_th - temp_th);
        if (dth >= PI)
          dth = 2 * PI - dth;
        if (dth <= person_view_limit)
        {
          Map[x][y].person_view = true;
        }
      }
    }
  }

  return true;
}

bool set_person_view_id(vector< vector< CollisionMapData > >& Map, int person_id, vector< double >& person_pos,
                        double& person_th, string& message)
{
  // idと一致した人の最新データ(behavior, position)を取得
  person_pos.resize(3);
  tms_msg_db::TmsdbGetData srv;
  srv.request.tmsdb.id = person_id;
  if (is_sim)
    srv.request.tmsdb.sensor = 3005;
  else
    srv.request.tmsdb.sensor = 3001;

  //#ifdef USE_TMS_DB
  int ref_i = 0;
  if (get_data_client.call(srv))
  {
    for (int j = 0; j < srv.response.tmsdb.size() - 1; j++)
    {
      if (srv.response.tmsdb[j].time < srv.response.tmsdb[j + 1].time)
        ref_i = j + 1;
    }
    ROS_INFO("Success person_id:%u, behavior = %u", srv.response.tmsdb[ref_i].id, srv.response.tmsdb[ref_i].state);

    // person_state: 0 消失, 1 歩行, 2 立位静止, 3 椅子付近 静止, 4 椅子着座, 5 ベッド着座, 6 ベッド上 (休息)
    if ((srv.response.tmsdb[ref_i].state == 1) || (srv.response.tmsdb[ref_i].state == 2) ||
        (srv.response.tmsdb[ref_i].state == 3))
    {
      ROS_INFO("Success person_id:%u, x = %f, y = %f, th = %f", srv.response.tmsdb[ref_i].id,
               srv.response.tmsdb[ref_i].x, srv.response.tmsdb[ref_i].y, srv.response.tmsdb[ref_i].ry);
      if (!set_person_view(Map, srv.response.tmsdb[ref_i].x, srv.response.tmsdb[ref_i].y,
                           deg2rad(srv.response.tmsdb[ref_i].ry)))
      {
        message = "person position is out of Map";
        ROS_ERROR(message.c_str());
      }
      person_pos[0] = srv.response.tmsdb[ref_i].x / 1000.0;  // mm to m
      person_pos[1] = srv.response.tmsdb[ref_i].y / 1000.0;
      //			person_pos[2] = srv.response.tmsdb[ref_i].z / 1000.0;
      person_pos[2] = 1.5;  // 人の目の高さ，150cm
      person_th = deg2rad(srv.response.tmsdb[ref_i].ry);
    }

    if (srv.response.tmsdb[ref_i].state == 4)
    {
      if (srv.response.tmsdb[ref_i].place < 6000 || srv.response.tmsdb[ref_i].place > 7000)
      {
        message = "Person sit on unknown furniture.";
        ROS_ERROR(message.c_str());
        return false;
      }
      srv.request.tmsdb.id = srv.response.tmsdb[ref_i].place;  // 人が着座している椅子のID
      if (get_data_client.call(srv))
      {
        ROS_INFO("Success chair_x = %f, y = %f, th = %f", srv.response.tmsdb[ref_i].x, srv.response.tmsdb[ref_i].y,
                 srv.response.tmsdb[ref_i].ry);
        if (!set_person_view(Map, srv.response.tmsdb[ref_i].x, srv.response.tmsdb[ref_i].y,
                             deg2rad(srv.response.tmsdb[ref_i].ry)))
        {
          message = "person position is out of Map";
          ROS_ERROR(message.c_str());
        }
        person_pos[0] = srv.response.tmsdb[ref_i].x / 1000.0;
        person_pos[1] = srv.response.tmsdb[ref_i].y / 1000.0;
        //~ person_pos[2] = srv.response.tmsdb[ref_i].z / 1000.0;
        person_pos[2] = 1182.9 / 1000.0;  // ?
        person_th = deg2rad(srv.response.tmsdb[ref_i].ry);
      }
      else
      {
        message = "Failed to call service to get chair_info";
        ROS_ERROR(message.c_str());
        return false;
      }
    }
  }
  else
  {
    message = "Failed to call service to get person behavior data.";
    ROS_ERROR(message.c_str());
    return false;
  }
  return true;
}

bool dividePosList(unsigned int k, vector< vector< double > > in_posList,
                   vector< vector< vector< double > > >& out_posList, string& message)
{
  if (k < 0)
  {
    message = "Error <dividePosList>: number of divides < 0";
    ROS_ERROR(message.c_str());
    return false;
  }
  if (k > in_posList.size())
  {
    message = "Error <dividePosList>: posList's size < number of divides";
    ROS_ERROR(message.c_str());
    return false;
  }

  out_posList.clear();
  vector< vector< double > > tempList;
  tempList.clear();
  int j = 0;
  for (unsigned int i = 0; i < in_posList.size(); i++)
  {
    if (i < (j + 1) * (in_posList.size() / k))
    {
      tempList.push_back(in_posList[i]);
      continue;
    }
    out_posList.push_back(tempList);
    tempList.clear();
    i--;
    j++;
  }
  out_posList.push_back(tempList);
  tempList.clear();

  return true;
}

bool calcRobotPos_GiveObj(unsigned int rank, vector< vector< double > >& out_posList, vector< double >& object_pos,
                          vector< double >& robot_joint_angle, string& message)
{
  out_posList.clear();
  object_pos.clear();
  robot_joint_angle.clear();

  if (manip_Map.empty())
  {
    message = "Error <calcRobotPosAtGiveObj>: manip_Map is not set";
    ROS_ERROR(message.c_str());
    return false;
  }
  if ((rank < 0) || (rank >= manip_Map.size()))
  {
    message = "Error <calcRobotPosAtGiveObj>: set rank is invalid";
    ROS_ERROR(message.c_str());
    return false;
  }

  QSort_manip_map_peak(manip_Map, 0, manip_Map.size() - 1);
  //~ QSort_manip_map_total(manip_Map, 0, manip_Map.size()-1);

  object_pos = manip_Map[rank].obj_pos;
  robot_joint_angle = manip_Map[rank].robot_joint_angle;

  vector< double > BasePalmPos, BasePos, PalmPos;
  BasePalmPos.resize(6);
  BasePos.resize(3);
  PalmPos.resize(3);

  double dist = manip_Map[rank].dist_from_obj_to_robot;
  double th = manip_Map[rank].th_from_obj_to_robot;

  for (double i = 0; i < 2 * M_PI; i += deg2rad(1.0))
  {
    BasePos[0] = object_pos[0] + dist * cos(i);
    BasePos[1] = object_pos[1] + dist * sin(i);
    BasePos[2] = i + M_PI - th;
    if (BasePos[2] > M_PI)
      BasePos[2] -= 2 * M_PI;
    if (BasePos[2] < -M_PI)
      BasePos[2] += 2 * M_PI;

    PalmPos[0] = BasePos[0] + (manip_Map[rank].robot_palm_pos_rel[0] * cos(BasePos[2]) -
                               manip_Map[rank].robot_palm_pos_rel[1] * sin(BasePos[2]));
    PalmPos[1] = BasePos[1] + (manip_Map[rank].robot_palm_pos_rel[0] * sin(BasePos[2]) +
                               manip_Map[rank].robot_palm_pos_rel[1] * cos(BasePos[2]));
    PalmPos[2] = manip_Map[0].robot_palm_pos_rel[2];

    for (int j = 0; j < 3; j++)
    {
      BasePalmPos[j] = BasePos[j];
      BasePalmPos[j + 3] = PalmPos[j];
    }
    out_posList.push_back(BasePalmPos);
  }
  return true;
}

bool start_give_obj_pos_planner(tms_msg_rp::rps_goal_planning::Request& req,
                                tms_msg_rp::rps_goal_planning::Response& res)
{
  ROS_INFO("Start Give Obj Pos Plan...");
  res.success = 0;
  res.goal_pos.clear();
  res.joint_angle_array.clear();

  vector< double > person_pos, object_pos, robot_joint_angle;
  double person_th;
  if (!set_person_view_id(sub_Map, req.target_id, person_pos, person_th, res.message))
    return false;
  //	if(!set_table_area_id(sub_Map, 14, res.message)) // <- テーブルとの干渉判定用
  //		return false;

  vector< vector< double > > giveObjPos;
  vector< vector< vector< double > > > divide_giveObjPos, free_giveObjPos, robotPos;
  if (!unifyManipulabilityMap(person_pos, person_th, person_height, useRobotHand, useRobotWaist, useHumanHand,
                              useHumanWaist, res.message))
    return false;
  if (!calcRobotPos_GiveObj(0, giveObjPos, object_pos, robot_joint_angle, res.message))
    return false;

  vector< int > i_obj_pos;
  i_obj_pos.resize(3);
  double put_obj_offset = 100.0;  //(mm)

  cout << "obj_pos:" << object_pos[0] << "	" << object_pos[1] << "	" << object_pos[2] << endl;
  convertPos(object_pos[0], object_pos[1], i_obj_pos[0], i_obj_pos[1]);
  // 机にものが置けたらその上に置く処理
  //	if(sub_Map[i_obj_pos[0]][i_obj_pos[1]].table_height!=0.0){
  //		object_pos[2] = (sub_Map[i_obj_pos[0]][i_obj_pos[1]].table_height + put_obj_offset)/1000.0;
  //	}

  cout << "obj_pos:" << object_pos[0] << "	" << object_pos[1] << "	" << object_pos[2] << endl;
  res.target_pos.x = object_pos[0] * 1000.0;
  res.target_pos.y = object_pos[1] * 1000.0;
  res.target_pos.z = object_pos[2] * 1000.0;

  if (!dividePosList(give_obj_pos_divide_num, giveObjPos, divide_giveObjPos, res.message))
    return false;

  double collision_threshold = getRobotCollisionDist(req.robot_id);
  if (!setCollisionArea(sub_Map, collision_threshold, res.message))
    return false;

  free_giveObjPos.resize(divide_giveObjPos.size());
  vector< vector< GivePosData > > give_ObjPosData;
  give_ObjPosData.resize(divide_giveObjPos.size());
  GivePosData temp_GPD;
  for (unsigned int i = 0; i < divide_giveObjPos.size(); i++)
  {
    temp_GPD.joint_angle = robot_joint_angle;
    removeCollisionPos(sub_Map, divide_giveObjPos[i], free_giveObjPos[i]);

    double l = 0.1;
    while (free_giveObjPos[i].empty())
    {
      temp_GPD.joint_angle.clear();
      expandRobotObjDist(object_pos, l, divide_giveObjPos[i]);
      removeCollisionPos(sub_Map, divide_giveObjPos[i], free_giveObjPos[i]);
      if ((!free_giveObjPos[i].empty()) || l >= 0.1)
        break;
      else
        l += 0.1;
    }

    for (unsigned int j = 0; j < free_giveObjPos[i].size(); j++)
    {
      temp_GPD.robotPos = free_giveObjPos[i][j];
      temp_GPD.set_Evaluation(sub_Map, person_pos);
      give_ObjPosData[i].push_back(temp_GPD);
    }

    if (!give_ObjPosData[i].empty())
      QSort_give_point(give_ObjPosData[i], 0, give_ObjPosData[i].size() - 1);

    //~ cout<<i<<endl;
    //~ for(unsigned int j=0;j<give_ObjPosData[i].size();j++){
    //~ cout<<give_ObjPosData[i][j].robotPos[0]<<"	"<<give_ObjPosData[i][j].robotPos[1]<<"
    //"<<give_ObjPosData[i][j].robotPos[2]<<"	"<<give_ObjPosData[i][j].evaluation<<endl;
    //~ }
    //~ cout<<endl;
  }

  tms_msg_rp::rps_position robot_pos;
  tms_msg_rp::rps_joint_angle temp_joint_angle;
  for (unsigned int i = 0; i < give_ObjPosData.size(); i++)
  {
    temp_joint_angle.joint_angle.clear();
    if (!give_ObjPosData[i].empty())
    {
      robot_pos.x = give_ObjPosData[i][0].robotPos[0] * 1000.0;
      robot_pos.y = give_ObjPosData[i][0].robotPos[1] * 1000.0;
      robot_pos.th = rad2deg(give_ObjPosData[i][0].robotPos[2]);

      res.goal_pos.push_back(robot_pos);
      //~ cout<<robot_pos.x<<"	"<<robot_pos.y<<"	"<<robot_pos.th<<"	"<<give_ObjPosData[i][0].evaluation<<"
      //"<<give_ObjPosData[i][0].view<<"	"<<give_ObjPosData[i][0].human_dist<<"
      //"<<give_ObjPosData[i][0].obj_dist<<endl;

      for (unsigned int j = 0; j < give_ObjPosData[i][0].joint_angle.size(); j++)
      {
        temp_joint_angle.joint_angle.push_back(rad2deg(give_ObjPosData[i][0].joint_angle[j]));
        //~ cout<<j<<":"<<rad2deg(give_ObjPosData[i][0].joint_angle[j])<<"	";
      }
      //~ cout<<endl;
      res.joint_angle_array.push_back(temp_joint_angle);
    }
  }

  ROS_INFO("...Give Obj Pos Plan Success");

  res.success = 1;
  res.message = "Success: goal_planning";
  return true;
}

int main(int argc, char** argv)
{
  manip_Map.clear();
  sub_Map.clear();

  ros::init(argc, argv, "rps_give");
  ros::NodeHandle n;

  is_sim = false;
  std::string tms_rp_state;
  if (ros::param::has("/tms_rp_state"))
  {
    ros::param::get("/tms_rp_state", tms_rp_state);
  }

  if (tms_rp_state.compare("real") == 0)
  {
    is_sim = false;
  }
  else if (tms_rp_state.compare("simulation") == 0)
  {
    is_sim = true;
  }

  ros::Subscriber rps_map_subscriber = n.subscribe("rps_map_data", 1, set_RPS_MAP);
  ros::ServiceServer service_give_obj_pos = n.advertiseService("rps_give_obj_pos_planning", start_give_obj_pos_planner);

  get_data_client = n.serviceClient< tms_msg_db::TmsdbGetData >("/tms_db_reader");

  ros::spin();
  return 0;
}
