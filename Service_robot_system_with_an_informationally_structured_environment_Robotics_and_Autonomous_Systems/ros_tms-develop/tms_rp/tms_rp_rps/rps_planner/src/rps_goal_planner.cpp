#include "ros/ros.h"
#include "std_msgs/String.h"
//~ #include "../../rps.h"
#include "rps_voronoi.h"
#include <tms_msg_rp/rps_map_full.h>
#include <tms_msg_rp/rps_goal_planning.h>
#include <tms_msg_rp/rps_select_path.h>
#include <tms_msg_db/tmsdb_get_furnitures_info.h>
#include <tms_msg_db/tmsdb_get_movable_furnitures_info.h>
#include <tms_msg_db/tmsdb_get_person_behavior_info.h>
#include <tms_msg_db/tmsdb_get_person_info.h>
#include <tms_msg_db/tmsdb_get_robots_info.h>

#include <sstream>

#define USE_TMS_DB

using namespace std;

ros::ServiceClient commander_to_get_robots_info;
ros::ServiceClient commander_to_get_furnitures_info;
ros::ServiceClient commander_to_get_movable_furnitures_info;
ros::ServiceClient commander_to_get_person_behavior_info;
ros::ServiceClient commander_to_get_person_info;

vector< vector< CollisionMapData > > sub_Map;
vector< ManipulabilityMapData > manip_Map;

////////////////////////////////////////////////////////////////////////
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
  int Robot_numJoints = 18;
  temp_name = dirPath;
  temp_name += "robot/SmartPal5";
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

bool set_table_area(vector< vector< CollisionMapData > >& Map, double table_center_x, double table_center_y,
                    double table_th, double table_size_x, double table_size_y, double table_size_z)
{
  for (int x = 0; x < Map.size(); x++)
  {
    for (int y = 0; y < Map[x].size(); y++)
    {
      Map[x][y].table_height = 0.0;
    }
  }

  double p0[2], p1[2], p2[2];
  double _p0[2], _p1[2], _p2[2];
  double e1[2], e2[2];
  double norm1, norm2;

  double temp_p0[2], temp_p[2];
  int i_temp_p[2];

  p0[0] = -(table_size_x / 2);
  p0[1] = -(table_size_y / 2);
  _p0[0] = (p0[0] * cos(table_th) - p0[1] * sin(table_th)) + table_center_x;
  _p0[1] = (p0[0] * sin(table_th) + p0[1] * cos(table_th)) + table_center_y;

  p1[0] = (table_size_x / 2);
  p1[1] = -(table_size_y / 2);
  _p1[0] = (p1[0] * cos(table_th) - p1[1] * sin(table_th)) + table_center_x;
  _p1[1] = (p1[0] * sin(table_th) + p1[1] * cos(table_th)) + table_center_y;

  p2[0] = -(table_size_x / 2);
  p2[1] = (table_size_y / 2);
  _p2[0] = (p2[0] * cos(table_th) - p2[1] * sin(table_th)) + table_center_x;
  _p2[1] = (p2[0] * sin(table_th) + p2[1] * cos(table_th)) + table_center_y;

  norm1 = getLength(_p0[0], _p0[1], _p1[0], _p1[1]);
  e1[0] = (_p1[0] - _p0[0]) / norm1;
  e1[1] = (_p1[1] - _p0[1]) / norm1;

  norm2 = getLength(_p0[0], _p0[1], _p2[0], _p2[1]);
  e2[0] = (_p2[0] - _p0[0]) / norm2;
  e2[1] = (_p2[1] - _p0[1]) / norm2;

  for (int i = 0; i < norm2; i += 10)
  {
    temp_p0[0] = _p0[0] + e2[0] * i;
    temp_p0[1] = _p0[1] + e2[1] * i;
    for (int j = 0; j < norm1; j += 10)
    {
      temp_p[0] = temp_p0[0] + (e1[0] * j);
      temp_p[1] = temp_p0[1] + (e1[1] * j);

      i_temp_p[0] = (int)round(((temp_p[0] / 1000.0) - x_llimit) / cell_size);
      i_temp_p[1] = (int)round(((temp_p[1] / 1000.0) - y_llimit) / cell_size);

      if ((i_temp_p[0] < 0) || (i_temp_p[0] > Map.size() - 1) || (i_temp_p[1] < 0) || (i_temp_p[1] > Map[0].size() - 1))
        return false;

      Map[i_temp_p[0]][i_temp_p[1]].table_height = table_size_z;
    }
  }

  return true;
}

bool set_table_area_id(vector< vector< CollisionMapData > >& Map, int table_id, string& message)
{
  double table_th = deg2rad(0.0);
  double table_size_x = 800.0, table_size_y = 800.0, table_size_z = 700.0;

#ifdef USE_TMS_DB
  tms_msg_db::tmsdb_get_furnitures_info srv_get_f_info;
  srv_get_f_info.request.furnitures_id = table_id;
  if (commander_to_get_furnitures_info.call(srv_get_f_info))
  {
    ROS_INFO("Success target_x = %lf, y = %lf, z = %lf", srv_get_f_info.response.furniture_x,
             srv_get_f_info.response.furniture_y, srv_get_f_info.response.furniture_z);
    if (!set_table_area(Map, srv_get_f_info.response.furniture_x, srv_get_f_info.response.furniture_y,
                        deg2rad(table_th), table_size_x, table_size_y, table_size_z))
    {
      message = "table area is out of Map";
      ROS_ERROR(message.c_str());
      return false;
    }
  }
  else
  {
    message = "Failed to call service get_furnitures_info";
    ROS_ERROR(message.c_str());
    return false;
  }
#else
  if (!set_table_area(Map, 1400, 1900, deg2rad(table_th), table_size_x, table_size_y, table_size_z))
  {
    message = "table area is out of Map";
    ROS_ERROR(message.c_str());
    return false;
  }
#endif

  return true;
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
  //引数にidを使っているが,実際にtms_dbから取得できるデータはidに関わらず最新の1人分のデータ
  //つまり引数のidに何を入れても結果は変わらない

  person_pos.resize(3);
  tms_msg_db::tmsdb_get_person_behavior_info srv_get_pb_info;
  tms_msg_db::tmsdb_get_person_info srv_get_p_info;
//~ srv_get_p_info.request.id[0] = person_id;
#ifdef USE_TMS_DB
  if (commander_to_get_person_behavior_info.call(srv_get_pb_info))
  {
    ROS_INFO("Success person_id:%u, behavior = %u", srv_get_pb_info.response.id, srv_get_pb_info.response.behavior);
    /*
    person_behavior
    0 消失
    1 歩行
    2 立位静止
    3 椅子付近 静止
    4 椅子着座
    5 ベッド着座
    6 ベッド上 (休息)
    */
  }
  else
  {
    message = "Failed to call service get_person_behavior_info";
    ROS_ERROR(message.c_str());
    return false;
  }

  if ((srv_get_pb_info.response.behavior == 1) || (srv_get_pb_info.response.behavior == 2) ||
      (srv_get_pb_info.response.behavior == 3))
  {
    if (commander_to_get_person_info.call(srv_get_p_info))
    {
      ROS_INFO("Success person_id:%u, x = %f, y = %f, th = %f", srv_get_p_info.response.id[0],
               srv_get_p_info.response.x[0], srv_get_p_info.response.y[0], srv_get_p_info.response.theta[0]);
      if (!set_person_view(Map, srv_get_p_info.response.x[0], srv_get_p_info.response.y[0],
                           deg2rad(srv_get_p_info.response.theta[0])))
      {
        message = "person position is out of Map";
        ROS_ERROR(message.c_str());
      }
      person_pos[0] = srv_get_p_info.response.x[0] / 1000.0;
      person_pos[1] = srv_get_p_info.response.y[0] / 1000.0;
      person_pos[2] = srv_get_p_info.response.z[0] / 1000.0;
      person_th = deg2rad(srv_get_p_info.response.theta[0]);
    }
    else
    {
      message = "Failed to call service get_person_info";
      ROS_ERROR(message.c_str());
      return false;
    }
  }

  if (srv_get_pb_info.response.behavior == 4)
  {
    tms_msg_db::tmsdb_get_movable_furnitures_info srv_get_f_info;
    srv_get_f_info.request.furnitures_id = 23;
    if (commander_to_get_movable_furnitures_info.call(srv_get_f_info))
    {
      ROS_INFO("Success chair_x = %f, y = %f, th = %f", srv_get_f_info.response.furniture_x,
               srv_get_f_info.response.furniture_y, srv_get_f_info.response.furnitures_theta);
      if (!set_person_view(Map, srv_get_f_info.response.furniture_x, srv_get_f_info.response.furniture_y,
                           deg2rad(srv_get_f_info.response.furnitures_theta)))
      {
        message = "person position is out of Map";
        ROS_ERROR(message.c_str());
      }
      person_pos[0] = srv_get_f_info.response.furniture_x / 1000.0;
      person_pos[1] = srv_get_f_info.response.furniture_y / 1000.0;
      //~ person_pos[2] = srv_get_f_info.response.furniture_z / 1000.0;
      person_pos[2] = 1182.9 / 1000.0;
      person_th = deg2rad(srv_get_f_info.response.furnitures_theta);
    }
    else
    {
      message = "Failed to call service get_chair_info";
      ROS_ERROR(message.c_str());
      return false;
    }
  }
#else
  //~ person_pos[0] = 2500.0 / 1000.0;
  //~ person_pos[1] = 2000.0 / 1000.0;
  //~ person_pos[2] = 1500.0 / 1000.0;
  //~ person_th = deg2rad(-90);
  person_pos[0] = 1400.0 / 1000.0;
  person_pos[1] = 2500.0 / 1000.0;
  person_pos[2] = 1182.9 / 1000.0;
  person_th = deg2rad(-90);
  if (!set_person_view(Map, person_pos[0] * 1000.0, person_pos[1] * 1000.0, person_th))
  {
    message = "person position is out of Map";
    ROS_ERROR(message.c_str());
  }
#endif

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

bool start_rps_goal_planner(tms_msg_rp::rps_goal_planning::Request& req, tms_msg_rp::rps_goal_planning::Response& res)
{
  cout << "Start Goal Planning..." << endl;
  res.success = 0;
  res.goal_pos.clear();

  std::vector< double > start, target;
  tms_msg_rp::rps_position temp_pos;
  start.clear();
  start.resize(3);
  target.clear();
  target.resize(3);

  //	task	//////////////////////////////////////////////////////////////////
  cout << "task:" << req.task_id << endl;

  //	goal plan	//////////////////////////////////////////////////////////////////

  temp_pos.x = 3000.0;
  temp_pos.y = 1300.0;
  temp_pos.th = -90.0;
  res.goal_pos.push_back(temp_pos);

  //	response	//////////////////////////////////////////////////////////////////
  if (res.goal_pos.size() != 0)
  {
    for (int i = 0; i < res.goal_pos.size(); i++)
    {
      cout << "goal_candidate:"
           << "	no." << i + 1 << "	x:" << res.goal_pos[i].x << " y:" << res.goal_pos[i].y
           << " th:" << res.goal_pos[i].th << endl;
    }
  }
  else
  {
    ROS_ERROR("Goal is not found");
    res.success = 1;
    res.message = "Goal is not found";
    return true;
  }

  res.success = 1;
  res.message = "Success: goal_planning";
  return true;
}

bool start_grasp_wagon_pos_planner(tms_msg_rp::rps_goal_planning::Request& req,
                                   tms_msg_rp::rps_goal_planning::Response& res)
{
  ROS_INFO("Start Wagon Grasp Pos Plan...");
  res.success = 0;
  res.goal_pos.clear();
  res.joint_angle_array.clear();

  tms_msg_rp::rps_position robot_pos, wagon_pos;

//	get wagon info	//////////////////////////////////////////////////////////////////
#ifdef USE_TMS_DB
  tms_msg_db::tmsdb_get_movable_furnitures_info srv_get_f_info;
  srv_get_f_info.request.furnitures_id = req.target_id;
  if (commander_to_get_movable_furnitures_info.call(srv_get_f_info))
  {
    ROS_INFO("Success wagon_x = %lf, y = %lf, th = %lf", srv_get_f_info.response.furniture_x,
             srv_get_f_info.response.furniture_y, srv_get_f_info.response.furnitures_theta);
    ROS_INFO("Success wagon_width = %lf, depth = %lf, height = %lf", srv_get_f_info.response.furnitures_width,
             srv_get_f_info.response.furnitures_depth, srv_get_f_info.response.furnitures_height);

    wagon_pos.x = srv_get_f_info.response.furniture_x;
    wagon_pos.y = srv_get_f_info.response.furniture_y;
    wagon_pos.th = deg2rad(srv_get_f_info.response.furnitures_theta);

    Wagon_Size_LongSide_Length = srv_get_f_info.response.furnitures_width;
    Wagon_Size_ShortSide_Length = srv_get_f_info.response.furnitures_depth;
    Wagon_Size_Height = srv_get_f_info.response.furnitures_height;
  }
  else
  {
    ROS_ERROR("Failed to call service get_furnitures_info\n");
    res.message = "Failed to call service get_furnitures_info";
    return false;
  }
  if (srv_get_f_info.response.furnitures_state == 1)
  {  // if wagon exist(stop) : state = 1
    set_wagon_as_obstacle(sub_Map, wagon_pos);
    if (!calcDistFromObj(sub_Map, res.message))
      return false;
  }
#else
  wagon_pos.x = 3000;
  wagon_pos.y = 1000;
  wagon_pos.th = deg2rad(0.0);

  Wagon_Size_LongSide_Length = 435;
  Wagon_Size_ShortSide_Length = 310;
  Wagon_Size_Height = 1125;
  set_wagon_as_obstacle(sub_Map, wagon_pos);
  if (!calcDistFromObj(sub_Map, res.message))
    return false;
#endif

  double collision_threshold = getRobotCollisionDist(req.robot_id);

  setCollisionArea(sub_Map, collision_threshold, res.message);
  ///////////////////////////////////////////////

  vector< vector< double > > temp_posList, robot_posList;
  temp_posList.clear();
  robot_posList.clear();
  vector< double > temp_pos;
  temp_pos.resize(3);

  for (int i = 0; i < 4; i++)
  {
    if (i % 2 == 0)
    {
      temp_pos[0] =
          wagon_pos.x +
          (Wagon_Size_LongSide_Length / 2.0 + Robot_Control_Wagon_Dist) * cos(wagon_pos.th + (i / 2.0) * M_PI);
      temp_pos[1] =
          wagon_pos.y +
          (Wagon_Size_LongSide_Length / 2.0 + Robot_Control_Wagon_Dist) * sin(wagon_pos.th + (i / 2.0) * M_PI);
    }
    else
    {
      temp_pos[0] =
          wagon_pos.x +
          (Wagon_Size_ShortSide_Length / 2.0 + Robot_Control_Wagon_Dist) * cos(wagon_pos.th + (i / 2.0) * M_PI);
      temp_pos[1] =
          wagon_pos.y +
          (Wagon_Size_ShortSide_Length / 2.0 + Robot_Control_Wagon_Dist) * sin(wagon_pos.th + (i / 2.0) * M_PI);
    }
    temp_pos[2] = wagon_pos.th + (2 + i) * M_PI / 2.0;
    if (temp_pos[2] > M_PI)
      temp_pos[2] -= 2 * M_PI;
    if (temp_pos[2] < -M_PI)
      temp_pos[2] += 2 * M_PI;

    temp_pos[0] /= 1000.0;
    temp_pos[1] /= 1000.0;
    temp_posList.push_back(temp_pos);
  }
  removeCollisionPos(sub_Map, temp_posList, robot_posList);

  for (unsigned int i = 0; i < robot_posList.size(); i++)
  {
    robot_pos.x = robot_posList[i][0] * 1000.0;
    robot_pos.y = robot_posList[i][1] * 1000.0;
    robot_pos.th = rad2deg(robot_posList[i][2]);

    res.goal_pos.push_back(robot_pos);
    cout << robot_pos.x << "	" << robot_pos.y << "	" << robot_pos.th << endl;
  }

  ROS_INFO("...Grasp Wagon Pos Plan Success");

  res.success = 1;
  res.message = "Wagon Grasp Pos Plan Success";
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
  if (!set_table_area_id(sub_Map, 14, res.message))
    return false;

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
  if (sub_Map[i_obj_pos[0]][i_obj_pos[1]].table_height != 0.0)
  {
    object_pos[2] = (sub_Map[i_obj_pos[0]][i_obj_pos[1]].table_height + put_obj_offset) / 1000.0;
  }

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

bool start_select_path(tms_msg_rp::rps_select_path::Request& req, tms_msg_rp::rps_select_path::Response& res)
{
  ROS_INFO("Start Select Path...");
  res.success = 0;
  res.out_path.clear();
  res.out_path_rank.clear();

  vector< double > person_pos;
  double person_th;
  if (!set_person_view_id(sub_Map, req.person_id, person_pos, person_th, res.message))
    return false;
  if (!set_table_area_id(sub_Map, 14, res.message))
    return false;

  vector< vector< double > > temp_path, temp_comp_path;
  temp_path.clear();
  temp_comp_path.clear();
  vector< double > temp_pos;
  temp_pos.clear();

  vector< int > rank;
  rank.clear();
  vector< double > L, R, V, E;
  L.clear();
  R.clear();
  V.clear();
  E.clear();
  double temp_L, temp_R, temp_V, min_L = 1.0e8, min_R = 1.0e8;

  for (unsigned int i = 0; i < req.in_path.size(); i++)
  {
    temp_path.clear();
    temp_comp_path.clear();
    temp_pos.clear();
    for (unsigned int j = 0; j < req.in_path[i].rps_route.size(); j++)
    {
      temp_pos.clear();

      temp_pos.push_back(req.in_path[i].rps_route[j].x / 1000.0);
      temp_pos.push_back(req.in_path[i].rps_route[j].y / 1000.0);
      temp_pos.push_back(deg2rad(req.in_path[i].rps_route[j].th));

      temp_path.push_back(temp_pos);
    }

    compVoronoiPath(temp_path, temp_comp_path);

    temp_L = temp_R = 0.0;
    temp_V = sub_Map[(int)round((temp_comp_path[0][0] - x_llimit) /
                                cell_size)][(int)round((temp_comp_path[0][1] - y_llimit) / cell_size)].person_view;
    for (unsigned int j = 1; j < temp_comp_path.size(); j++)
    {
      temp_L += fabs(
          getLength(temp_comp_path[j - 1][0], temp_comp_path[j - 1][1], temp_comp_path[j][0], temp_comp_path[j][1]));
      temp_R += fabs(temp_comp_path[j - 1][2] - temp_comp_path[j][2]);
      temp_V += sub_Map[(int)round((temp_comp_path[j][0] - x_llimit) /
                                   cell_size)][(int)round((temp_comp_path[j][1] - y_llimit) / cell_size)].person_view;
    }
    L.push_back(temp_L);
    R.push_back(temp_R);
    V.push_back(req.weight_View * temp_V / temp_comp_path.size());

    if (min_L > temp_L)
      min_L = temp_L;
    if (min_R > temp_R)
      min_R = temp_R;
  }

  for (int i = 0; i < L.size(); i++)
  {
    rank.push_back(i);
    cout << "no:" << i << "	" << L[i] << "	" << R[i] << "	" << V[i] << endl;
    L[i] = req.weight_Length * (min_L / L[i]);
    R[i] = req.weight_Rotation * (min_R / R[i]);

    E.push_back(L[i] + R[i] + V[i]);

    cout << "	no:" << i << "	" << E[i] << "	" << L[i] << "	" << R[i] << "	" << V[i] << endl;
  }

  QSort_path_Evaluation(E, rank, 0, E.size() - 1);

  for (unsigned int i = 0; i < rank.size(); i++)
  {
    res.out_path_rank.push_back(rank[i]);
    res.out_path.push_back(req.in_path[rank[i]]);
  }

  ROS_INFO("...Select Path Success");

  res.success = 1;
  res.message = "Success: select path";
  return true;
}

int main(int argc, char** argv)
{
  manip_Map.clear();
  sub_Map.clear();

  ros::init(argc, argv, "rps_goal_planner");
  ros::NodeHandle n;

  ros::Subscriber rps_map_subscriber = n.subscribe("rps_map_data", 1, set_RPS_MAP);
  ros::ServiceServer service_g = n.advertiseService("rps_goal_planning", start_rps_goal_planner);
  ros::ServiceServer service_grasp_wagon_pos =
      n.advertiseService("rps_grasp_wagon_pos_planning", start_grasp_wagon_pos_planner);
  ros::ServiceServer service_give_obj_pos = n.advertiseService("rps_give_obj_pos_planning", start_give_obj_pos_planner);

  ros::ServiceServer service_select_path = n.advertiseService("rps_select_path", start_select_path);

  commander_to_get_robots_info = n.serviceClient< tms_msg_db::tmsdb_get_robots_info >("tmsdb_get_robots_info");
  commander_to_get_furnitures_info =
      n.serviceClient< tms_msg_db::tmsdb_get_furnitures_info >("tmsdb_get_furnitures_info");
  commander_to_get_movable_furnitures_info =
      n.serviceClient< tms_msg_db::tmsdb_get_movable_furnitures_info >("tmsdb_get_movable_furnitures_info");
  commander_to_get_person_behavior_info =
      n.serviceClient< tms_msg_db::tmsdb_get_person_behavior_info >("tmsdb_get_person_info_3");
  commander_to_get_person_info = n.serviceClient< tms_msg_db::tmsdb_get_person_info >("tmsdb_get_current_person_info");

  ros::spin();

  return 0;
}