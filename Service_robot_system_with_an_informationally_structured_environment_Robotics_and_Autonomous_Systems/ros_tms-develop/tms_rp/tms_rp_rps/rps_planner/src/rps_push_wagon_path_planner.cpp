#include "ros/ros.h"
#include "std_msgs/String.h"
//~ #include "../../rps.h"
#include "rps_voronoi.h"
#include <tms_msg_rp/rps_map_full.h>
#include <tms_msg_rp/rps_voronoi_path_planning.h>
#include <tms_msg_rp/rps_push_wagon_path_planning.h>
#include <tms_msg_rp/rps_cnoid_grasp_wagon_planning.h>
#include <tms_msg_rp/rps_robot_drive.h>
#include <tms_msg_db/tmsdb_get_movable_furnitures_info.h>
#include <tms_msg_db/tmsdb_get_robots_info.h>

#include <sstream>

#define USE_TMS_DB

using namespace std;

ros::Publisher rps_robot_path_pub;
ros::Publisher rps_wagon_path_pub;
ros::ServiceClient commander_to_get_robots_info;
ros::ServiceClient commander_to_get_movable_furnitures_info;
ros::ServiceClient commander_to_calc_grasp_wagon_pose;

vector< vector< CollisionMapData > > sub_Map;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

void getPushWagonType(tms_msg_rp::rps_position robot_pos, tms_msg_rp::rps_position wagon_pos, int& push_wagon_type,
                      double& rel_th)
{
  push_wagon_type = 0;
  rel_th = 0.0;

  double rw_th = atan2(robot_pos.y - wagon_pos.y, robot_pos.x - wagon_pos.x);
  double w_th = deg2rad(wagon_pos.th);

  vector< double > pw_th;
  pw_th.resize(2);

  pw_th[0] = atan2(Wagon_Size_ShortSide_Length / 2.0, Wagon_Size_LongSide_Length / 2.0);
  pw_th[1] = atan2(Wagon_Size_LongSide_Length / 2.0, Wagon_Size_ShortSide_Length / 2.0);

  double dth = rw_th - w_th;
  if (dth > M_PI)
    dth -= 2 * M_PI;
  if (dth < -M_PI)
    dth += 2 * M_PI;

  if ((dth >= -pw_th[0]) && (dth <= pw_th[0]))
  {
    push_wagon_type = 1;
    rel_th = w_th + M_PI - deg2rad(robot_pos.th);
  }
  else if ((dth > -pw_th[1] + (M_PI / 2.0)) && (dth < pw_th[1] + (M_PI / 2.0)))
  {
    push_wagon_type = 2;
    rel_th = w_th - (M_PI / 2.0) - deg2rad(robot_pos.th);
  }
  else if (((dth >= -M_PI) && (dth <= pw_th[0] - M_PI)) || ((dth >= -pw_th[0] + M_PI) && (dth <= M_PI)))
  {
    push_wagon_type = 3;
    rel_th = w_th - deg2rad(robot_pos.th);
  }
  else if ((dth > -pw_th[1] - (M_PI / 2.0)) && (dth < pw_th[1] - (M_PI / 2.0)))
  {
    push_wagon_type = 4;
    rel_th = w_th + (M_PI / 2.0) - deg2rad(robot_pos.th);
  }

  if (rel_th > M_PI)
    rel_th -= 2 * M_PI;
  if (rel_th < -M_PI)
    rel_th += 2 * M_PI;
}

bool calcWagonPosOnVoronoi(vector< vector< CollisionMapData > > Map, vector< double >& robot_pos, int wagon_state,
                           double control_dist, double& rel_th, bool change_rel, double threshold)
{
  if (Map.empty())
  {
    ROS_ERROR("Map is empty");
    return false;
  }

  double incre_th = deg2rad(0.1);
  vector< double > temp_r_pos, temp_w_pos, temp_control_pos;
  temp_r_pos.resize(3);
  temp_r_pos = robot_pos;
  temp_w_pos.resize(2);
  temp_control_pos.resize(2);

  vector< int > i_temp_w_pos, i_temp_control_pos;
  i_temp_w_pos.resize(2);
  i_temp_control_pos.resize(2);

  double wagon_l = 0.0, temp_rel_th = rel_th;

  switch (wagon_state)
  {
    case 1:
      wagon_l = Wagon_Size_LongSide_Length / 1000.0;
      break;

    case 2:
      wagon_l = Wagon_Size_ShortSide_Length / 1000.0;
      break;

    case 3:
      wagon_l = Wagon_Size_LongSide_Length / 1000.0;
      break;

    case 4:
      wagon_l = Wagon_Size_ShortSide_Length / 1000.0;
      break;
  }

  double temp_dist;
  double next_dist;
  double p_rot, p_dist, m_rot, m_dist;
  bool p_flg = false, m_flg = false;

  if (!change_rel)
  {
    temp_control_pos[0] = temp_r_pos[0] + (control_dist + (wagon_l / 2.0)) * cos(temp_r_pos[2]);
    temp_control_pos[1] = temp_r_pos[1] + (control_dist + (wagon_l / 2.0)) * sin(temp_r_pos[2]);
    i_temp_control_pos[0] = (int)round((temp_control_pos[0] - x_llimit) / cell_size);
    i_temp_control_pos[1] = (int)round((temp_control_pos[1] - y_llimit) / cell_size);
    temp_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
    next_dist = temp_dist;
    p_flg = false;
    m_flg = false;

    if ((!Map[i_temp_control_pos[0]][i_temp_control_pos[1]].collision) && (temp_dist > threshold))
    {
      while (next_dist <= temp_dist)
      {
        if (next_dist <= threshold)
        {
          p_flg = true;
          break;
        }
        temp_r_pos[2] += incre_th;
        temp_control_pos[0] = temp_r_pos[0] + (control_dist + (wagon_l / 2.0)) * cos(temp_r_pos[2]);
        temp_control_pos[1] = temp_r_pos[1] + (control_dist + (wagon_l / 2.0)) * sin(temp_r_pos[2]);

        i_temp_control_pos[0] = (int)round((temp_control_pos[0] - x_llimit) / cell_size);
        i_temp_control_pos[1] = (int)round((temp_control_pos[1] - y_llimit) / cell_size);
        next_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
      }
      if (!p_flg)
      {
        temp_r_pos[2] -= incre_th;
        temp_control_pos[0] = temp_r_pos[0] + (control_dist + (wagon_l / 2.0)) * cos(temp_r_pos[2]);
        temp_control_pos[1] = temp_r_pos[1] + (control_dist + (wagon_l / 2.0)) * sin(temp_r_pos[2]);

        i_temp_control_pos[0] = (int)round((temp_control_pos[0] - x_llimit) / cell_size);
        i_temp_control_pos[1] = (int)round((temp_control_pos[1] - y_llimit) / cell_size);
        next_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
      }
      p_dist = next_dist;
      p_rot = temp_r_pos[2] - robot_pos[2];
      if (next_dist == temp_dist)
        p_rot = 0.0;

      next_dist = temp_dist;
      temp_r_pos[2] = robot_pos[2];
      while (next_dist <= temp_dist)
      {
        if (next_dist <= threshold)
        {
          m_flg = true;
          break;
        }
        temp_r_pos[2] -= incre_th;
        temp_control_pos[0] = temp_r_pos[0] + (control_dist + (wagon_l / 2.0)) * cos(temp_r_pos[2]);
        temp_control_pos[1] = temp_r_pos[1] + (control_dist + (wagon_l / 2.0)) * sin(temp_r_pos[2]);

        i_temp_control_pos[0] = (int)round((temp_control_pos[0] - x_llimit) / cell_size);
        i_temp_control_pos[1] = (int)round((temp_control_pos[1] - y_llimit) / cell_size);
        next_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
      }
      if (!m_flg)
      {
        temp_r_pos[2] += incre_th;
        temp_control_pos[0] = temp_r_pos[0] + (control_dist + (wagon_l / 2.0)) * cos(temp_r_pos[2]);
        temp_control_pos[1] = temp_r_pos[1] + (control_dist + (wagon_l / 2.0)) * sin(temp_r_pos[2]);

        i_temp_control_pos[0] = (int)round((temp_control_pos[0] - x_llimit) / cell_size);
        i_temp_control_pos[1] = (int)round((temp_control_pos[1] - y_llimit) / cell_size);
        next_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
      }
      m_dist = next_dist;
      m_rot = temp_r_pos[2] - robot_pos[2];
      if (next_dist == temp_dist)
        m_rot = 0.0;

      if (p_dist == m_dist)
      {
        if (fabs(p_rot) <= fabs(m_rot))
          robot_pos[2] += p_rot;
        else
          robot_pos[2] += m_rot;
      }
      else if (p_dist <= m_dist)
        robot_pos[2] += p_rot;
      else
        robot_pos[2] += m_rot;
    }
    else
    {
      while (Map[i_temp_control_pos[0]][i_temp_control_pos[1]].collision)
      {
        temp_r_pos[2] += incre_th;
        temp_control_pos[0] = temp_r_pos[0] + (control_dist + (wagon_l / 2.0)) * cos(temp_r_pos[2]);
        temp_control_pos[1] = temp_r_pos[1] + (control_dist + (wagon_l / 2.0)) * sin(temp_r_pos[2]);

        i_temp_control_pos[0] = (int)round((temp_control_pos[0] - x_llimit) / cell_size);
        i_temp_control_pos[1] = (int)round((temp_control_pos[1] - y_llimit) / cell_size);
      }
      p_rot = temp_r_pos[2] - robot_pos[2];

      temp_r_pos[2] = robot_pos[2];
      temp_control_pos[0] = temp_r_pos[0] + (control_dist + (wagon_l / 2.0)) * cos(temp_r_pos[2]);
      temp_control_pos[1] = temp_r_pos[1] + (control_dist + (wagon_l / 2.0)) * sin(temp_r_pos[2]);

      i_temp_control_pos[0] = (int)round((temp_control_pos[0] - x_llimit) / cell_size);
      i_temp_control_pos[1] = (int)round((temp_control_pos[1] - y_llimit) / cell_size);
      while (Map[i_temp_control_pos[0]][i_temp_control_pos[1]].collision)
      {
        temp_r_pos[2] -= incre_th;
        temp_control_pos[0] = temp_r_pos[0] + (control_dist + (wagon_l / 2.0)) * cos(temp_r_pos[2]);
        temp_control_pos[1] = temp_r_pos[1] + (control_dist + (wagon_l / 2.0)) * sin(temp_r_pos[2]);

        i_temp_control_pos[0] = (int)round((temp_control_pos[0] - x_llimit) / cell_size);
        i_temp_control_pos[1] = (int)round((temp_control_pos[1] - y_llimit) / cell_size);
      }
      m_rot = temp_r_pos[2] - robot_pos[2];

      if (fabs(p_rot) <= fabs(m_rot))
        robot_pos[2] += p_rot;
      else
        robot_pos[2] += m_rot;
    }
  }

  if (change_rel)
  {
    temp_control_pos[0] = temp_r_pos[0] + control_dist * cos(temp_r_pos[2]);
    temp_control_pos[1] = temp_r_pos[1] + control_dist * sin(temp_r_pos[2]);
    i_temp_control_pos[0] = (int)round((temp_control_pos[0] - x_llimit) / cell_size);
    i_temp_control_pos[1] = (int)round((temp_control_pos[1] - y_llimit) / cell_size);
    temp_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
    next_dist = temp_dist;
    p_flg = false;
    m_flg = false;

    if ((!Map[i_temp_control_pos[0]][i_temp_control_pos[1]].collision) && (temp_dist > threshold))
    {
      while (next_dist <= temp_dist)
      {
        if (next_dist <= threshold)
        {
          p_flg = true;
          break;
        }
        temp_r_pos[2] += incre_th;
        temp_control_pos[0] = temp_r_pos[0] + control_dist * cos(temp_r_pos[2]);
        temp_control_pos[1] = temp_r_pos[1] + control_dist * sin(temp_r_pos[2]);

        i_temp_control_pos[0] = (int)round((temp_control_pos[0] - x_llimit) / cell_size);
        i_temp_control_pos[1] = (int)round((temp_control_pos[1] - y_llimit) / cell_size);
        next_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
      }
      if (!p_flg)
      {
        temp_r_pos[2] -= incre_th;
        temp_control_pos[0] = temp_r_pos[0] + control_dist * cos(temp_r_pos[2]);
        temp_control_pos[1] = temp_r_pos[1] + control_dist * sin(temp_r_pos[2]);

        i_temp_control_pos[0] = (int)round((temp_control_pos[0] - x_llimit) / cell_size);
        i_temp_control_pos[1] = (int)round((temp_control_pos[1] - y_llimit) / cell_size);
        next_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
      }
      p_dist = next_dist;
      p_rot = temp_r_pos[2] - robot_pos[2];
      if (next_dist == temp_dist)
        p_rot = 0.0;

      next_dist = temp_dist;
      temp_r_pos[2] = robot_pos[2];
      while (next_dist <= temp_dist)
      {
        if (next_dist <= threshold)
        {
          m_flg = true;
          break;
        }
        temp_r_pos[2] -= incre_th;
        temp_control_pos[0] = temp_r_pos[0] + control_dist * cos(temp_r_pos[2]);
        temp_control_pos[1] = temp_r_pos[1] + control_dist * sin(temp_r_pos[2]);

        i_temp_control_pos[0] = (int)round((temp_control_pos[0] - x_llimit) / cell_size);
        i_temp_control_pos[1] = (int)round((temp_control_pos[1] - y_llimit) / cell_size);
        next_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
      }
      if (!m_flg)
      {
        temp_r_pos[2] += incre_th;
        temp_control_pos[0] = temp_r_pos[0] + control_dist * cos(temp_r_pos[2]);
        temp_control_pos[1] = temp_r_pos[1] + control_dist * sin(temp_r_pos[2]);

        i_temp_control_pos[0] = (int)round((temp_control_pos[0] - x_llimit) / cell_size);
        i_temp_control_pos[1] = (int)round((temp_control_pos[1] - y_llimit) / cell_size);
        next_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
      }
      m_dist = next_dist;
      m_rot = temp_r_pos[2] - robot_pos[2];
      if (next_dist == temp_dist)
        m_rot = 0.0;

      if (p_dist == m_dist)
      {
        if (fabs(p_rot) <= fabs(m_rot))
          robot_pos[2] += p_rot;
        else
          robot_pos[2] += m_rot;
      }
      else if (p_dist <= m_dist)
        robot_pos[2] += p_rot;
      else
        robot_pos[2] += m_rot;
    }
    else
    {
      while (Map[i_temp_control_pos[0]][i_temp_control_pos[1]].collision)
      {
        temp_r_pos[2] += incre_th;
        temp_control_pos[0] = temp_r_pos[0] + control_dist * cos(temp_r_pos[2]);
        temp_control_pos[1] = temp_r_pos[1] + control_dist * sin(temp_r_pos[2]);

        i_temp_control_pos[0] = (int)round((temp_control_pos[0] - x_llimit) / cell_size);
        i_temp_control_pos[1] = (int)round((temp_control_pos[1] - y_llimit) / cell_size);
      }
      p_rot = temp_r_pos[2] - robot_pos[2];

      temp_r_pos[2] = robot_pos[2];
      temp_control_pos[0] = temp_r_pos[0] + control_dist * cos(temp_r_pos[2]);
      temp_control_pos[1] = temp_r_pos[1] + control_dist * sin(temp_r_pos[2]);

      i_temp_control_pos[0] = (int)round((temp_control_pos[0] - x_llimit) / cell_size);
      i_temp_control_pos[1] = (int)round((temp_control_pos[1] - y_llimit) / cell_size);
      while (Map[i_temp_control_pos[0]][i_temp_control_pos[1]].collision)
      {
        temp_r_pos[2] -= incre_th;
        temp_control_pos[0] = temp_r_pos[0] + control_dist * cos(temp_r_pos[2]);
        temp_control_pos[1] = temp_r_pos[1] + control_dist * sin(temp_r_pos[2]);

        i_temp_control_pos[0] = (int)round((temp_control_pos[0] - x_llimit) / cell_size);
        i_temp_control_pos[1] = (int)round((temp_control_pos[1] - y_llimit) / cell_size);
      }
      m_rot = temp_r_pos[2] - robot_pos[2];

      if (fabs(p_rot) <= fabs(m_rot))
        robot_pos[2] += p_rot;
      else
        robot_pos[2] += m_rot;
    }

    temp_control_pos[0] = robot_pos[0] + control_dist * cos(robot_pos[2]);
    temp_control_pos[1] = robot_pos[1] + control_dist * sin(robot_pos[2]);

    temp_w_pos[0] = temp_control_pos[0] + wagon_l * cos(robot_pos[2] + rel_th);
    temp_w_pos[1] = temp_control_pos[1] + wagon_l * sin(robot_pos[2] + rel_th);
    i_temp_w_pos[0] = (int)round((temp_w_pos[0] - x_llimit) / cell_size);
    i_temp_w_pos[1] = (int)round((temp_w_pos[1] - y_llimit) / cell_size);

    temp_dist = Map[i_temp_w_pos[0]][i_temp_w_pos[1]].dist_from_voronoi;
    next_dist = temp_dist;
    p_flg = false;
    m_flg = false;

    if ((!Map[i_temp_w_pos[0]][i_temp_w_pos[1]].collision) && (temp_dist > threshold))
    {
      while (next_dist <= temp_dist)
      {
        if (next_dist <= threshold)
        {
          p_flg = true;
          break;
        }
        if (temp_rel_th > wagon_control_limit_th)
          break;
        temp_rel_th += incre_th;
        temp_w_pos[0] = temp_control_pos[0] + wagon_l * cos(robot_pos[2] + temp_rel_th);
        temp_w_pos[1] = temp_control_pos[1] + wagon_l * sin(robot_pos[2] + temp_rel_th);
        i_temp_w_pos[0] = (int)round((temp_w_pos[0] - x_llimit) / cell_size);
        i_temp_w_pos[1] = (int)round((temp_w_pos[1] - y_llimit) / cell_size);
        next_dist = Map[i_temp_w_pos[0]][i_temp_w_pos[1]].dist_from_voronoi;
      }
      if (!p_flg)
      {
        temp_rel_th -= incre_th;
        temp_w_pos[0] = temp_control_pos[0] + wagon_l * cos(robot_pos[2] + temp_rel_th);
        temp_w_pos[1] = temp_control_pos[1] + wagon_l * sin(robot_pos[2] + temp_rel_th);
        i_temp_w_pos[0] = (int)round((temp_w_pos[0] - x_llimit) / cell_size);
        i_temp_w_pos[1] = (int)round((temp_w_pos[1] - y_llimit) / cell_size);
        next_dist = Map[i_temp_w_pos[0]][i_temp_w_pos[1]].dist_from_voronoi;
      }
      p_dist = next_dist;
      p_rot = temp_rel_th - rel_th;
      if (next_dist == temp_dist)
        p_rot = 0.0;

      next_dist = temp_dist;
      temp_rel_th = rel_th;
      while (next_dist <= temp_dist)
      {
        if (next_dist <= threshold)
        {
          m_flg = true;
          break;
        }
        if (temp_rel_th < -wagon_control_limit_th)
          break;
        temp_rel_th -= incre_th;
        temp_w_pos[0] = temp_control_pos[0] + wagon_l * cos(robot_pos[2] + temp_rel_th);
        temp_w_pos[1] = temp_control_pos[1] + wagon_l * sin(robot_pos[2] + temp_rel_th);
        i_temp_w_pos[0] = (int)round((temp_w_pos[0] - x_llimit) / cell_size);
        i_temp_w_pos[1] = (int)round((temp_w_pos[1] - y_llimit) / cell_size);
        next_dist = Map[i_temp_w_pos[0]][i_temp_w_pos[1]].dist_from_voronoi;
      }
      if (!m_flg)
      {
        temp_rel_th += incre_th;
        temp_w_pos[0] = temp_control_pos[0] + wagon_l * cos(robot_pos[2] + temp_rel_th);
        temp_w_pos[1] = temp_control_pos[1] + wagon_l * sin(robot_pos[2] + temp_rel_th);
        i_temp_w_pos[0] = (int)round((temp_w_pos[0] - x_llimit) / cell_size);
        i_temp_w_pos[1] = (int)round((temp_w_pos[1] - y_llimit) / cell_size);
        next_dist = Map[i_temp_w_pos[0]][i_temp_w_pos[1]].dist_from_voronoi;
      }
      m_dist = next_dist;
      m_rot = temp_rel_th - rel_th;
      if (next_dist == temp_dist)
        m_rot = 0.0;

      if (p_dist == m_dist)
      {
        if (fabs(p_rot) <= fabs(m_rot))
          rel_th += p_rot;
        else
          rel_th += m_rot;
      }
      else if (p_dist <= m_dist)
        rel_th += p_rot;
      else
        rel_th += m_rot;
    }
    else
    {
      while (Map[i_temp_w_pos[0]][i_temp_w_pos[1]].collision)
      {
        if (temp_rel_th > wagon_control_limit_th)
          break;
        temp_rel_th += incre_th;
        temp_w_pos[0] = temp_control_pos[0] + wagon_l * cos(robot_pos[2] + temp_rel_th);
        temp_w_pos[1] = temp_control_pos[1] + wagon_l * sin(robot_pos[2] + temp_rel_th);
        i_temp_w_pos[0] = (int)round((temp_w_pos[0] - x_llimit) / cell_size);
        i_temp_w_pos[1] = (int)round((temp_w_pos[1] - y_llimit) / cell_size);
      }
      p_rot = temp_rel_th - rel_th;

      temp_rel_th = rel_th;
      temp_w_pos[0] = temp_control_pos[0] + wagon_l * cos(robot_pos[2] + temp_rel_th);
      temp_w_pos[1] = temp_control_pos[1] + wagon_l * sin(robot_pos[2] + temp_rel_th);
      i_temp_w_pos[0] = (int)round((temp_w_pos[0] - x_llimit) / cell_size);
      i_temp_w_pos[1] = (int)round((temp_w_pos[1] - y_llimit) / cell_size);
      while (Map[i_temp_w_pos[0]][i_temp_w_pos[1]].collision)
      {
        if (temp_rel_th < -wagon_control_limit_th)
          break;
        temp_rel_th -= incre_th;
        temp_w_pos[0] = temp_control_pos[0] + wagon_l * cos(robot_pos[2] + temp_rel_th);
        temp_w_pos[1] = temp_control_pos[1] + wagon_l * sin(robot_pos[2] + temp_rel_th);
        i_temp_w_pos[0] = (int)round((temp_w_pos[0] - x_llimit) / cell_size);
        i_temp_w_pos[1] = (int)round((temp_w_pos[1] - y_llimit) / cell_size);
      }
      m_rot = temp_rel_th - rel_th;

      if (fabs(p_rot) <= fabs(m_rot))
        rel_th += p_rot;
      else
        rel_th += m_rot;
    }
  }

  return true;
}

void cut_unnecessary_path_point(vector< vector< double > > in_wagon_path, vector< vector< double > > passage_point,
                                double threshold, vector< vector< double > >& out_wagon_path)
{
  out_wagon_path.clear();
  if (in_wagon_path.empty())
  {
    cout << "Error : wagon_push_path is empty" << endl;
    return;
  }
  vector< double > temp_param = in_wagon_path[0];
  out_wagon_path.push_back(temp_param);

  double temp_th = temp_param[2], temp_rel_th = temp_param[3];
  bool cut_flg = true;
  for (unsigned int i = 1; i < in_wagon_path.size(); i++)
  {
    cut_flg = true;
    //~ if( (temp_param[2]!=in_wagon_path[i][2])||(temp_param[3]!=in_wagon_path[i][3]) )
    //~ cut_flg = false;
    if ((fabs(temp_th - in_wagon_path[i][2]) > threshold) || (fabs(temp_rel_th - in_wagon_path[i][3]) > threshold))
      cut_flg = false;
    if (cut_flg)
    {
      for (unsigned int j = 0; j < passage_point.size(); j++)
      {
        if ((passage_point[j][0] == in_wagon_path[i][0]) && (passage_point[j][1] == in_wagon_path[i][1]))
          cut_flg = false;
      }
    }
    temp_param = in_wagon_path[i];
    if (!cut_flg)
    {
      temp_th = temp_param[2];
      temp_rel_th = temp_param[3];
      out_wagon_path.push_back(temp_param);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool start_push_wagon_path_planner(tms_msg_rp::rps_push_wagon_path_planning::Request& req,
                                   tms_msg_rp::rps_push_wagon_path_planning::Response& res)
{
  ROS_INFO("Start Push Wagon Path Plan...");
  res.success = 0;

//	get wagon info	//////////////////////////////////////////////////////////////////
#ifdef USE_TMS_DB
  tms_msg_db::tmsdb_get_movable_furnitures_info srv_get_f_info;
  srv_get_f_info.request.furnitures_id = req.wagon_id;
  if (commander_to_get_movable_furnitures_info.call(srv_get_f_info))
  {
    //~ ROS_INFO("Success wagon_width = %lf, depth = %lf, height = %lf", srv_get_f_info.response.furnitures_width,
    // srv_get_f_info.response.furnitures_depth, srv_get_f_info.response.furnitures_height);

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
#else
  Wagon_Size_LongSide_Length = 435;
  Wagon_Size_ShortSide_Length = 310;
  Wagon_Size_Height = 1125;
#endif

  int push_wagon_type;
  double rel_th = 0.0;
  getPushWagonType(req.start_robot_pos, req.start_wagon_pos, push_wagon_type, rel_th);
  //~ cout<<push_wagon_type<<endl;

  ////calc robot path
  vector< double > start, goal;
  vector< vector< double > > voronoi_path, smooth_path, comp_path;
  start.resize(3);
  goal.resize(3);
  start[0] = req.start_robot_pos.x / 1000.0;
  start[1] = req.start_robot_pos.y / 1000.0;
  start[2] = deg2rad(req.start_robot_pos.th);
  goal[0] = req.goal_robot_pos.x / 1000.0;
  goal[1] = req.goal_robot_pos.y / 1000.0;
  goal[2] = deg2rad(req.goal_robot_pos.th);

  double collision_threshold = getRobotCollisionDist(req.robot_id);
  res.success = setCollisionArea(sub_Map, collision_threshold, res.message);
  if (!res.success)
  {
    ROS_ERROR((res.message).c_str());
    return false;
  }
  res.success = setVoronoiLine(sub_Map, res.message);
  if (!res.success)
  {
    ROS_ERROR((res.message).c_str());
    return false;
  }
  res.success = calcDistFromVoronoi(sub_Map, res.message);
  if (!res.success)
  {
    ROS_ERROR((res.message).c_str());
    return false;
  }
  res.success = connectToVoronoi(sub_Map, start, res.message);
  if (!res.success)
  {
    ROS_ERROR("Error : Start Point");
    ROS_ERROR((res.message).c_str());
    return false;
  }
  res.success = calcDistFromVoronoi(sub_Map, res.message);
  if (!res.success)
  {
    ROS_ERROR((res.message).c_str());
    return false;
  }
  res.success = connectToVoronoi(sub_Map, goal, res.message);
  if (!res.success)
  {
    ROS_ERROR("Error : Goal Point");
    ROS_ERROR((res.message).c_str());
    return false;
  }
  res.success = calcDistFromVoronoi(sub_Map, res.message);
  if (!res.success)
  {
    ROS_ERROR((res.message).c_str());
    return false;
  }
  res.success = calcDistFromGoal(sub_Map, goal, res.message);
  if (!res.success)
  {
    ROS_ERROR((res.message).c_str());
    return false;
  }
  res.success = calcVoronoiPath(sub_Map, start, goal, voronoi_path, res.message);
  if (!res.success)
  {
    ROS_ERROR((res.message).c_str());
    return false;
  }
  smoothVoronoiPath(sub_Map, start, goal, voronoi_path, smooth_path, Smooth_Voronoi_Path_Threshold / 1000.0);
  compVoronoiPath(smooth_path, comp_path);
  ////calc robot path end

  vector< double > rob_pos, temp_push_wagon_param;
  vector< vector< double > > push_wagon_path, simple_push_wagon_path;
  temp_push_wagon_param.clear();
  push_wagon_path.clear();
  rob_pos.resize(3);
  rob_pos[0] = comp_path[0][0];
  rob_pos[1] = comp_path[0][1];
  rob_pos[2] = comp_path[0][2];
  temp_push_wagon_param.push_back(rob_pos[0]);
  temp_push_wagon_param.push_back(rob_pos[1]);
  temp_push_wagon_param.push_back(rob_pos[2]);
  temp_push_wagon_param.push_back(rel_th);
  push_wagon_path.push_back(temp_push_wagon_param);
  for (unsigned int i = 1; i < comp_path.size(); i++)
  {
    temp_push_wagon_param.clear();
    if ((rob_pos[0] == comp_path[i][0]) && (rob_pos[1] == comp_path[i][1]))
      continue;
    rob_pos[0] = comp_path[i][0];
    rob_pos[1] = comp_path[i][1];
    calcWagonPosOnVoronoi(sub_Map, rob_pos, push_wagon_type, Robot_Control_Wagon_Dist / 1000.0, rel_th,
                          change_wagon_rel_th, Push_Wagon_Path_Threshold / 1000.0);

    //~ cout<<rob_pos[0]<<"	"<<rob_pos[1]<<"	"<<rad2deg(rob_pos[2])<<"	"<<rad2deg(rel_th)<<endl;
    temp_push_wagon_param.push_back(rob_pos[0]);
    temp_push_wagon_param.push_back(rob_pos[1]);
    temp_push_wagon_param.push_back(rob_pos[2]);
    temp_push_wagon_param.push_back(rel_th);
    push_wagon_path.push_back(temp_push_wagon_param);
  }

  cut_unnecessary_path_point(push_wagon_path, smooth_path, cut_th_threshold, simple_push_wagon_path);

  double wagon_l = 0.0, wagon_th_offset;
  switch (push_wagon_type)
  {
    case 1:
      wagon_l = Wagon_Size_LongSide_Length;
      wagon_th_offset = M_PI;
      break;

    case 2:
      wagon_l = Wagon_Size_ShortSide_Length;
      wagon_th_offset = M_PI / 2.0;
      break;

    case 3:
      wagon_l = Wagon_Size_LongSide_Length;
      wagon_th_offset = 0.0;
      break;

    case 4:
      wagon_l = Wagon_Size_ShortSide_Length;
      wagon_th_offset = -M_PI / 2.0;
      break;
  }

  tms_msg_rp::rps_position temp_pos;
  tms_msg_rp::rps_route rps_robot_path, rps_wagon_path;
  rps_robot_path.rps_route.clear();
  rps_wagon_path.rps_route.clear();

  tms_msg_rp::rps_cnoid_grasp_wagon_planning srv_calc_grasp_wagon_pose;

  for (unsigned int i = 0; i < simple_push_wagon_path.size(); i++)
  {
    temp_pos.x = simple_push_wagon_path[i][0] * 1000.0;
    temp_pos.y = simple_push_wagon_path[i][1] * 1000.0;
    temp_pos.th = rad2deg(simple_push_wagon_path[i][2]);
    res.RobotPath.push_back(temp_pos);
    srv_calc_grasp_wagon_pose.request.robot_pos = temp_pos;

    temp_pos.th = simple_push_wagon_path[i][2];
    rps_robot_path.rps_route.push_back(temp_pos);

    res.control_wagon_th.push_back(rad2deg(simple_push_wagon_path[i][3]));

    temp_pos.x =
        ((simple_push_wagon_path[i][0] * 1000.0) + Robot_Control_Wagon_Dist * cos(simple_push_wagon_path[i][2])) +
        (wagon_l / 2.0) * cos(simple_push_wagon_path[i][2] + simple_push_wagon_path[i][3]);
    temp_pos.y =
        ((simple_push_wagon_path[i][1] * 1000.0) + Robot_Control_Wagon_Dist * sin(simple_push_wagon_path[i][2])) +
        (wagon_l / 2.0) * sin(simple_push_wagon_path[i][2] + simple_push_wagon_path[i][3]);
    temp_pos.th = rad2deg(simple_push_wagon_path[i][2] + simple_push_wagon_path[i][3] + wagon_th_offset);
    res.WagonPath.push_back(temp_pos);
    srv_calc_grasp_wagon_pose.request.wagon_pos = temp_pos;

    temp_pos.th = simple_push_wagon_path[i][2] + simple_push_wagon_path[i][3] + wagon_th_offset;
    rps_wagon_path.rps_route.push_back(temp_pos);

    //~ if(commander_to_calc_grasp_wagon_pose.call(srv_calc_grasp_wagon_pose)){
    //~ res.joint_angle_array.push_back(srv_calc_grasp_wagon_pose.response.robot_joint_angle[0]);
    //~ }
  }

  int k = 0;
  while (k < 100)
  {
    rps_robot_path_pub.publish(rps_robot_path);
    rps_wagon_path_pub.publish(rps_wagon_path);
    k++;
  }
  ROS_INFO("path publish");
  //~ int a;
  //~ scanf("%d", &a);

  ROS_INFO("...Push Wagon Path Plan Success");
  res.message = "Push Wagon Path Plan Success";
  res.success = 1;
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rps_push_wagon_path_planner");

  ros::NodeHandle n;

  ros::Subscriber rps_map_sub = n.subscribe("rps_map_data", 1, set_RPS_MAP);
  ros::ServiceServer service_push_wagon_path =
      n.advertiseService("rps_push_wagon_path_planning", start_push_wagon_path_planner);

  commander_to_get_robots_info = n.serviceClient< tms_msg_db::tmsdb_get_robots_info >("tmsdb_get_robots_info");
  commander_to_get_movable_furnitures_info =
      n.serviceClient< tms_msg_db::tmsdb_get_movable_furnitures_info >("tmsdb_get_movable_furnitures_info");

  commander_to_calc_grasp_wagon_pose =
      n.serviceClient< tms_msg_rp::rps_cnoid_grasp_wagon_planning >("rps_cnoid_calc_grasp_wagon_pose");

  rps_robot_path_pub = n.advertise< tms_msg_rp::rps_route >("rps_robot_path", 1);
  rps_wagon_path_pub = n.advertise< tms_msg_rp::rps_route >("rps_wagon_path", 1);

  ros::Rate loop_rate(1);

  ros::spin();

  loop_rate.sleep();

  return 0;
}
