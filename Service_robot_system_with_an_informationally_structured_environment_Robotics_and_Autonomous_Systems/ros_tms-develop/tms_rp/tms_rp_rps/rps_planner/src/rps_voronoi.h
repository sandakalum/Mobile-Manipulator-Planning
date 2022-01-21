#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../../rps.h"

#include <sstream>

using namespace std;

bool calcDistFromVoronoi(vector< vector< CollisionMapData > >& Map, string& message)
{
  if (Map.empty())
  {
    message = "Error : Map is empty";
    cout << message << endl;
    return false;
  }

  for (int x = 0; x < Map.size(); x++)
  {
    for (int y = 0; y < Map[x].size(); y++)
    {
      if (Map[x][y].voronoi && (!Map[x][y].collision))
        Map[x][y].dist_from_voronoi = 0.0;
      else
        Map[x][y].dist_from_voronoi = (x_ulimit - x_llimit) * (y_ulimit - y_llimit);
    }
  }

  for (int x = 1; x < Map.size() - 1; x++)
  {
    for (int y = 1; y < Map[x].size() - 1; y++)
    {
      if (!Map[x][y].collision)
      {
        Map[x][y].dist_from_voronoi = min(min(min(Map[x - 1][y - 1].dist_from_voronoi + sqrt(2.0) * cell_size,
                                                  Map[x][y - 1].dist_from_voronoi + 1.0 * cell_size),
                                              min(Map[x - 1][y + 1].dist_from_voronoi + sqrt(2.0) * cell_size,
                                                  Map[x - 1][y].dist_from_voronoi + 1.0 * cell_size)),
                                          Map[x][y].dist_from_voronoi);
      }
    }
  }
  for (int x = Map.size() - 2; x > 0; x--)
  {
    for (int y = Map[x].size() - 2; y > 0; y--)
    {
      if (!Map[x][y].collision)
      {
        Map[x][y].dist_from_voronoi = min(min(min(Map[x + 1][y].dist_from_voronoi + 1.0 * cell_size,
                                                  Map[x + 1][y - 1].dist_from_voronoi + sqrt(2.0) * cell_size),
                                              min(Map[x][y + 1].dist_from_voronoi + 1.0 * cell_size,
                                                  Map[x + 1][y + 1].dist_from_voronoi + sqrt(2.0) * cell_size)),
                                          Map[x][y].dist_from_voronoi);
      }
    }
  }

  return true;
}

bool calcDistFromGoal(vector< vector< CollisionMapData > >& Map, vector< double > goal_point, string& message)
{
  if (Map.empty())
  {
    message = "Error : Map is empty";
    cout << message << endl;
    return false;
  }

  if ((goal_point[0] < x_llimit) || (goal_point[0] > x_ulimit) || (goal_point[1] < y_llimit) ||
      (goal_point[1] > y_ulimit))
  {
    message = "Error : Goal is out of range";
    cout << message << endl;
    return false;
  }

  int temp_x, temp_y;
  temp_x = (int)round((goal_point[0] - x_llimit) / cell_size);
  temp_y = (int)round((goal_point[1] - y_llimit) / cell_size);

  for (int x = 0; x < Map.size(); x++)
  {
    for (int y = 0; y < Map[x].size(); y++)
    {
      Map[x][y].dist_from_goal = (x_ulimit - x_llimit) * (y_ulimit - y_llimit);
    }
  }
  Map[temp_x][temp_y].dist_from_goal = 0.0;

  bool change_flg = true;
  double temp = 0.0;

  while (change_flg)
  {
    change_flg = false;
    for (unsigned int i = 1; i < Map.size() - 1; i++)
    {  //距離値計算 ラスタ走査
      for (unsigned int j = 1; j < Map[i].size() - 1; j++)
      {
        if (Map[i][j].voronoi)
        {
          temp = Map[i][j].dist_from_goal;
          Map[i][j].dist_from_goal = min(min(min(Map[i - 1][j - 1].dist_from_goal + sqrt(2.0) * cell_size,
                                                 Map[i - 1][j].dist_from_goal + 1.0 * cell_size),
                                             Map[i - 1][j + 1].dist_from_goal + sqrt(2.0) * cell_size),
                                         min(Map[i][j - 1].dist_from_goal + 1.0 * cell_size, Map[i][j].dist_from_goal));
          if (change_flg == false)
          {
            if (Map[i][j].dist_from_goal != temp)
              change_flg = true;
          }
        }
      }
    }
    for (int i = Map.size() - 2; i > 0; i--)
    {  //距離値計算 逆ラスタ走査
      for (int j = Map[i].size() - 2; j > 0; j--)
      {
        if (Map[i][j].voronoi)
        {
          temp = Map[i][j].dist_from_goal;
          Map[i][j].dist_from_goal = min(min(Map[i][j].dist_from_goal, Map[i][j + 1].dist_from_goal + 1.0 * cell_size),
                                         min(Map[i + 1][j - 1].dist_from_goal + sqrt(2.0) * cell_size,
                                             min(Map[i + 1][j].dist_from_goal + 1.0 * cell_size,
                                                 Map[i + 1][j + 1].dist_from_goal + sqrt(2.0) * cell_size)));
          if (change_flg == false)
          {
            if (Map[i][j].dist_from_goal != temp)
              change_flg = true;
          }
        }
      }
    }
  }

  return true;
}

bool connectToVoronoi(vector< vector< CollisionMapData > >& Map, vector< double > connect_point, string& message)
{
  if (Map.empty())
  {
    message = "Error : Map is empty";
    cout << message << endl;
    return false;
  }

  if ((connect_point[0] < x_llimit) || (connect_point[0] > x_ulimit) || (connect_point[1] < y_llimit) ||
      (connect_point[1] > y_ulimit))
  {
    message = "Error : Connect point is out of range";
    cout << message << endl;
    cout << "	x:" << connect_point[0] << "	y:" << connect_point[1] << "	limit_x:" << x_llimit << " ~ " << x_ulimit
         << "	limit_y:" << y_llimit << " ~ " << y_ulimit << endl;
    return false;
  }

  int target_x, target_y, temp_x, temp_y, dx = 0, dy = 0;
  target_x = (int)round((connect_point[0] - x_llimit) / cell_size);
  target_y = (int)round((connect_point[1] - y_llimit) / cell_size);
  temp_x = target_x;
  temp_y = target_y;

  double temp_dist = Map[temp_x][temp_y].dist_from_voronoi;
  if (Map[temp_x][temp_y].collision)
  {
    message = "Error : Connect point is collision";
    cout << message << endl;
    cout << "	x:" << temp_x << "	y:" << temp_y << endl;
    return false;
  }
  if (temp_dist == 0.0)
    return true;

  while (temp_dist != 0.0)
  {
    if (Map[temp_x - 1][temp_y].dist_from_voronoi < temp_dist)
    {
      dx = -1;
      dy = 0;
      temp_dist = Map[temp_x + dx][temp_y].dist_from_voronoi;
    }
    if (Map[temp_x][temp_y - 1].dist_from_voronoi < temp_dist)
    {
      dx = 0;
      dy = -1;
      temp_dist = Map[temp_x][temp_y + dy].dist_from_voronoi;
    }
    if (Map[temp_x + 1][temp_y].dist_from_voronoi < temp_dist)
    {
      dx = 1;
      dy = 0;
      temp_dist = Map[temp_x + dx][temp_y].dist_from_voronoi;
    }
    if (Map[temp_x][temp_y + 1].dist_from_voronoi < temp_dist)
    {
      dx = 0;
      dy = 1;
      temp_dist = Map[temp_x][temp_y + dy].dist_from_voronoi;
    }
    if (Map[temp_x - 1][temp_y - 1].dist_from_voronoi < temp_dist)
    {
      dx = -1;
      dy = -1;
      temp_dist = Map[temp_x + dx][temp_y + dy].dist_from_voronoi;
    }
    if (Map[temp_x - 1][temp_y + 1].dist_from_voronoi < temp_dist)
    {
      dx = -1;
      dy = 1;
      temp_dist = Map[temp_x + dx][temp_y + dy].dist_from_voronoi;
    }
    if (Map[temp_x + 1][temp_y - 1].dist_from_voronoi < temp_dist)
    {
      dx = 1;
      dy = -1;
      temp_dist = Map[temp_x + dx][temp_y + dy].dist_from_voronoi;
    }
    if (Map[temp_x + 1][temp_y + 1].dist_from_voronoi < temp_dist)
    {
      dx = 1;
      dy = 1;
      temp_dist = Map[temp_x + dx][temp_y + dy].dist_from_voronoi;
    }
    temp_x += dx;
    temp_y += dy;
  }

  vector< double > vec;
  vec.resize(2);
  vec[0] = temp_x - target_x;
  vec[1] = temp_y - target_y;
  double vec_norm = sqrt((vec[0] * vec[0]) + (vec[1] * vec[1]));
  vec[0] /= vec_norm;
  vec[1] /= vec_norm;

  double temp_norm = 1.0;
  while (temp_norm < vec_norm)
  {
    temp_x = (int)round(target_x + temp_norm * vec[0]);
    temp_y = (int)round(target_y + temp_norm * vec[1]);
    Map[temp_x][temp_y].voronoi = true;
    temp_norm += 1.0;
  }
  temp_x = (int)round(target_x + vec_norm * vec[0]);
  temp_y = (int)round(target_y + vec_norm * vec[1]);
  Map[temp_x][temp_y].voronoi = true;

  Map[target_x][target_y].voronoi = true;

  return true;
}

bool calcVoronoiPath(vector< vector< CollisionMapData > >& Map, vector< double > start, vector< double > goal,
                     vector< vector< double > >& out_path, string& message)
{
  out_path.clear();

  if (Map.empty())
  {
    message = "Error : Map is empty";
    cout << message << endl;
  }

  if ((start[0] < x_llimit) || (start[0] > x_ulimit) || (start[1] < y_llimit) || (start[1] > y_ulimit))
  {
    message = "Error : Start point is out of range";
    cout << message << endl;
    cout << "	x:" << start[0] << "	y:" << start[1] << "	limit_x:" << x_llimit << " ~ " << x_ulimit
         << "	limit_y:" << y_llimit << " ~ " << y_ulimit << endl;
    return false;
  }
  if ((goal[0] < x_llimit) || (goal[0] > x_ulimit) || (goal[1] < y_llimit) || (goal[1] > y_ulimit))
  {
    message = "Error : Goal point is out of range";
    cout << "	x:" << goal[0] << "	y:" << goal[1] << "	limit_x:" << x_llimit << " ~ " << x_ulimit
         << "	limit_y:" << y_llimit << " ~ " << y_ulimit << endl;
    cout << message << endl;
    return false;
  }

  int i_start_x, i_start_y, i_goal_x, i_goal_y, i_temp_x, i_temp_y, dx = 0, dy = 0;
  i_start_x = (int)round((start[0] - x_llimit) / cell_size);
  i_start_y = (int)round((start[1] - y_llimit) / cell_size);
  i_goal_x = (int)round((goal[0] - x_llimit) / cell_size);
  i_goal_y = (int)round((goal[1] - y_llimit) / cell_size);

  if (Map[i_start_x][i_start_y].collision)
  {
    message = "Error : Start is collision";
    cout << message << endl;
    cout << "	x:" << i_start_x << "	y:" << i_start_y << endl;
    return false;
  }
  if (Map[i_goal_x][i_goal_y].collision)
  {
    message = "Error : Goal is collision";
    cout << message << endl;
    cout << "	x:" << i_goal_x << "	y:" << i_goal_y << endl;
    return false;
  }
  Map[i_start_x][i_start_y].path = true;
  Map[i_goal_x][i_goal_y].path = true;

  //~ out_path.push_back(start);

  i_temp_x = i_start_x;
  i_temp_y = i_start_y;
  double temp_dist = Map[i_temp_x][i_temp_y].dist_from_goal;
  vector< double > temp_pos;
  temp_pos.resize(2);

  if (temp_dist == 0)
    return true;

  while (1)
  {
    if (temp_dist > Map[i_temp_x - 1][i_temp_y].dist_from_goal)
    {
      dx = -1;
      dy = 0;
      temp_dist = Map[i_temp_x + dx][i_temp_y + dy].dist_from_goal;
    }
    if (temp_dist > Map[i_temp_x][i_temp_y - 1].dist_from_goal)
    {
      dx = 0;
      dy = -1;
      temp_dist = Map[i_temp_x + dx][i_temp_y + dy].dist_from_goal;
    }
    if (temp_dist > Map[i_temp_x + 1][i_temp_y].dist_from_goal)
    {
      dx = 1;
      dy = 0;
      temp_dist = Map[i_temp_x + dx][i_temp_y + dy].dist_from_goal;
    }
    if (temp_dist > Map[i_temp_x][i_temp_y + 1].dist_from_goal)
    {
      dx = 0;
      dy = 1;
      temp_dist = Map[i_temp_x + dx][i_temp_y + dy].dist_from_goal;
    }
    if (temp_dist > Map[i_temp_x - 1][i_temp_y - 1].dist_from_goal)
    {
      dx = -1;
      dy = -1;
      temp_dist = Map[i_temp_x + dx][i_temp_y + dy].dist_from_goal;
    }
    if (temp_dist > Map[i_temp_x - 1][i_temp_y + 1].dist_from_goal)
    {
      dx = -1;
      dy = 1;
      temp_dist = Map[i_temp_x + dx][i_temp_y + dy].dist_from_goal;
    }
    if (temp_dist > Map[i_temp_x + 1][i_temp_y - 1].dist_from_goal)
    {
      dx = 1;
      dy = -1;
      temp_dist = Map[i_temp_x + dx][i_temp_y + dy].dist_from_goal;
    }
    if (temp_dist > Map[i_temp_x + 1][i_temp_y + 1].dist_from_goal)
    {
      dx = 1;
      dy = 1;
      temp_dist = Map[i_temp_x + dx][i_temp_y + dy].dist_from_goal;
    }
    i_temp_x += dx;
    i_temp_y += dy;
    Map[i_temp_x][i_temp_y].path = true;

    if (temp_dist == 0.0)
      break;

    temp_pos[0] = (i_temp_x * cell_size) + x_llimit;
    temp_pos[1] = (i_temp_y * cell_size) + y_llimit;
    out_path.push_back(temp_pos);
  }

  //~ out_path.push_back(goal);

  //~ for(int i=0;i<out_path.size();i++){
  //~ cout<<out_path[i][0]<<"	"<<out_path[i][1]<<endl;
  //~ }
  //~ cout<<endl;
  return true;
}

bool smoothVoronoiPath(vector< vector< CollisionMapData > >& Map, vector< double > start, vector< double > goal,
                       vector< vector< double > > in_path, vector< vector< double > >& out_path, double threshold)
{
  out_path.clear();
  vector< double > temp_pos;
  temp_pos.resize(3);

  if (in_path.size() < 2)
  {
    out_path.push_back(start);
    out_path.push_back(goal);
    return true;
  }

  out_path.push_back(start);
  temp_pos = start;

  int i_temp_x, i_temp_y, i_prev_x, i_prev_y, i_next_x, i_next_y;
  double temp_th = 0.0;
  i_prev_x = (int)round((start[0] - x_llimit) / cell_size);
  i_prev_y = (int)round((start[1] - y_llimit) / cell_size);

  vector< double > vec;
  vec.resize(2);
  double vec_norm, temp_norm;
  bool smoothing = true;

  for (unsigned int i = 1; i < in_path.size(); i++)
  {
    smoothing = true;
    i_next_x = (int)round((in_path[i][0] - x_llimit) / cell_size);
    i_next_y = (int)round((in_path[i][1] - y_llimit) / cell_size);

    vec[0] = i_next_x - i_prev_x;
    vec[1] = i_next_y - i_prev_y;
    vec_norm = sqrt((vec[0] * vec[0]) + (vec[1] * vec[1]));
    vec[0] /= vec_norm;
    vec[1] /= vec_norm;

    temp_norm = 1.0;
    while (temp_norm < vec_norm)
    {
      i_temp_x = (int)round(i_prev_x + temp_norm * vec[0]);
      i_temp_y = (int)round(i_prev_y + temp_norm * vec[1]);
      if (Map[i_temp_x][i_temp_y].dist_from_voronoi > threshold)
      {
        smoothing = false;
        break;
      }
      temp_norm += 1.0;
    }

    if (!smoothing)
    {
      temp_pos[2] = atan2(i_next_y - i_prev_y, i_next_x - i_prev_x);
      out_path.push_back(temp_pos);
      temp_pos[0] = (i_next_x * cell_size) + x_llimit;
      temp_pos[1] = (i_next_y * cell_size) + y_llimit;
      out_path.push_back(temp_pos);
      i_prev_x = i_next_x;
      i_prev_y = i_next_y;
    }
  }
  temp_pos[2] = atan2(goal[1] - ((i_prev_y * cell_size) + y_llimit), goal[0] - ((i_prev_x * cell_size) + x_llimit));
  out_path.push_back(temp_pos);
  temp_pos[0] = goal[0];
  temp_pos[1] = goal[1];
  out_path.push_back(temp_pos);

  out_path.push_back(goal);
  //~ for(int i=0;i<out_path.size();i++){
  //~ cout<<out_path[i][0]<<"	"<<out_path[i][1]<<"	"<<rad2deg(out_path[i][2])<<endl;
  //~ }

  return true;
}

bool compVoronoiPath(vector< vector< double > > in_path, vector< vector< double > >& out_path)
{
  out_path.clear();

  if (in_path.size() < 2)
  {
    cout << "Error : path is empty" << endl;
    return false;
  }

  double incre_th = deg2rad(1.0);
  vector< double > vec, temp_pos, next_pos;
  vec.resize(3);
  temp_pos.resize(3);
  next_pos.resize(3);
  double vec_norm, temp_norm;

  for (unsigned int i = 0; i < in_path.size() - 1; i++)
  {
    temp_pos[0] = in_path[i][0];
    temp_pos[1] = in_path[i][1];
    temp_pos[2] = in_path[i][2];
    out_path.push_back(temp_pos);

    next_pos[0] = in_path[i + 1][0];
    next_pos[1] = in_path[i + 1][1];
    next_pos[2] = in_path[i + 1][2];

    vec[0] = next_pos[0] - temp_pos[0];
    vec[1] = next_pos[1] - temp_pos[1];
    vec[2] = next_pos[2] - temp_pos[2];
    vec_norm = sqrt((vec[0] * vec[0]) + (vec[1] * vec[1]));
    if (vec_norm != 0.0)
    {
      vec[0] /= vec_norm;
      vec[1] /= vec_norm;
    }
    if (vec[2] > M_PI)
    {
      vec[2] -= 2 * M_PI;
      next_pos[2] -= 2 * M_PI;
    }
    if (vec[2] < -M_PI)
    {
      vec[2] += 2 * M_PI;
      next_pos[2] += 2 * M_PI;
    }

    while (1)
    {
      if (vec[2] >= 0)
      {
        temp_pos[2] += incre_th;
        if (temp_pos[2] > next_pos[2])
          break;
      }
      else
      {
        temp_pos[2] -= incre_th;
        if (temp_pos[2] < next_pos[2])
          break;
      }
      out_path.push_back(temp_pos);
    }
    temp_pos[2] = next_pos[2];
    out_path.push_back(temp_pos);

    temp_norm = 0.01;
    while (temp_norm < vec_norm)
    {
      temp_pos[0] = in_path[i][0] + temp_norm * vec[0];
      temp_pos[1] = in_path[i][1] + temp_norm * vec[1];
      temp_norm += 0.01;
      out_path.push_back(temp_pos);
    }
    out_path.push_back(next_pos);
  }

  temp_pos[0] = next_pos[0];
  temp_pos[1] = next_pos[1];
  temp_pos[2] = next_pos[2];

  next_pos[0] = in_path[in_path.size() - 1][0];
  next_pos[1] = in_path[in_path.size() - 1][1];
  next_pos[2] = in_path[in_path.size() - 1][2];

  vec[0] = next_pos[0] - temp_pos[0];
  vec[1] = next_pos[1] - temp_pos[1];
  vec[2] = next_pos[2] - temp_pos[2];
  vec_norm = sqrt((vec[0] * vec[0]) + (vec[1] * vec[1]));
  if (vec_norm != 0.0)
  {
    vec[0] /= vec_norm;
    vec[1] /= vec_norm;
  }
  if (vec[2] > M_PI)
  {
    vec[2] -= 2 * M_PI;
    next_pos[2] -= 2 * M_PI;
  }
  if (vec[2] < -M_PI)
  {
    vec[2] += 2 * M_PI;
    next_pos[2] += 2 * M_PI;
  }

  while (1)
  {
    if (vec[2] >= 0)
    {
      temp_pos[2] += incre_th;
      if (temp_pos[2] > next_pos[2])
        break;
    }
    else
    {
      temp_pos[2] -= incre_th;
      if (temp_pos[2] < next_pos[2])
        break;
    }
    out_path.push_back(temp_pos);
  }
  temp_pos[2] = next_pos[2];
  out_path.push_back(temp_pos);

  //~ for(int i=0;i<out_path.size();i++){
  //~ cout<<out_path[i][0]<<"	"<<out_path[i][1]<<"	"<<rad2deg(out_path[i][2])<<endl;
  //~ }

  return true;
}
