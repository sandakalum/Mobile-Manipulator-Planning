#include <time.h>
#include <tms_msg_rp/rps_position.h>
#include <tms_msg_rp/rps_joint_angle.h>
#include <tms_msg_rp/rps_map_full.h>

using namespace std;

//~ #define DEBUG_MODE

#define PI 3.14159265359
#define deg2rad(x) ((x)*PI / 180.0)
#define rad2deg(x) ((x)*180.0 / PI)

double getLength(double x1, double y1, double x2, double y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

const double sp5_arm_init_pose[18] = {0.0, 0.0,                                                /*waist*/
                                      0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                /*right arm*/
                                      0.0, 10.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0 /*left arm*/};  // degree
//~ const double sp5_arm_give_pre_pose[18] = {	0.0, 0.0,	/*waist*/
//~ 20.0, -15.0, -65.0, 90.0, 40.0, 0.0, 0.0, 0.0, /*right arm*/
//~ 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 /*left arm*/};	//degree
const double sp5_arm_give_pre_pose[18] = {0.0, 0.0,                                                       /*waist*/
                                          0.0, -10.0, -40.0, 100.0, 25.0, 0.0, -40.0, 0.0,                /*right arm*/
                                          0.0, 10.0,  0.0,   0.0,   0.0,  0.0, 0.0,   0.0 /*left arm*/};  // degree

const int jointNum = sizeof sp5_arm_init_pose / sizeof sp5_arm_init_pose[0];

const double SmartPal5_Collision_Threshold = 400.0;  // mm
const double SmartPal4_Collision_Threshold = 400.0;  // mm
const double Kobuki_Collision_Threshold = 200.0;     // mm
const double KKP_Collision_Threshold = 400.0;        // mm

const double Robot_Control_Wagon_Dist = 400.0;  // mm
//~ const double Robot_Control_Wagon_Dist		=	500.0;	//mm

const double Smooth_Voronoi_Path_Threshold = 100.0;  // mm
const double Push_Wagon_Path_Threshold = 200.0;      // mm
const bool change_wagon_rel_th = false;
const double wagon_control_limit_th = deg2rad(30.0);
const double cut_th_threshold = deg2rad(5.0);

const int person_height = 160;  // cm
const double person_view_limit = deg2rad(60.0);

const int give_obj_pos_divide_num = 4;

const char useRobotHand = 'R';
const char useHumanHand = 'R';
const bool useRobotWaist = true;
const bool useHumanWaist = false;

double Wagon_Size_LongSide_Length, Wagon_Size_ShortSide_Length, Wagon_Size_Height;  // mm

double x_llimit, x_ulimit, y_llimit, y_ulimit, cell_size;  // m

class CollisionMapData
{
public:
  bool object;
  double dist_from_obj;

  bool collision;

  bool voronoi;
  int thinning_flg;
  double dist_from_voronoi;

  bool path;
  double dist_from_path;

  double dist_from_goal;

  bool person_view;

  double table_height;
};

class ManipulabilityMapData
{
public:
  bool reachable;

  double robot_manipulability;
  double human_manipulability;
  double peak_manipulability;
  double total_manipulability;

  vector< double > obj_pos;
  vector< double > robot_palm_pos_rel;

  double dist_from_obj_to_robot;
  double th_from_obj_to_robot;

  vector< double > human_joint_angle;
  vector< double > robot_joint_angle;
};

class GivePosData
{
public:
  vector< double > robotPos;
  vector< double > joint_angle;

  bool view;
  double obj_dist;    // m
  double human_dist;  // m
  double evaluation;

  void set_Evaluation(vector< vector< CollisionMapData > > Map, vector< double > human_pos)
  {
    view = Map[(int)round((robotPos[0] - x_llimit) / cell_size)][(int)round((robotPos[1] - y_llimit) / cell_size)]
               .person_view;
    obj_dist = Map[(int)round((robotPos[0] - x_llimit) / cell_size)][(int)round((robotPos[1] - y_llimit) / cell_size)]
                   .dist_from_obj;
    human_dist = getLength(robotPos[0], robotPos[1], human_pos[0], human_pos[1]);

    evaluation = view + obj_dist + human_dist;
  };
};

class PushWagonPathData
{
public:
  int robot_id;
  int wagon_id;
  int person_id;
  int object_id;

  tms_msg_rp::rps_position start_Robot_pos, start_Wagon_pos, give_obj_Obj_pos;
  vector< tms_msg_rp::rps_position > grasp_wagon_Robot_pos, give_obj_Robot_pos;
  vector< tms_msg_rp::rps_joint_angle > give_obj_Robot_joint_angle;

  vector< vector< tms_msg_rp::rps_position > > grasp_wagon_Robot_path;
  vector< vector< vector< tms_msg_rp::rps_position > > > push_wagon_Robot_path, push_wagon_Wagon_path;
};

class ExecutePushWagonPath
{
public:
  int robot_id;
  int wagon_id;
  int person_id;
  int object_id;

  tms_msg_rp::rps_position start_Robot_pos, start_Wagon_pos, give_obj_Obj_pos, grasp_wagon_Robot_pos,
      give_obj_Robot_pos;
  vector< tms_msg_rp::rps_position > grasp_wagon_Robot_path, push_wagon_Robot_path, push_wagon_Wagon_path;
  vector< tms_msg_rp::rps_joint_angle > grasp_wagon_Robot_joint_angle, push_wagon_Robot_joint_angle,
      release_wagon_Robot_joint_angle, grasp_obj_Robot_joint_angle, give_obj_Robot_joint_angle;
};

// Function///////////////////////////////////////
bool convertPos(double in_x, double in_y, int& out_x, int& out_y)
{  // in_pos:(m)
  if ((in_x < x_llimit) || (in_x > x_ulimit) || (in_y < y_llimit) || (in_y > y_ulimit))
  {
    cout << "in_Pos is out of Map" << endl;
    return false;
  }

  out_x = (int)round((in_x - x_llimit) / cell_size);
  out_y = (int)round((in_y - y_llimit) / cell_size);

  return true;
}

double getRobotCollisionDist(int robot_id)
{
  double collision_threshold = 0.0;

  switch (robot_id)
  {
    case 2001:  // SmartPal4
      collision_threshold = SmartPal4_Collision_Threshold / 1000.0;
      break;
    case 2002:  // SmartPal5_1
      collision_threshold = SmartPal5_Collision_Threshold / 1000.0;
      break;
    case 2003:  // SmartPal5_2
      collision_threshold = SmartPal5_Collision_Threshold / 1000.0;
      break;
    case 2004:  // turtlebot2
      collision_threshold = Kobuki_Collision_Threshold / 1000.0;
      break;
    case 2005:  // kobuki
      collision_threshold = Kobuki_Collision_Threshold / 1000.0;
      break;
    case 4:
      collision_threshold = 0.2;
      break;
    case 2006:  // KKP
      collision_threshold = KKP_Collision_Threshold / 1000.0;
      break;
    default:
      collision_threshold = 0.1;
      break;
  }

  return collision_threshold;
}

bool calcDistFromObj(vector< vector< CollisionMapData > >& Map, string& message)
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
      if (Map[x][y].object)
        Map[x][y].dist_from_obj = 0.0;
      else
        Map[x][y].dist_from_obj = (x_ulimit - x_llimit) * (y_ulimit - y_llimit);
    }
  }

  for (int x = 1; x < Map.size() - 1; x++)
  {
    for (int y = 1; y < Map[x].size() - 1; y++)
    {
      if (!Map[x][y].object)
      {
        Map[x][y].dist_from_obj = min(min(min(Map[x - 1][y - 1].dist_from_obj + sqrt(2.0) * cell_size,
                                              Map[x][y - 1].dist_from_obj + 1.0 * cell_size),
                                          min(Map[x - 1][y + 1].dist_from_obj + sqrt(2.0) * cell_size,
                                              Map[x - 1][y].dist_from_obj + 1.0 * cell_size)),
                                      Map[x][y].dist_from_obj);
      }
    }
  }
  for (int x = Map.size() - 2; x > 0; x--)
  {
    for (int y = Map[x].size() - 2; y > 0; y--)
    {
      if (!Map[x][y].object)
      {
        Map[x][y].dist_from_obj = min(min(min(Map[x + 1][y].dist_from_obj + 1.0 * cell_size,
                                              Map[x + 1][y - 1].dist_from_obj + sqrt(2.0) * cell_size),
                                          min(Map[x][y + 1].dist_from_obj + 1.0 * cell_size,
                                              Map[x + 1][y + 1].dist_from_obj + sqrt(2.0) * cell_size)),
                                      Map[x][y].dist_from_obj);
      }
    }
  }

  return true;
}

void calcWagonPolePos(tms_msg_rp::rps_position wagon_pos, vector< tms_msg_rp::rps_position >& out_pos)
{
  out_pos.clear();

  tms_msg_rp::rps_position tempPos;
  vector< tms_msg_rp::rps_position > polePos;
  polePos.clear();

  tempPos.x = Wagon_Size_LongSide_Length / 2.0;
  tempPos.y = Wagon_Size_ShortSide_Length / 2.0;
  polePos.push_back(tempPos);

  tempPos.x *= -1;
  polePos.push_back(tempPos);

  tempPos.y *= -1;
  polePos.push_back(tempPos);

  tempPos.x *= -1;
  polePos.push_back(tempPos);

  for (unsigned int i = 0; i < polePos.size(); i++)
  {
    tempPos.x = polePos[i].x * cos(wagon_pos.th) - polePos[i].y * sin(wagon_pos.th);
    tempPos.y = polePos[i].x * sin(wagon_pos.th) + polePos[i].y * cos(wagon_pos.th);
    tempPos.x += wagon_pos.x;
    tempPos.y += wagon_pos.y;
    out_pos.push_back(tempPos);
    //~ cout<<polePos[i][0]<<"	"<<polePos[i][1]<<endl;
  }

  polePos.clear();
}

void set_wagon_as_obstacle(vector< vector< CollisionMapData > >& Map, tms_msg_rp::rps_position wagon_pos)
{
  vector< tms_msg_rp::rps_position > polePos;
  calcWagonPolePos(wagon_pos, polePos);

  int temp_x, temp_y;
  temp_x = temp_y = 0;
  for (unsigned int i = 0; i < polePos.size(); i++)
  {
    temp_x = (int)round(((polePos[i].x / 1000.0) - x_llimit) / cell_size);
    temp_y = (int)round(((polePos[i].y / 1000.0) - y_llimit) / cell_size);

    Map[temp_x][temp_y].object = true;
    Map[temp_x][temp_y].collision = true;
  }
}

bool setCollisionArea(vector< vector< CollisionMapData > >& Map, double threshold, string& message)
{
  if (Map.empty())
  {
    message = "Error : Map is empty";
    cout << message << endl;
    return false;
  }

  for (unsigned int x = 0; x < Map.size(); x++)
  {
    for (unsigned int y = 0; y < Map[x].size(); y++)
    {
      if (Map[x][y].dist_from_obj - threshold < 1.0e-5)
        Map[x][y].collision = true;
      else
        Map[x][y].collision = false;
    }
  }

  return true;
}

bool setVoronoiLine(vector< vector< CollisionMapData > >& Map, string& message)
{
  if (Map.empty())
  {
    message = "Error : Map is empty";
    cout << message << endl;
    return false;
  }

  for (unsigned int x = 0; x < Map.size(); x++)
  {
    for (unsigned int y = 0; y < Map[x].size(); y++)
    {
      Map[x][y].voronoi = false;
      if (!Map[x][y].collision)
        Map[x][y].thinning_flg = 1;
      else
        Map[x][y].thinning_flg = 0;
    }
  }

  bool flg = true;
  while (flg)
  {
    flg = false;
    for (int k = 0; k < 4; k++)
    {
      for (int x = 1; x < Map.size() - 1; x++)
      {
        for (int y = 1; y < Map[x].size() - 1; y++)
        {
          int i, j;
          switch (k)
          {
            case 0:
              i = x + 1;
              j = y;
              break;
            case 1:
              i = x;
              j = y - 1;
              break;
            case 2:
              i = x - 1;
              j = y;
              break;
            case 3:
              i = x;
              j = y + 1;
              break;
          }
          if ((Map[x][y].thinning_flg == 1) && (Map[i][j].thinning_flg == 0))
          {
            if (((Map[x][y - 1].thinning_flg == 0) && (Map[x + 1][y].thinning_flg == 1) &&
                 (Map[x][y + 1].thinning_flg == 1) && (Map[x + 1][y + 1].thinning_flg == 0)) ||
                ((Map[x - 1][y - 1].thinning_flg == 0) && (Map[x][y - 1].thinning_flg == 1) &&
                 (Map[x - 1][y].thinning_flg == 1) && (Map[x + 1][y].thinning_flg == 0)) ||
                ((Map[x - 1][y - 1].thinning_flg == 0) && (Map[x][y - 1].thinning_flg == 1) &&
                 (Map[x - 1][y].thinning_flg == 1) && (Map[x][y + 1].thinning_flg == 0)) ||
                ((Map[x - 1][y].thinning_flg == 0) && (Map[x + 1][y].thinning_flg == 1) &&
                 (Map[x][y + 1].thinning_flg == 1) && (Map[x + 1][y + 1].thinning_flg == 0)) ||
                ((Map[x - 1][y].thinning_flg == 0) && (Map[x + 1][y].thinning_flg == 0) &&
                 (Map[x][y + 1].thinning_flg == 1)) ||
                ((Map[x][y - 1].thinning_flg == 0) && (Map[x - 1][y].thinning_flg == 1) &&
                 (Map[x][y + 1].thinning_flg == 0)) ||
                ((Map[x][y - 1].thinning_flg == 1) && (Map[x - 1][y].thinning_flg == 0) &&
                 (Map[x + 1][y].thinning_flg == 0)) ||
                ((Map[x][y - 1].thinning_flg == 0) && (Map[x + 1][y].thinning_flg == 1) &&
                 (Map[x][y + 1].thinning_flg == 0)) ||
                ((Map[x - 1][y].thinning_flg == 0) && (Map[x - 1][y + 1].thinning_flg == 1) &&
                 (Map[x][y + 1].thinning_flg == 0)) ||
                ((Map[x - 1][y - 1].thinning_flg == 1) && (Map[x][y - 1].thinning_flg == 0) &&
                 (Map[x - 1][y].thinning_flg == 0)) ||
                ((Map[x][y - 1].thinning_flg == 0) && (Map[x + 1][y - 1].thinning_flg == 1) &&
                 (Map[x + 1][y].thinning_flg == 0)) ||
                ((Map[x + 1][y].thinning_flg == 0) && (Map[x][y + 1].thinning_flg == 0) &&
                 (Map[x + 1][y + 1].thinning_flg == 1)) ||
                ((Map[x - 1][y - 1].thinning_flg == 0) && (Map[x][y - 1].thinning_flg == 1) &&
                 (Map[x + 1][y - 1].thinning_flg == 0) && (Map[x - 1][y].thinning_flg == 1) &&
                 (Map[x + 1][y].thinning_flg == 1) && (Map[x - 1][y + 1].thinning_flg == 0) &&
                 (Map[x + 1][y + 1].thinning_flg == 0)) ||
                ((Map[x - 1][y - 1].thinning_flg == 0) && (Map[x][y - 1].thinning_flg == 1) &&
                 (Map[x + 1][y - 1].thinning_flg == 0) && (Map[x + 1][y].thinning_flg == 1) &&
                 (Map[x - 1][y + 1].thinning_flg == 0) && (Map[x][y + 1].thinning_flg == 1) &&
                 (Map[x + 1][y + 1].thinning_flg == 0)) ||
                ((Map[x - 1][y - 1].thinning_flg == 0) && (Map[x + 1][y - 1].thinning_flg == 0) &&
                 (Map[x - 1][y].thinning_flg == 1) && (Map[x + 1][y].thinning_flg == 1) &&
                 (Map[x - 1][y + 1].thinning_flg == 0) && (Map[x][y + 1].thinning_flg == 1) &&
                 (Map[x + 1][y + 1].thinning_flg == 0)) ||
                ((Map[x - 1][y - 1].thinning_flg == 0) && (Map[x][y - 1].thinning_flg == 1) &&
                 (Map[x + 1][y - 1].thinning_flg == 0) && (Map[x - 1][y].thinning_flg == 1) &&
                 (Map[x - 1][y + 1].thinning_flg == 0) && (Map[x][y + 1].thinning_flg == 1) &&
                 (Map[x + 1][y + 1].thinning_flg == 0)) ||
                ((Map[x - 1][y - 1].thinning_flg == 0) && (Map[x][y - 1].thinning_flg == 0) &&
                 (Map[x + 1][y - 1].thinning_flg == 0) && (Map[x - 1][y].thinning_flg == 0) &&
                 (Map[x - 1][y + 1].thinning_flg == 0)) ||
                ((Map[x - 1][y - 1].thinning_flg == 0) && (Map[x][y - 1].thinning_flg == 0) &&
                 (Map[x + 1][y - 1].thinning_flg == 0) && (Map[x + 1][y].thinning_flg == 0) &&
                 (Map[x + 1][y + 1].thinning_flg == 0)) ||
                ((Map[x - 1][y - 1].thinning_flg == 0) && (Map[x - 1][y].thinning_flg == 0) &&
                 (Map[x - 1][y + 1].thinning_flg == 0) && (Map[x][y + 1].thinning_flg == 0) &&
                 (Map[x + 1][y + 1].thinning_flg == 0)) ||
                ((Map[x + 1][y - 1].thinning_flg == 0) && (Map[x + 1][y].thinning_flg == 0) &&
                 (Map[x - 1][y + 1].thinning_flg == 0) && (Map[x][y + 1].thinning_flg == 0) &&
                 (Map[x + 1][y + 1].thinning_flg == 0)))
              Map[x][y].thinning_flg = 1;
            else
            {
              flg = true;
              Map[x][y].thinning_flg = 3;
            }
          }
        }
      }
      for (int x = 1; x < Map.size() - 1; x++)
      {
        for (int y = 1; y < Map[x].size() - 1; y++)
        {
          if (Map[x][y].thinning_flg == 3)
            Map[x][y].thinning_flg = 0;
        }
      }
    }
  }
  for (int x = 0; x < Map.size(); x++)
  {
    for (int y = 0; y < Map[x].size(); y++)
    {
      if (Map[x][y].thinning_flg == 1)
      {
        Map[x][y].voronoi = 1;
        Map[x][y].dist_from_voronoi = 0.0;
      }
      else
        Map[x][y].voronoi = 0;
    }
  }
  return true;
}

bool initCollisionMap(vector< vector< CollisionMapData > >& Map)
{
  FILE* fp;

  string file_name;
  char home_dir[255];
  strcpy(home_dir, getenv("HOME"));

  const char* fname;

  file_name = home_dir;
  file_name += "/catkin_ws/src/ros_tms/tms_rp/tms_rp_rps/rps_checker/MAP/use_collision_map.csv";
  fname = file_name.c_str();

  while (1)
  {
    if ((fp = fopen(fname, "r")) == NULL)
    {
      cout << "Error: collision map cannot open" << endl;
      return false;
    }
    else
      break;
  }

  Map.clear();
  vector< CollisionMapData > tempMapLine;
  CollisionMapData tempMapData;

  char* tp, buff[4096];

  if (fgets(buff, 4096, fp) == NULL)
    return false;

  tp = strtok(buff, ",");
  x_llimit = atof(tp);
  tp = strtok(NULL, ",");
  x_ulimit = atof(tp);
  tp = strtok(NULL, ",");
  y_llimit = atof(tp);
  tp = strtok(NULL, ",");
  y_ulimit = atof(tp);
  tp = strtok(NULL, ",");
  cell_size = atof(tp);

  int CommaCount = 0;

  while (fgets(buff, 4096, fp) != NULL)
  {  // collision
    CommaCount = 0;
    tp = strtok(buff, ",");
    tempMapData.object = atof(tp);
    if (tempMapData.object)
    {
      tempMapData.dist_from_obj = 0.0;
      tempMapData.collision = true;
    }
    else
    {
      tempMapData.dist_from_obj = (x_ulimit - x_llimit) * (y_ulimit - y_llimit);
      tempMapData.collision = false;
    }
    tempMapData.voronoi = false;
    tempMapData.path = false;
    tempMapData.thinning_flg = 0;
    tempMapData.dist_from_voronoi = (x_ulimit - x_llimit) * (y_ulimit - y_llimit);
    tempMapData.dist_from_goal = (x_ulimit - x_llimit) * (y_ulimit - y_llimit);
    tempMapData.dist_from_path = (x_ulimit - x_llimit) * (y_ulimit - y_llimit);
    tempMapLine.push_back(tempMapData);
    CommaCount++;
    while (tp != NULL)
    {
      if (CommaCount == int(round((y_ulimit - y_llimit) / cell_size)) + 1)
        break;
      tp = strtok(NULL, ",");
      CommaCount++;
      if (tp != NULL)
      {
        tempMapData.object = atof(tp);
        if (tempMapData.object)
        {
          tempMapData.dist_from_obj = 0.0;
          tempMapData.collision = true;
        }
        else
        {
          tempMapData.dist_from_obj = (x_ulimit - x_llimit) * (y_ulimit - y_llimit);
          tempMapData.collision = false;
        }
        tempMapData.voronoi = false;
        tempMapData.path = false;
        tempMapData.thinning_flg = 0;
        tempMapData.dist_from_voronoi = (x_ulimit - x_llimit) * (y_ulimit - y_llimit);
        tempMapData.dist_from_goal = (x_ulimit - x_llimit) * (y_ulimit - y_llimit);
        tempMapData.dist_from_path = (x_ulimit - x_llimit) * (y_ulimit - y_llimit);
        tempMapLine.push_back(tempMapData);
      }
    }

    Map.push_back(tempMapLine);
    tempMapLine.clear();
  }

  fclose(fp);

  return true;
}

void convertMap(vector< vector< CollisionMapData > > Map, tms_msg_rp::rps_map_full& rps_map)
{
  rps_map.rps_map_x.clear();

  rps_map.x_llimit = x_llimit;
  rps_map.x_ulimit = x_ulimit;
  rps_map.y_llimit = y_llimit;
  rps_map.y_ulimit = y_ulimit;
  rps_map.cell_size = cell_size;

  tms_msg_rp::rps_map_data temp_map_d;
  tms_msg_rp::rps_map_y temp_map_y;

  for (unsigned int x = 0; x < Map.size(); x++)
  {
    temp_map_y.rps_map_y.clear();
    for (unsigned int y = 0; y < Map[x].size(); y++)
    {
      temp_map_d.object = Map[x][y].object;
      temp_map_d.voronoi = Map[x][y].voronoi;
      temp_map_d.dist_from_obj_f = Map[x][y].dist_from_obj;

      temp_map_y.rps_map_y.push_back(temp_map_d);
    }
    rps_map.rps_map_x.push_back(temp_map_y);
  }
}
