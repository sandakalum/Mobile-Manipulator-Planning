#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../../rps.h"

#include <tms_msg_rp/rps_position.h>
#include <tms_msg_rp/rps_route.h>
#include <tms_msg_rp/rps_map_data.h>
#include <tms_msg_rp/rps_map_y.h>
#include <tms_msg_rp/rps_map_full.h>
#include <tms_msg_rp/rps_path_checking.h>
#include <tms_msg_rp/rps_voronoi_path_planning.h>
#include <tms_msg_rp/rps_path_alarm.h>
#include <tms_msg_db/tmsdb_get_robots_info.h>

#include <sstream>

using namespace std;

ros::Publisher rps_robot_path_pub;
ros::Publisher rps_path_pub;
ros::Publisher rps_path_alarm_pub;
ros::ServiceClient client_voronoi_path;
ros::ServiceClient commander_to_get_robots_info;

tms_msg_rp::rps_voronoi_path_planning service_voronoi_path;
tms_msg_rp::rps_route rps_robot_path;

int robot_id;
double goal_pos[3] = {0.0};
bool set_path_flg = false;

enum PATH_ALARM path_alarm;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void path_comp(vector< vector< double > > in_path, vector< vector< double > >& out_path)
{
  double incre_th = deg2rad(1.0);
  double d_norm;

  out_path.clear();
  std::vector< double > temp_pos, next_pos, d_vector;
  temp_pos.resize(3);
  next_pos.resize(3);
  d_vector.resize(3);

  for (unsigned int i = 0; i < in_path.size() - 1; i++)
  {
    temp_pos[0] = in_path[i][0];
    temp_pos[1] = in_path[i][1];
    temp_pos[2] = in_path[i][2];
    out_path.push_back(temp_pos);

    next_pos[0] = in_path[i + 1][0];
    next_pos[1] = in_path[i + 1][1];
    next_pos[2] = in_path[i + 1][2];

    d_vector[0] = next_pos[0] - temp_pos[0];
    d_vector[1] = next_pos[1] - temp_pos[1];
    d_vector[2] = next_pos[2] - temp_pos[2];
    d_norm = sqrt((d_vector[0] * d_vector[0]) + (d_vector[1] * d_vector[1]));
    if (d_norm != 0)
    {
      d_vector[0] /= d_norm;
      d_vector[1] /= d_norm;
    }

    //~ if(d_vector[2]>PI){
    //~ d_vector[2] -= 2*PI;
    //~ next_pos[2] -= 2*PI;
    //~ }
    //~ if(d_vector[2]<-PI){
    //~ d_vector[2] += 2*PI;
    //~ next_pos[2] += 2*PI;
    //~ }
    //~
    //~ while(1){
    //~ if(d_vector[2]>=0){
    //~ temp_pos[2] += incre_th;
    //~ if(temp_pos[2]>next_pos[2])	break;
    //~ }
    //~ else{
    //~ temp_pos[2] -= incre_th;
    //~ if(temp_pos[2]<next_pos[2])	break;
    //~ }
    //~ out_path.push_back(temp_pos);
    //~ }
    //~ temp_pos[2]=next_pos[2];
    //~ out_path.push_back(temp_pos);

    for (unsigned int k = 1; k < (d_norm / (10 * cell_resolution)); k++)
    {
      temp_pos[0] += d_vector[0] * (10 * cell_resolution);
      temp_pos[1] += d_vector[1] * (10 * cell_resolution);
      out_path.push_back(temp_pos);
    }
    out_path.push_back(next_pos);
  }
  cout << "finish path comp" << endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool set_robot_path(tms_msg_rp::rps_path_checking::Request& req, tms_msg_rp::rps_path_checking::Response& res)
{
  cout << "set robot path..." << endl;
  vector< vector< double > > smoothing_path, complement_path;
  vector< double > temp_pos;
  smoothing_path.clear();
  complement_path.clear();
  temp_pos.resize(3);
  rps_robot_path.rps_route.clear();

  for (unsigned int x = 0; x < MAP.size(); x++)
  {
    for (unsigned int y = 0; y < MAP[x].size(); y++)
    {
      MAP[x][y].route = 0;
    }
  }

  robot_id = req.robot_id;

  if (req.rps_robot_path.rps_route.size() == 0)
  {
    set_path_flg = true;
    path_alarm = NOT_PATH;
    ROS_ERROR("Path is not found");
  }
  else
  {
    set_path_flg = true;
    path_alarm = SAFE;
    for (unsigned int i = 0; i < req.rps_robot_path.rps_route.size(); i++)
    {
      temp_pos[0] = req.rps_robot_path.rps_route[i].x;
      temp_pos[1] = req.rps_robot_path.rps_route[i].y;
      temp_pos[2] = deg2rad(req.rps_robot_path.rps_route[i].th);
      smoothing_path.push_back(temp_pos);
    }

    goal_pos[0] = temp_pos[0];
    goal_pos[1] = temp_pos[1];
    goal_pos[2] = req.rps_robot_path.rps_route[req.rps_robot_path.rps_route.size() - 1].th;

    path_comp(smoothing_path, complement_path);

    for (unsigned int i = 0; i < complement_path.size(); i++)
    {
      if (!MAP[(int)round(complement_path[i][0] / (10 * cell_resolution))][(int)round(complement_path[i][1] /
                                                                                      (10 * cell_resolution))]
               .robot_collision)
        MAP[(int)round(complement_path[i][0] / (10 * cell_resolution))][(int)round(complement_path[i][1] /
                                                                                   (10 * cell_resolution))].route = 1;
    }

    tms_msg_rp::rps_position pos;

    for (unsigned int j = 0; j < req.rps_robot_path.rps_route.size(); j++)
    {
      pos.x = req.rps_robot_path.rps_route[j].x;
      pos.y = req.rps_robot_path.rps_route[j].y;
      pos.th = deg2rad(req.rps_robot_path.rps_route[j].th);
      rps_robot_path.rps_route.push_back(pos);
    }
  }

  int k = 0;
  while (k < 100)
  {
    rps_robot_path_pub.publish(rps_robot_path);
    k++;
  }
  ROS_INFO("path publish");

  res.success = 0;
  res.message = "set robot_path finish";
  cout << "set robot_path finish" << endl;

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void path_check(const tms_msg_rp::rps_map_full::ConstPtr& RPS_MAP)
{
  cout << "map checking..." << endl;
  tms_msg_rp::rps_route plan_path;
  plan_path.rps_route.clear();
  tms_msg_rp::rps_path_alarm msg_path_alarm;
  msg_path_alarm.rps_route.clear();

  vector< double > start, end;
  start.resize(3);
  end.resize(3);

  double d_time = ros::Time::now().toSec();

  if (path_alarm != NOT_PATH)
  {
    path_alarm = SAFE;
    tms_msg_db::tmsdb_get_robots_info srv_r;
    srv_r.request.robots_id = robot_id;
    if (commander_to_get_robots_info.call(srv_r))
    {
      ROS_INFO("Success robots_x = %lf, y = %lf, theta = %lf", srv_r.response.robots_x, srv_r.response.robots_y,
               srv_r.response.robots_theta);
      start[0] = srv_r.response.robots_x / (10.0 * cell_resolution);
      start[1] = srv_r.response.robots_y / (10.0 * cell_resolution);
      start[2] = deg2rad(srv_r.response.robots_theta);
    }
    else
    {
      ROS_ERROR("Failed to call service get_robots_info\n");
      return;
    }

    end[0] = (int)round(goal_pos[0] / (10 * cell_resolution));
    end[1] = (int)round(goal_pos[1] / (10 * cell_resolution));

    for (unsigned int x = 0; x < RPS_MAP->rps_map_x.size(); x++)
    {
      for (unsigned int y = 0; y < RPS_MAP->rps_map_x[x].rps_map_y.size(); y++)
      {
        MAP[x][y].object = RPS_MAP->rps_map_x[x].rps_map_y[y].object;
        // MAP[x][y].object_detect_time	=	RPS_MAP->rps_map_x[x].rps_map_y[y].object_detect_time;
        MAP[x][y].robot_collision = RPS_MAP->rps_map_x[x].rps_map_y[y].robot_collision;
        MAP[x][y].wagon_collision = RPS_MAP->rps_map_x[x].rps_map_y[y].wagon_collision;
        MAP[x][y].voronoi = RPS_MAP->rps_map_x[x].rps_map_y[y].voronoi;
        MAP[x][y].dist_from_obj_i = RPS_MAP->rps_map_x[x].rps_map_y[y].dist_from_obj_i;

        if (MAP[x][y].robot_collision && MAP[x][y].route)
        {
          //~ if((getLength(start[0],start[1],x,y)*cell_resolution)<=LENGTH_LIMIT){
          //~ cout<<getLength(start[0],start[1],x,y)*cell_resolution<<endl;
          //~ cout<<"LENGTH_LIMIT!"<<endl;
          //~ path_alarm = PATH_BLOCK;
          //~ break;
          //~ }
          if (MAP[x][y].object_detect_time == 0.0)
          {
            MAP[x][y].object_detect_time = d_time;
            MAP[x][y].total_object_detect_time = 0.0;
          }
          //~ else{
          MAP[x][y].total_object_detect_time = d_time - MAP[x][y].object_detect_time;
          // cout<<MAP[x][y].total_object_detect_time<<endl;
          if (MAP[x][y].total_object_detect_time >= PATH_CHANGE_TIME)
          {
            cout << "TIME_LIMIT!	at x:" << x << "	y;" << y << endl;
            path_alarm = PATH_BLOCK;
            break;
          }
          //~ }
        }
        if (!MAP[x][y].robot_collision)
        {
          MAP[x][y].object_detect_time = 0.0;
          MAP[x][y].total_object_detect_time = 0.0;
        }
      }
      if (MAP[end[0]][end[1]].robot_collision)
      {
        //~ if(MAP[end[0]][end[1]].object_detect_time==0.0){
        //~ MAP[end[0]][end[1]].object_detect_time = d_time;
        //~ MAP[end[0]][end[1]].total_object_detect_time = 0.0;
        //~ }
        //~ else{
        //~ MAP[end[0]][end[1]].total_object_detect_time = d_time - MAP[end[0]][end[1]].object_detect_time;
        //~ cout<<MAP[end[0]][end[1]].total_object_detect_time<<endl;
        //~ if(MAP[end[0]][end[1]].total_object_detect_time >= PATH_CHANGE_TIME){
        //~ cout<<"TIME_LIMIT!"<<endl;
        path_alarm = GOAL_BLOCK;
        //~ }
        //~ }
      }
    }
  }

  if (set_path_flg)
  {
    service_voronoi_path.request.robot_id = robot_id;
    service_voronoi_path.request.goal_pos.x = goal_pos[0];
    service_voronoi_path.request.goal_pos.y = goal_pos[1];
    service_voronoi_path.request.goal_pos.th = goal_pos[2];

    if (client_voronoi_path.call(service_voronoi_path))
    {
      if (service_voronoi_path.response.voronoi_path.rps_route.size() != 0)
      {
        plan_path = service_voronoi_path.response.voronoi_path;
        msg_path_alarm.rps_route = plan_path.rps_route;
      }
    }
    else
    {
      ROS_ERROR("voronoi planner can't called");
      return;
    }

    cout << "path_alarm:	";
    switch (path_alarm)
    {
      case SAFE:
        // cout<<"SAFE"<<endl;
        msg_path_alarm.path_alarm = SAFE;
        break;
      case NOT_PATH:
        cout << "NOT_PATH" << endl;
        msg_path_alarm.path_alarm = NOT_PATH;
        break;
      case PATH_BLOCK:
        cout << "PATH_BLOCK" << endl;
        msg_path_alarm.path_alarm = PATH_BLOCK;
        break;
      case GOAL_BLOCK:
        cout << "GOAL_BLOCK" << endl;
        msg_path_alarm.path_alarm = GOAL_BLOCK;
        break;
      case CAUTION:
        // cout<<"CAUTION"<<endl;
        msg_path_alarm.path_alarm = CAUTION;
        break;
    }

    int k = 0;
    while (k < 100)
    {
      rps_path_pub.publish(plan_path);
      rps_path_alarm_pub.publish(msg_path_alarm);
      k++;
    }
    // ROS_INFO("plan path publish");
  }
  else
  {
  }
}

void MAP_time_init(vector< vector< map_data > >& Map)
{
  for (unsigned int x = 0; x < Map.size(); x++)
  {  //初期化
    for (unsigned int y = 0; y < Map[x].size(); y++)
    {
      Map[x][y].object_detect_time = 0.0;
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rps_path_checker");

  ros::NodeHandle n;

  MAP.resize(AreaMax_x + 1);
  for (unsigned int x = 0; x < AreaMax_x + 1; x++)
    MAP[x].resize(AreaMax_y + 1);

  MAP_time_init(MAP);

  path_alarm = NOT_PATH;

  ros::ServiceServer service_path_checking = n.advertiseService("rps_path_checking", set_robot_path);
  client_voronoi_path = n.serviceClient< tms_msg_rp::rps_voronoi_path_planning >("rps_voronoi_path_planning");
  commander_to_get_robots_info = n.serviceClient< tms_msg_db::tmsdb_get_robots_info >("tmsdb_get_robots_info");

  ros::Subscriber rps_map_subscriber = n.subscribe("rps_map_data", 1, path_check);
  rps_robot_path_pub = n.advertise< tms_msg_rp::rps_route >("rps_robot_path", 1);
  rps_path_pub = n.advertise< tms_msg_rp::rps_route >("rps_plan_path", 1);
  rps_path_alarm_pub = n.advertise< tms_msg_rp::rps_path_alarm >("rps_path_alarm", 1);

  ros::spin();

  return 0;
}
