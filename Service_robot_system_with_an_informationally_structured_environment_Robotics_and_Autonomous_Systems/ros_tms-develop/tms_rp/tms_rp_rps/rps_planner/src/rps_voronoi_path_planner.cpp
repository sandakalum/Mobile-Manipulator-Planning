#include "ros/ros.h"
#include "std_msgs/String.h"
//~ #include "../../rps.h"
#include "rps_voronoi.h"
#include <tms_msg_rp/rps_map_full.h>
#include <tms_msg_rp/rps_voronoi_path_planning.h>
#include <tms_msg_db/tmsdb_get_robots_info.h>
#include <tms_msg_db/tmsdb_get_movable_furnitures_info.h>

#include <sstream>

//#define USE_TMS_DB
//#define USE_WAGON

using namespace std;

ros::Publisher rps_robot_path_pub;
ros::ServiceClient commander_to_get_robots_info;
ros::ServiceClient commander_to_get_movable_furnitures_info;

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

bool start_voronoi_path_planner(tms_msg_rp::rps_voronoi_path_planning::Request& req,
                                tms_msg_rp::rps_voronoi_path_planning::Response& res)
{
  res.success = 0;
  res.VoronoiPath.clear();
  res.voronoi_path.rps_route.clear();
  ROS_INFO("Start Voronoi Path Plan...");
  //~ clock_t t_start,t_end;
  //~ t_start = clock();

  double collision_threshold = getRobotCollisionDist(req.robot_id);

  vector< double > start, goal;
  start.resize(3);
  goal.resize(3);
  start[0] = req.start_pos.x / 1000.0;   //(m)
  start[1] = req.start_pos.y / 1000.0;   //(m)
  start[2] = deg2rad(req.start_pos.th);  //(rad)
  goal[0] = req.goal_pos.x / 1000.0;     //(m)
  goal[1] = req.goal_pos.y / 1000.0;     //(m)
  goal[2] = deg2rad(req.goal_pos.th);    //(rad)

  tms_msg_db::tmsdb_get_movable_furnitures_info srv_get_f_info;
  tms_msg_rp::rps_position wagon_pos;
  srv_get_f_info.request.furnitures_id = 22;

  ROS_INFO("Init map ...");

#ifdef USE_TMS_DB
  if (commander_to_get_movable_furnitures_info.call(srv_get_f_info))
  {
    //~ ROS_INFO("Success target_x = %lf, y = %lf, th = %lf, width = %lf, depth = %lf, height = %lf",
    // srv_get_f_info.response.furniture_x,srv_get_f_info.response.furniture_y,srv_get_f_info.response.furnitures_theta,srv_get_f_info.response.furnitures_width,srv_get_f_info.response.furnitures_depth,srv_get_f_info.response.furnitures_height);
  }
  else
  {
    ROS_ERROR("Failed to call service get_furnitures_info");
    res.message = "Failed to call service get_furnitures_info";
    return false;
  }
  if (srv_get_f_info.response.furnitures_state == 1)
  {  // if wagon exist(stop) : state = 1
    wagon_pos.x = srv_get_f_info.response.furniture_x;
    wagon_pos.y = srv_get_f_info.response.furniture_y;
    wagon_pos.th = deg2rad(srv_get_f_info.response.furnitures_theta);

    Wagon_Size_LongSide_Length = srv_get_f_info.response.furnitures_width;
    Wagon_Size_ShortSide_Length = srv_get_f_info.response.furnitures_depth;
    Wagon_Size_Height = srv_get_f_info.response.furnitures_height;

    set_wagon_as_obstacle(sub_Map, wagon_pos);
    if (!calcDistFromObj(sub_Map, res.message))
      return false;
  }
#else
#ifdef USE_TMS_DB
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
#endif

  ROS_INFO("Calculate voronoi path...");

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

  vector< vector< double > > voronoi_path, smooth_path, comp_path;
  res.success = calcVoronoiPath(sub_Map, start, goal, voronoi_path, res.message);
  if (!res.success)
  {
    ROS_ERROR((res.message).c_str());
    return false;
  }
  smoothVoronoiPath(sub_Map, start, goal, voronoi_path, smooth_path, Smooth_Voronoi_Path_Threshold / 1000.0);
  //~ compVoronoiPath(smooth_path, comp_path);

  //~ t_end = clock();
  //~ printf("calc_time:%.2f(sec)\n",(double)(t_end-t_start)/CLOCKS_PER_SEC);

  tms_msg_rp::rps_position temp_pos;
  tms_msg_rp::rps_route rps_robot_path;
  rps_robot_path.robot_id = req.robot_id;
  rps_robot_path.rps_route.clear();

  for (unsigned int i = 0; i < smooth_path.size(); i++)
  {
    temp_pos.x = smooth_path[i][0] * 1000.0;
    temp_pos.y = smooth_path[i][1] * 1000.0;
    temp_pos.th = rad2deg(smooth_path[i][2]);

    res.VoronoiPath.push_back(temp_pos);
    res.voronoi_path.rps_route.push_back(temp_pos);

    temp_pos.th = smooth_path[i][2];
    rps_robot_path.rps_route.push_back(temp_pos);
  }

  ROS_INFO("...Voronoi Path Plan Success");

  int k = 0;
  while (k < 100)
  {
    rps_robot_path_pub.publish(rps_robot_path);
    k++;
  }
  ROS_INFO("path publish");

  //~ int a;
  //~ scanf("%d", &a);

  res.message = "Voronoi Path Plan Success";
  res.success = 1;
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rps_voronoi_path_planner");

  ros::NodeHandle n;

  ros::Subscriber rps_map_sub = n.subscribe("rps_map_data", 1, set_RPS_MAP);
  ros::ServiceServer service_voronoi_path = n.advertiseService("rps_voronoi_path_planning", start_voronoi_path_planner);
  commander_to_get_robots_info = n.serviceClient< tms_msg_db::tmsdb_get_robots_info >("tmsdb_get_robots_info");
  commander_to_get_movable_furnitures_info =
      n.serviceClient< tms_msg_db::tmsdb_get_movable_furnitures_info >("tmsdb_get_movable_furnitures_info");
  rps_robot_path_pub = n.advertise< tms_msg_rp::rps_route >("rps_robot_path", 1);

  ros::Rate loop_rate(1);

  ros::spin();

  loop_rate.sleep();

  return 0;
}
