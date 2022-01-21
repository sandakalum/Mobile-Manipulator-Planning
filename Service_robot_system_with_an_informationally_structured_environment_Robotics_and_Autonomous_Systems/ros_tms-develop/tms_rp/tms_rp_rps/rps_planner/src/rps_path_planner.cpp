#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../../rps.h"
#include <tms_msg_rp/rps_map_full.h>
#include <tms_msg_rp/rps_voronoi_path_planning.h>
#include <tms_msg_rp/rps_path_planning.h>
#include <tms_msg_db/tmsdb_get_robots_info.h>
#include <tms_msg_db/TmsdbGetData.h>

#include <sstream>

using namespace std;

ros::ServiceClient commander_to_get_robots_info;
ros::ServiceClient client_voronoi_path;
ros::ServiceClient get_data_client;

tms_msg_db::tmsdb_get_robots_info srv_r;
tms_msg_rp::rps_voronoi_path_planning service_voronoi_path;

bool start_rps_path_planner(tms_msg_rp::rps_path_planning::Request& req, tms_msg_rp::rps_path_planning::Response& res)
{
  cout << "Start Path Planning..." << endl;
  cout << "Select Robot_id is " << req.robot_id << endl;

  if (req.rps_goal_candidate.rps_route.size() == 0)
  {
    ROS_ERROR("Goal Pos is not set!");
    res.success = 0;
    res.message = "Goal Pos is not set!";
    return true;
  }

  tms_msg_db::TmsdbGetData getRobotData;

  getRobotData.request.tmsdb.id = 2002;
  getRobotData.request.tmsdb.sensor = 3005;

  if (get_data_client.call(getRobotData))
  {
    ROS_INFO("Get info of object ID: %d\n", getRobotData.request.tmsdb.id);
  }
  else
  {
    ROS_ERROR("Failed to call service get_robot_info\n");
    return false;
  }

  if (getRobotData.response.tmsdb.empty() == true)
  {
    ROS_ERROR("Failed to call service get_robot_info\n");
    return false;
  }

  int32_t robot_id = getRobotData.response.tmsdb[0].id;
  double robots_x = getRobotData.response.tmsdb[0].x;
  double robots_y = getRobotData.response.tmsdb[0].y;
  double robots_theta = getRobotData.response.tmsdb[0].ry;

  res.rps_path.clear();
  tms_msg_rp::rps_position temp_pos;
  tms_msg_rp::rps_route temp_route;
  temp_route.rps_route.clear();

  ROS_INFO("Robot x: %f\n", robots_x);
  ROS_INFO("Robot y: %f\n", robots_y);
  ROS_INFO("Robot t: %f\n", robots_theta);

  service_voronoi_path.request.robot_id = robot_id;
  service_voronoi_path.request.start_pos.x = robots_x;
  service_voronoi_path.request.start_pos.y = robots_y;
  service_voronoi_path.request.start_pos.th = robots_theta;

  for (unsigned int i = 0; i < req.rps_goal_candidate.rps_route.size(); i++)
  {
    service_voronoi_path.request.goal_pos.x = req.rps_goal_candidate.rps_route[i].x;
    service_voronoi_path.request.goal_pos.y = req.rps_goal_candidate.rps_route[i].y;
    service_voronoi_path.request.goal_pos.th = req.rps_goal_candidate.rps_route[i].th;

    cout << "Goal[" << i << "]"
         << " x:" << service_voronoi_path.request.goal_pos.x << "	y:" << service_voronoi_path.request.goal_pos.y
         << "	th:" << service_voronoi_path.request.goal_pos.th << endl;

    if (client_voronoi_path.call(service_voronoi_path))
    {
      ROS_INFO("Success Path [%d] : %ld", i, (long int)service_voronoi_path.response.success);
      res.success = service_voronoi_path.response.success;
      res.message = service_voronoi_path.response.message;

      if (service_voronoi_path.response.voronoi_path.rps_route.size() != 0)
      {
        for (unsigned int j = 0; j < service_voronoi_path.response.voronoi_path.rps_route.size(); j++)
        {
          temp_pos.x = service_voronoi_path.response.voronoi_path.rps_route[j].x;
          temp_pos.y = service_voronoi_path.response.voronoi_path.rps_route[j].y;
          temp_pos.th = service_voronoi_path.response.voronoi_path.rps_route[j].th;

          temp_route.rps_route.push_back(temp_pos);
        }
        res.rps_path.push_back(temp_route);

        for (unsigned int i = 0; i < res.rps_path.size(); i++)
        {
          for (unsigned int j = 0; j < res.rps_path[i].rps_route.size(); j++)
          {
            cout << "Path" << i << "-" << j << "	x: " << res.rps_path[i].rps_route[j].x
                 << "	y: " << res.rps_path[i].rps_route[j].y << "	th: " << res.rps_path[i].rps_route[j].th << endl;
          }
        }
      }
      else
      {
        ROS_ERROR("Path [%d] is not found : ", i);
        cout << "	" << service_voronoi_path.response.message << endl;
      }
    }
    else
    {
      ROS_ERROR("Failed to call voronoi_path_service");
      res.success = 0;
      res.message = "Failed to call voronoi_path_service";
      return false;
    }
    temp_route.rps_route.clear();
    res.success = 1;
    res.message = "Success path planning";
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rps_path_planner");
  ros::NodeHandle n;

  ros::ServiceServer service_p = n.advertiseService("rps_path_planning", start_rps_path_planner);

  get_data_client = n.serviceClient< tms_msg_db::TmsdbGetData >("/tms_db_reader/dbreader");
  commander_to_get_robots_info = n.serviceClient< tms_msg_db::tmsdb_get_robots_info >("tmsdb_get_robots_info");
  client_voronoi_path = n.serviceClient< tms_msg_rp::rps_voronoi_path_planning >("rps_voronoi_path_planning");

  ros::spin();

  return 0;
}
