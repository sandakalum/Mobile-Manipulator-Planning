// include for ROS
#include "ros/ros.h"
#include <unistd.h>
#include <boost/date_time/local_time/local_time.hpp>
#include <tms_msg_ss/ods_change_detection.h>
#include <tms_msg_ss/ods_furniture.h>
#include <tms_msg_ss/ods_get_robots_pos.h>
#include <tms_msg_rp/rps_path_planning.h>
#include <tms_msg_rc/smartpal_control.h>
#include <tms_msg_db/tmsdb_get_robots_info.h>
#include <tms_msg_rc/robot_tts.h>
#include <iostream>
#include <stdio.h>

#define MOVE_ROBOT 0
#define MOVE_LUMBA 0
#define MOVE_SOUND 0

ros::ServiceServer service;
ros::ServiceClient commander_to_table;
ros::ServiceClient commander_to_shelf;
ros::ServiceClient commander_to_chair;
ros::ServiceClient commander_to_bed;
ros::ServiceClient commander_to_get_robots_pos;
ros::ServiceClient commander_to_get_robots_info;
ros::ServiceClient commander_to_path_planner;
ros::ServiceClient commander_to_move_robot;
ros::ServiceClient commander_to_sp5_get_state;
ros::ServiceClient commander_to_sp5_sound;

bool change_detection(tms_msg_ss::ods_change_detection::Request &req, tms_msg_ss::ods_change_detection::Response &res)
{
  tms_msg_ss::ods_furniture srv;
  tms_msg_db::tmsdb_get_robots_info srv_r;
  tms_msg_rp::rps_path_planning srv2;
  tms_msg_ss::ods_get_robots_pos srv3;
  tms_msg_rc::smartpal_control srv4;
  tms_msg_rc::smartpal_control srv5;
  tms_msg_rc::robot_tts srv6;
  tms_msg_rp::rps_position rps_pos;

  std::cout << "change_detection" << std::endl;
  std::cout << "furniture_id : " << req.furniture_id << std::endl;

  switch (req.furniture_id)
  {
    // target furniture is TABLE
    case 14:
      std::cout << "change detection for table" << std::endl;
      std::cout << req.furniture << std::endl;
      std::cout << req.robot << std::endl;
      std::cout << req.sensor << std::endl;

      if (MOVE_SOUND == 1)
      {
        srv6.request.text = "search the object";
        if (commander_to_sp5_sound.call(srv6))
        {
          std::cout << "finish sound" << std::endl;
        }
        else
        {
          ROS_ERROR("Failed to call service smartpal5_tts\n");
          // return false;
        }
      }

      if (MOVE_ROBOT == 1)
      {
        srv_r.request.robots_id = 2;
        if (commander_to_get_robots_info.call(srv_r))
          ROS_INFO("Success robots_x = %lf, y = %lf, theta = %lf", srv_r.response.robots_x, srv_r.response.robots_y,
                   srv_r.response.robots_theta);
        else
        {
          ROS_ERROR("Failed to call service get_robots_info\n");
          return false;
        }
        srv4.request.unit = 1;
        srv4.request.cmd = 9;
        srv4.request.arg.resize(3);
        srv4.request.arg[0] = srv_r.response.robots_x;
        srv4.request.arg[1] = srv_r.response.robots_y;
        srv4.request.arg[2] = srv_r.response.robots_theta;
        if (commander_to_sp5_get_state.call(srv4))
        {
          ROS_INFO("result: %d", srv4.response.result);
        }
        else
        {
          ROS_ERROR("Failed to call service sp5_control");
          return false;
        }

        // std::cout << "move robot" << std::endl;

        /*rps_pos.x = 1400.0;
        rps_pos.y = 3200.0;
        rps_pos.th = -90.0;
        srv2.request.robot_id = 2;
        srv2.request.rps_goal_candidate.rps_route.push_back(rps_pos);

        if(commander_to_path_planner.call(srv2)){
            std::cout << "move robot" << std::endl;
        }
        else{
            ROS_ERROR("Failed to call service path planner");
            return false;
        }*/

        // for(int i=0;i < srv2.response.rps_path[0].rps_route.size();i++){

        srv4.request.unit = 1;
        srv4.request.cmd = 15;
        srv4.request.arg[0] = 1400.0;  // srv2.response.rps_path[0].rps_route[i].x;
        srv4.request.arg[1] = 3400.0;  // srv2.response.rps_path[0].rps_route[i].y;
        srv4.request.arg[2] = -90.0;   // srv2.response.rps_path[0].rps_route[i].th;

        if (commander_to_move_robot.call(srv4))
        {
          ROS_INFO("State: %d", srv4.response.result);
        }

        srv5.request.unit = 1;
        srv5.request.cmd = 7;

        while (1)
        {
          if (commander_to_sp5_get_state.call(srv5))
          {
            ROS_INFO("State: %d", srv5.response.result);
          }
          else
          {
            ROS_ERROR("Failed to call service sp5_control");
            return false;
          }
          if (srv5.response.result == 16)  // 18:Ready
            break;
        }
        //}
      }

      if (MOVE_LUMBA == 1)
      {
        srv4.request.unit = 6;
        srv4.request.cmd = 15;
        srv4.request.arg.resize(3);
        srv4.request.arg[0] = 25.0;  // srv2.response.rps_path[0].rps_route[i].x;
        srv4.request.arg[1] = 5.0;   // srv2.response.rps_path[0].rps_route[i].th;
        srv4.request.arg[2] = 3.0;

        if (commander_to_move_robot.call(srv4))
        {
          ROS_INFO("State: %d", srv4.response.result);
        }

        srv5.request.unit = 6;
        srv5.request.cmd = 7;

        while (1)
        {
          if (commander_to_sp5_get_state.call(srv5))
          {
            ROS_INFO("State: %d", srv5.response.result);
          }
          else
          {
            ROS_ERROR("Failed to call service sp5_control");
            return false;
          }
          if (srv5.response.result == 18)  // 18:Ready
            break;
        }
      }

      std::cout << "complete move" << std::endl;

      srv.request.id = 1;
      srv.request.cloud = req.cloud;
      srv.request.model = req.model;
      srv.request.furniture = req.furniture;
      srv.request.robot = req.robot;
      srv.request.sensor = req.sensor;

      std::cout << "complete move" << std::endl;
      if (commander_to_table.call(srv))
      {
        for (int i = 0; i < srv.response.objects.poses.size(); i++)
        {
          std::cout << srv.response.objects.poses[i].position.x << " " << srv.response.objects.poses[i].position.y
                    << " " << srv.response.objects.poses[i].position.z << std::endl;
        }
        res.objects = srv.response.objects;
        res.cloud = srv.response.cloud;

        if (MOVE_SOUND == 1)
        {
          srv6.request.text = "find the object";
          if (commander_to_sp5_sound.call(srv6))
          {
            std::cout << "finish sound" << std::endl;
          }
          else
          {
            ROS_ERROR("Failed to call service smartpal5_tts\n");
            // return false;
          }
        }

        if (MOVE_LUMBA == 1)
        {
          srv4.request.unit = 6;
          srv4.request.cmd = 15;
          srv4.request.arg.resize(3);
          srv4.request.arg[0] = 0.0;  // srv2.response.rps_path[0].rps_route[i].x;
          srv4.request.arg[1] = 5.0;  // srv2.response.rps_path[0].rps_route[i].y;
          srv4.request.arg[2] = 3.0;  // srv2.response.rps_path[0].rps_route[i].th;

          if (commander_to_move_robot.call(srv4))
          {
            ROS_INFO("State: %d", srv4.response.result);
          }

          srv5.request.unit = 6;
          srv5.request.cmd = 7;

          while (1)
          {
            if (commander_to_sp5_get_state.call(srv5))
            {
              ROS_INFO("State: %d", srv5.response.result);
            }
            else
            {
              ROS_ERROR("Failed to call service sp5_control");
              return false;
            }
            if (srv5.response.result == 18)  // 18:Ready
              break;
          }
        }

        if (MOVE_ROBOT == 1)
        {
          srv4.request.unit = 1;
          srv4.request.cmd = 15;
          srv4.request.arg[0] = 500.0;   // srv2.response.rps_path[0].rps_route[i].x;
          srv4.request.arg[1] = 3500.0;  // srv2.response.rps_path[0].rps_route[i].y;
          srv4.request.arg[2] = 0.0;     // srv2.response.rps_path[0].rps_route[i].th;

          if (commander_to_move_robot.call(srv4))
          {
            ROS_INFO("State: %d", srv4.response.result);
          }

          srv5.request.unit = 1;
          srv5.request.cmd = 7;

          while (1)
          {
            if (commander_to_sp5_get_state.call(srv5))
            {
              ROS_INFO("State: %d", srv5.response.result);
            }
            else
            {
              ROS_ERROR("Failed to call service sp5_control");
              return false;
            }
            if (srv5.response.result == 16)  // 18:Ready
              break;
          }
        }
      }

      break;

    // target furniture is TABLE 2
    case 15:
      std::cout << "change detection for table 2" << std::endl;
      std::cout << req.furniture << std::endl;
      std::cout << req.robot << std::endl;
      std::cout << req.sensor << std::endl;

      srv.request.id = 2;
      srv.request.cloud = req.cloud;
      srv.request.model = req.model;
      srv.request.furniture = req.furniture;
      srv.request.robot = req.robot;
      srv.request.sensor = req.sensor;

      if (commander_to_table.call(srv))
      {
        for (int i = 0; i < srv.response.objects.poses.size(); i++)
        {
          std::cout << srv.response.objects.poses[i].position.x << " " << srv.response.objects.poses[i].position.y
                    << " " << srv.response.objects.poses[i].position.z << std::endl;
        }
        res.objects = srv.response.objects;
        res.cloud = srv.response.cloud;
      }

      break;

    // target furniture is SMALL SHELF
    case 13:
      std::cout << "change detection for small shelf" << std::endl;
      std::cout << req.furniture << std::endl;
      std::cout << req.robot << std::endl;
      std::cout << req.sensor << std::endl;

      srv.request.id = 1;
      srv.request.cloud = req.cloud;
      srv.request.model = req.model;
      srv.request.furniture = req.furniture;
      srv.request.robot = req.robot;
      srv.request.sensor = req.sensor;

      if (commander_to_shelf.call(srv))
      {
        for (int i = 0; i < srv.response.objects.poses.size(); i++)
        {
          std::cout << srv.response.objects.poses[i].position.x << " " << srv.response.objects.poses[i].position.y
                    << " " << srv.response.objects.poses[i].position.z << std::endl;
        }
        res.objects = srv.response.objects;
        res.cloud = srv.response.cloud;
      }

      break;

    // target furniture is BIG SHELF
    case 12:
      std::cout << "change detection for big shelf" << std::endl;
      std::cout << req.furniture << std::endl;
      std::cout << req.robot << std::endl;
      std::cout << req.sensor << std::endl;

      srv.request.id = 2;
      srv.request.cloud = req.cloud;
      srv.request.model = req.model;
      srv.request.furniture = req.furniture;
      srv.request.robot = req.robot;
      srv.request.sensor = req.sensor;

      if (commander_to_shelf.call(srv))
      {
        for (int i = 0; i < srv.response.objects.poses.size(); i++)
        {
          std::cout << srv.response.objects.poses[i].position.x << " " << srv.response.objects.poses[i].position.y
                    << " " << srv.response.objects.poses[i].position.z << std::endl;
        }
        res.objects = srv.response.objects;
        res.cloud = srv.response.cloud;
      }

      break;

    // target furniture is CHAIR
    case 16:
      std::cout << "change detection for chair" << std::endl;
      std::cout << req.furniture << std::endl;
      std::cout << req.robot << std::endl;
      std::cout << req.sensor << std::endl;

      srv.request.id = 1;
      srv.request.cloud = req.cloud;
      srv.request.model = req.model;
      srv.request.furniture = req.furniture;
      srv.request.robot = req.robot;
      srv.request.sensor = req.sensor;

      if (commander_to_chair.call(srv))
      {
        for (int i = 0; i < srv.response.objects.poses.size(); i++)
        {
          std::cout << srv.response.objects.poses[i].position.x << " " << srv.response.objects.poses[i].position.y
                    << " " << srv.response.objects.poses[i].position.z << std::endl;
        }
      }
      res.objects = srv.response.objects;
      res.cloud = srv.response.cloud;

    // target furniture is BED
    case 17:
      std::cout << "change detection for bed" << std::endl;
      std::cout << req.furniture << std::endl;
      std::cout << req.robot << std::endl;
      std::cout << req.sensor << std::endl;

      srv.request.id = 1;
      srv.request.cloud = req.cloud;
      srv.request.model = req.model;
      srv.request.furniture = req.furniture;
      srv.request.robot = req.robot;
      srv.request.sensor = req.sensor;

      if (commander_to_bed.call(srv))
      {
        for (int i = 0; i < srv.response.objects.poses.size(); i++)
        {
          std::cout << srv.response.objects.poses[i].position.x << " " << srv.response.objects.poses[i].position.y
                    << " " << srv.response.objects.poses[i].position.z << std::endl;
        }
        res.objects = srv.response.objects;
        res.cloud = srv.response.cloud;
      }

      break;

    case 20:
      std::cout << "modify robot position" << std::endl;
      std::cout << req.furniture << std::endl;
      std::cout << req.robot << std::endl;
      std::cout << req.sensor << std::endl;

      srv3.request.model = req.model;
      srv3.request.furniture = req.furniture;
      srv3.request.robot = req.robot;
      srv3.request.sensor = req.sensor;

      if (commander_to_get_robots_pos.call(srv3))
      {
        std::cout << srv3.response.m_robot.x << " " << srv3.response.m_robot.y << " " << srv3.response.m_robot.theta
                  << std::endl;
      }

      break;

    default:
      std::cout << "ID is NOT furniture id" << std::endl;
      break;
  }

  res.tMeasuredTime = ros::Time::now() + ros::Duration(9 * 60 * 60);

  std::cout << "finish" << std::endl;

  return true;
}

int main(int argc, char **argv)
{
  std::cout << "init" << std::endl;
  ros::init(argc, argv, "ods_change_detection");
  ros::NodeHandle n;

  service = n.advertiseService("ods_change_detection", change_detection);
  commander_to_table = n.serviceClient< tms_msg_ss::ods_furniture >("chg_dt_table");
  commander_to_shelf = n.serviceClient< tms_msg_ss::ods_furniture >("chg_dt_shelf");
  commander_to_chair = n.serviceClient< tms_msg_ss::ods_furniture >("chg_dt_chair");
  commander_to_bed = n.serviceClient< tms_msg_ss::ods_furniture >("chg_dt_bed");
  commander_to_get_robots_pos = n.serviceClient< tms_msg_ss::ods_get_robots_pos >("ods_robot_position");
  commander_to_path_planner = n.serviceClient< tms_msg_rp::rps_path_planning >("rps_path_planning");
  commander_to_move_robot = n.serviceClient< tms_msg_rc::smartpal_control >("sp5_control");
  commander_to_sp5_get_state = n.serviceClient< tms_msg_rc::smartpal_control >("sp5_control");
  commander_to_get_robots_info = n.serviceClient< tms_msg_db::tmsdb_get_robots_info >("tmsdb_get_robots_info");
  commander_to_sp5_sound = n.serviceClient< tms_msg_rc::robot_tts >("smartpal5_tts");

  ros::spin();

  return 0;
}
