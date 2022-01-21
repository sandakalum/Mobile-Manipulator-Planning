
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
#include <tms_msg_rc/smartpal_control.h>
#include "tms_msg_rc/smartpal_speak.h"

#include <sstream>

using namespace std;

#define UNIT_ALL 0
#define UNIT_VEHICLE 1
#define UNIT_ARM_R 2
#define UNIT_ARM_L 3
#define UNIT_GRIPPER_R 4
#define UNIT_GRIPPER_L 5
#define UNIT_LUMBA 6
#define UNIT_CC 7
#define CMD_clearAlarm 0
#define CMD_setPower 1
#define CMD_setServo 2
#define CMD_pause 3
#define CMD_resume 4
#define CMD_abort 5
#define CMD_stop 6
#define CMD_getState 7
#define CMD_getPose 8
#define CMD_setPose 9
#define CMD_setVel 10
#define CMD_setAcc 11
#define CMD_move 15

ros::ServiceClient client_path_checking;
ros::ServiceClient client_smartpal5_control;
ros::ServiceClient client_smartpal5_speak;

tms_msg_rp::rps_voronoi_path_planning service_voronoi_path;
tms_msg_rp::rps_path_checking rpc;
tms_msg_rc::smartpal_control srv_smartpal_control;
tms_msg_rc::smartpal_speak srv_smartpal_speak;

enum PATH_ALARM path_alarm;

void alarm_check(const tms_msg_rp::rps_path_alarm::ConstPtr &msg_path_alarm)
{
  cout << "alarm checking..." << endl;

  path_alarm = PATH_ALARM(msg_path_alarm->path_alarm);

  switch (path_alarm)
  {
    case SAFE:
      cout << "SAFE" << endl;

      srv_smartpal_control.request.unit = UNIT_ALL;
      srv_smartpal_control.request.cmd = CMD_resume;
      if (client_smartpal5_control.call(srv_smartpal_control))
      {
        ROS_INFO("result: %d", srv_smartpal_control.response.result);
      }
      else
      {
        ROS_ERROR("Failed to call service sp5_control");
        return;
      }

      break;
    case NOT_PATH:
      cout << "NOT_PATH" << endl;
      //~ rpc.request.robot_id = 1;
      //~ rpc.request.rps_robot_path.rps_route.clear();
      //~ rpc.request.rps_robot_path.rps_route = msg_path_alarm->rps_route;
      //~ if(client_path_checking.call(rpc))
      //~ {
      //~ ROS_INFO("Success: %ld", (long int)rpc.response.success);
      //~ if(rpc.response.success){
      //~ std::cout<<"	"<<rpc.response.message<<std::endl;
      //~ return;
      //~ }
      //~ }
      //~ else
      //~ {
      //~ ROS_ERROR("Failed to call path checking");
      //~ return;
      //~ }

      srv_smartpal_control.request.unit = UNIT_ALL;
      srv_smartpal_control.request.cmd = CMD_pause;
      if (client_smartpal5_control.call(srv_smartpal_control))
      {
        ROS_INFO("result: %d", srv_smartpal_control.response.result);
      }
      else
      {
        ROS_ERROR("Failed to call service sp5_control");
        return;
      }

      break;
    case PATH_BLOCK:
      cout << "PATH_BLOCK" << endl;
      //~ rpc.request.robot_id = 1;
      //~ rpc.request.rps_robot_path.rps_route.clear();
      //~ rpc.request.rps_robot_path.rps_route = msg_path_alarm->rps_route;
      //~ if(client_path_checking.call(rpc))
      //~ {
      //~ ROS_INFO("Success: %ld", (long int)rpc.response.success);
      //~ if(rpc.response.success){
      //~ std::cout<<"	"<<rpc.response.message<<std::endl;
      //~ return;
      //~ }
      //~ }
      //~ else
      //~ {
      //~ ROS_ERROR("Failed to call path checking");
      //~ return;
      //~ }

      srv_smartpal_control.request.unit = UNIT_ALL;
      srv_smartpal_control.request.cmd = CMD_pause;
      if (client_smartpal5_control.call(srv_smartpal_control))
      {
        ROS_INFO("result: %d", srv_smartpal_control.response.result);
      }
      else
      {
        ROS_ERROR("Failed to call service sp5_control");
        return;
      }

      srv_smartpal_speak.request.words = "Your  position  is  danger.  Move,  please.";
      if (!client_smartpal5_speak.call(srv_smartpal_speak))
      {
        ROS_ERROR("Failed to call service sp5_control");
        return;
      }

      sleep(3);

      break;
    case GOAL_BLOCK:
      cout << "GOAL_BLOCK" << endl;
      //~ rpc.request.robot_id = 1;
      //~ rpc.request.rps_robot_path.rps_route.clear();
      //~ if(client_path_checking.call(rpc))
      //~ {
      //~ ROS_INFO("Success: %ld", (long int)rpc.response.success);
      //~ if(rpc.response.success){
      //~ std::cout<<"	"<<rpc.response.message<<std::endl;
      //~ return;
      //~ }
      //~ }
      //~ else
      //~ {
      //~ ROS_ERROR("Failed to call path checking");
      //~ return;
      //~ }

      srv_smartpal_control.request.unit = UNIT_ALL;
      srv_smartpal_control.request.cmd = CMD_pause;
      if (client_smartpal5_control.call(srv_smartpal_control))
      {
        ROS_INFO("result: %d", srv_smartpal_control.response.result);
      }
      else
      {
        ROS_ERROR("Failed to call service sp5_control");
        return;
      }

      srv_smartpal_speak.request.words = "Your  position  is  danger.  Move,  please.";
      if (!client_smartpal5_speak.call(srv_smartpal_speak))
      {
        ROS_ERROR("Failed to call service sp5_control");
        return;
      }

      sleep(3);

      break;
    case CAUTION:
      cout << "CAUTION" << endl;
      break;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rps_alarm_checker");

  ros::NodeHandle n;

  client_path_checking = n.serviceClient< tms_msg_rp::rps_path_checking >("rps_path_checking");
  client_smartpal5_control = n.serviceClient< tms_msg_rc::smartpal_control >("sp5_control");
  client_smartpal5_speak = n.serviceClient< tms_msg_rc::smartpal_speak >("sp5_speak");

  ros::Subscriber rps_alarm_subscriber = n.subscribe("rps_path_alarm", 1, alarm_check);

  ros::spin();

  return 0;
}
