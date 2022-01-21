#include <ros/ros.h>
#include <tms_msg_rc/smartpal_control.h>

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
#define CMD_move 15

int main(int argc, char **argv)
{
  //----------------------------------------------------------------------------
  ros::init(argc, argv, "spc_test");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient< tms_msg_rc::smartpal_control >("sp4_control");
  tms_msg_rc::smartpal_control srv;

  //----------------------------------------------------------------------------
  srv.request.unit = UNIT_VEHICLE;
  srv.request.cmd = CMD_move;
  srv.request.arg.resize(3);
  srv.request.arg[0] = 100;
  srv.request.arg[1] = 100;
  srv.request.arg[2] = 90;

  if (client.call(srv))
    ROS_INFO("result: %d", srv.response.result);
  else
    ROS_ERROR("Failed to call service sp4_control");

  //----------------------------------------------------------------------------
  srv.request.unit = UNIT_ARM_R;
  srv.request.cmd = CMD_move;
  srv.request.arg.resize(8);
  srv.request.arg[0] = 0;
  srv.request.arg[1] = -30;
  srv.request.arg[2] = 0;
  srv.request.arg[3] = 0;
  srv.request.arg[4] = 0;
  srv.request.arg[5] = 0;
  srv.request.arg[6] = 0;
  srv.request.arg[7] = 10;

  if (client.call(srv))
    ROS_INFO("result: %d", srv.response.result);
  else
    ROS_ERROR("Failed to call service sp4_control");

  //----------------------------------------------------------------------------
  srv.request.unit = UNIT_ARM_L;
  srv.request.cmd = CMD_move;
  srv.request.arg.resize(8);
  srv.request.arg[0] = 0;
  srv.request.arg[1] = -30;
  srv.request.arg[2] = 0;
  srv.request.arg[3] = 0;
  srv.request.arg[4] = 0;
  srv.request.arg[5] = 0;
  srv.request.arg[6] = 0;
  srv.request.arg[7] = 10;

  if (client.call(srv))
    ROS_INFO("result: %d", srv.response.result);
  else
    ROS_ERROR("Failed to call service sp4_control");

  //----------------------------------------------------------------------------
  srv.request.unit = UNIT_GRIPPER_R;
  srv.request.cmd = CMD_move;
  srv.request.arg.resize(3);
  srv.request.arg[0] = 30;
  srv.request.arg[1] = 10;
  srv.request.arg[2] = 10;

  if (client.call(srv))
    ROS_INFO("result: %d", srv.response.result);
  else
    ROS_ERROR("Failed to call service sp4_control");

  //----------------------------------------------------------------------------
  srv.request.unit = UNIT_GRIPPER_L;
  srv.request.cmd = CMD_move;
  srv.request.arg.resize(3);
  srv.request.arg[0] = 30;
  srv.request.arg[1] = 10;
  srv.request.arg[2] = 10;

  if (client.call(srv))
    ROS_INFO("result: %d", srv.response.result);
  else
    ROS_ERROR("Failed to call service sp4_control");

  //----------------------------------------------------------------------------
  sleep(10);

  //----------------------------------------------------------------------------
  srv.request.unit = UNIT_VEHICLE;
  srv.request.cmd = CMD_move;
  srv.request.arg.resize(3);
  srv.request.arg[0] = -100;
  srv.request.arg[1] = -100;
  srv.request.arg[2] = -90;

  if (client.call(srv))
    ROS_INFO("result: %d", srv.response.result);
  else
    ROS_ERROR("Failed to call service sp4_control");

  //----------------------------------------------------------------------------
  srv.request.unit = UNIT_ARM_R;
  srv.request.cmd = CMD_move;
  srv.request.arg.resize(8);
  srv.request.arg[0] = 0;
  srv.request.arg[1] = -10;
  srv.request.arg[2] = 0;
  srv.request.arg[3] = 0;
  srv.request.arg[4] = 0;
  srv.request.arg[5] = 0;
  srv.request.arg[6] = 0;
  srv.request.arg[7] = 10;

  if (client.call(srv))
    ROS_INFO("result: %d", srv.response.result);
  else
    ROS_ERROR("Failed to call service sp4_control");

  //----------------------------------------------------------------------------
  srv.request.unit = UNIT_ARM_L;
  srv.request.cmd = CMD_move;
  srv.request.arg.resize(8);
  srv.request.arg[0] = 0;
  srv.request.arg[1] = -10;
  srv.request.arg[2] = 0;
  srv.request.arg[3] = 0;
  srv.request.arg[4] = 0;
  srv.request.arg[5] = 0;
  srv.request.arg[6] = 0;
  srv.request.arg[7] = 10;

  if (client.call(srv))
    ROS_INFO("result: %d", srv.response.result);
  else
    ROS_ERROR("Failed to call service sp4_control");

  //----------------------------------------------------------------------------
  srv.request.unit = UNIT_GRIPPER_R;
  srv.request.cmd = CMD_move;
  srv.request.arg.resize(3);
  srv.request.arg[0] = 0;
  srv.request.arg[1] = 10;
  srv.request.arg[2] = 10;

  if (client.call(srv))
    ROS_INFO("result: %d", srv.response.result);
  else
    ROS_ERROR("Failed to call service sp4_control");

  //----------------------------------------------------------------------------
  srv.request.unit = UNIT_GRIPPER_L;
  srv.request.cmd = CMD_move;
  srv.request.arg.resize(3);
  srv.request.arg[0] = 0;
  srv.request.arg[1] = 10;
  srv.request.arg[2] = 10;

  if (client.call(srv))
    ROS_INFO("result: %d", srv.response.result);
  else
    ROS_ERROR("Failed to call service sp4_control");

  //----------------------------------------------------------------------------

  return 0;
}

//------------------------------------------------------------------------------
