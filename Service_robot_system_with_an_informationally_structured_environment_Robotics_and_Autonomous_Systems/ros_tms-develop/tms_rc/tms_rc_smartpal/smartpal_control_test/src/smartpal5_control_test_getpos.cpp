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
  ros::ServiceClient client = n.serviceClient< tms_msg_rc::smartpal_control >("sp5_control");
  tms_msg_rc::smartpal_control srv;

  //----------------------------------------------------------------------------
  srv.request.unit = UNIT_VEHICLE;
  srv.request.cmd = CMD_getPose;
  srv.response.val.resize(3);

  if (client.call(srv))
  {
    ROS_INFO("result: %d", srv.response.result);
    ROS_INFO("value0: %f", srv.response.val[0]);
    ROS_INFO("value1: %f", srv.response.val[1]);
    ROS_INFO("value2: %f", srv.response.val[2]);
  }
  else
  {
    ROS_ERROR("Failed to call service sp5_control");
    return 1;
  }

  //----------------------------------------------------------------------------
  srv.request.unit = UNIT_ARM_R;
  srv.request.cmd = CMD_getPose;
  srv.request.arg.resize(1);
  srv.request.arg[0] = 0;
  srv.response.val.resize(7);
  if (client.call(srv))
  {
    ROS_INFO("result: %d", srv.response.result);
    ROS_INFO("value0: %f", srv.response.val[0]);
    ROS_INFO("value1: %f", srv.response.val[1]);
    ROS_INFO("value2: %f", srv.response.val[2]);
    ROS_INFO("value3: %f", srv.response.val[3]);
    ROS_INFO("value4: %f", srv.response.val[4]);
    ROS_INFO("value5: %f", srv.response.val[5]);
    ROS_INFO("value6: %f", srv.response.val[6]);
  }
  else
  {
    ROS_ERROR("Failed to call service sp5_control");
    return 1;
  }
  //----------------------------------------------------------------------------
  srv.request.unit = UNIT_GRIPPER_R;
  srv.request.cmd = CMD_getPose;
  srv.response.val.resize(1);
  if (client.call(srv))
  {
    ROS_INFO("result: %d", srv.response.result);
    ROS_INFO("value0: %f", srv.response.val[0]);
  }
  else
  {
    ROS_ERROR("Failed to call service sp5_control");
    return 1;
  }
  //----------------------------------------------------------------------------
  srv.request.unit = UNIT_LUMBA;
  srv.request.cmd = CMD_getPose;
  srv.response.val.resize(2);
  if (client.call(srv))
  {
    ROS_INFO("result: %d", srv.response.result);
    ROS_INFO("value0: %f", srv.response.val[0]);
    ROS_INFO("value1: %f", srv.response.val[1]);
  }
  else
  {
    ROS_ERROR("Failed to call service sp5_control");
    return 1;
  }

  //----------------------------------------------------------------------------

  return 0;
}

//------------------------------------------------------------------------------
