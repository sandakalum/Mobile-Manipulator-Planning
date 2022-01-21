///-----------------------------------------------------------------------------
/// @FileName smartpal5_virtual_control.cpp
/// @Date 2015.09.16 / 2013.09.09
/// @author Yoonseok Pyo and Yuka Hashiguchi
///-----------------------------------------------------------------------------
#include <ros/ros.h>
#include <unistd.h>
#include <tms_msg_rc/rc_robot_control.h>
#include <tms_msg_db/TmsdbStamped.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sstream>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <moveit_msgs/DisplayTrajectory.h>
#include <boost/bind.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tms_msg_db/TmsdbGetData.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/PlanningScene.h>
#include <stdlib.h>
#include <tf/transform_datatypes.h>

//#define OFF       0
#define ON 1

#define ArmR 0
#define ArmL 1

#define GripperR 0
#define GripperL 1

#define SUCCESS 1
#define FAILURE -1

#define OK 0
#define NG -1
#define STATUS_ERR -2
#define VALUE_ERR -3
#define NOT_SV_ON_ERR -4
#define FULL_MOTION_QUEUE_ERR -5
#define OVER_MAX_VEL_ERR -6
#define OVER_MAX_ACC_ERR -7
#define LOAD_ESTIMATE_ERR -8
#define FULL_COMMAND_ERR -9
#define OVER_MAX_VEL_WARNING -100
#define OVER_MAX_ACC_WARNING -101

#define CORBA_ERR -110
#define RL_ERR -111
#define SRV_UNIT_ERR -112
#define SRV_CMD_ERR -113
#define SRV_UNSUPPORTED_CMD_ERR -114

#define Unpowered 16   // 0x10
#define Powered 17     // 0x11
#define Ready 18       // 0x12
#define Busy 19        // 0x13
#define Paused 20      // 0x14
#define Alarm 21       // 0x15
#define jogBusy 22     // 0x16
#define DirectBusy 23  // 0x17
#define Locked 24      // 0x18
#define Stuck 25       // 0x19
#define Caution 26     // 0x1A

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
#define CMD_syncObj 8
#define CMD_move 15

ros::Publisher pose_publisher;
ros::Subscriber arm_data_sub;
ros::Subscriber object_data_sub;
ros::ServiceClient get_data_client_;

tf::TransformListener *listener;

const int sid_ = 100000;

double g_x = 5.2;   // 3.0;
double g_y = 2.9;   // 4.0;
double g_t = 0.65;  // 0.0;
double g_jR[7] = {0.0, -0.17, 0.0, 0.0, 0.0, 0.0, 0.0};
double g_jL[7] = {0.0, -0.17, 0.0, 0.0, 0.0, 0.0, 0.0};

double g_gripper_right = 0.3;
double g_gripper_left = 0.3;

double g_lumba_high = 0.0;
double g_lumba_low = 0.0;

double g_r_state = 1;

double g_vehicle_velocity_t = 100.0;   // Velocity of translation (now mm/s)
double g_vehicle_velocity_r = 1000.0;  // Velocity of rotation   (now deg/s)

double g_joint_velocity = 10.0;      // Velocity (now deg/s)
double g_joint_acceleration = 10.0;  // Acceleration (now deg/s^2)

bool grasping = false;
bool is_grasp = false;

int grasping_object_id = 0;

int g_oid;
double g_ox;
double g_oy;
double g_oz;
double g_orr;
double g_orp;
double g_ory;

double g_place;
double g_o_state = 1;

int8_t SyncObj(double r_x, double r_y, double r_ry, double r_state, double o_id, double o_x, double o_y, double o_z,
               double o_rr, double o_rp, double o_ry, double o_place, double o_state)
{
  bool ret = true;
  grasping = true;

  if (r_x != -1)
  {
    g_x = r_x;  // m
  }

  if (r_y != -1)
  {
    g_y = r_y;  // m
  }

  if (r_ry != -1)
  {
    g_t = r_ry;  // rad
  }

  if (r_state != -1)
  {
    g_r_state = r_state;
  }

  if (o_id != -1)
    g_oid = (int)o_id;
  if (o_x != -1)
    g_ox = o_x;
  if (o_y != -1)
    g_oy = o_y;
  if (o_z != -1)
    g_oz = o_z;
  if (o_rr != -1)
    g_orr = o_rr;
  if (o_rp != -1)
    g_orp = o_rp;
  if (o_ry != -1)
    g_ory = o_ry;

  if (o_place != -1)
    g_place = o_place;
  if (o_state != -1)
    g_o_state = o_state;

  printf("SyncObj result: %0.1fmm, %0.1fmm, %0.1frad, %f\n ", g_x, g_y, g_t, g_r_state);
  printf("SyncObj result: %d, %0.1fmm, %0.1fmm, %0.1fmm, %0.1frad, %0.1fdeg, %0.1fdeg\n", g_oid, g_ox, g_oy, g_oz,
         g_orr, g_orp, g_ory);
  printf("SyncObj result: %0.1f, %0.1f\n ", g_place, g_o_state);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
  {
    grasping = false;
    return SUCCESS;
  }
  else
    return FAILURE;
}

int8_t CalcBackground(double r2_x, double r2_y, double r2_ry, double o_x, double o_y, double o_z, double o_rr,
                      double o_rp, double o_ry, double r2_wh, double r2_wl, double r2_j0, double r2_j1, double r2_j2,
                      double r2_j3, double r2_j4, double r2_j5, double r2_j6, double r2_gR)
{
  bool ret = true;
  grasping = true;

  // g_x2 = r2_x;  // m
  // g_y2 = r2_y;  // m
  // g_t2 = r2_ry; // rad
  //
  // if (o_x  != -1) g_ox  = o_x;
  // if (o_y  != -1) g_oy  = o_y;
  // if (o_z  != -1) g_oz  = o_z;
  // if (o_rr != -1) g_orr = o_rr;
  // if (o_rp != -1) g_orp = o_rp;
  // if (o_ry != -1) g_ory = o_ry;
  //
  // if (r2_wh != -1)
  // {
  //   g_lumba_high2 = r2_wh;
  //   g_lumba_high  = r2_wh;
  // }
  //
  // if (r2_wl != -1)
  // {
  //   g_lumba_low2 = r2_wl;
  //   g_lumba_low  = r2_wl;
  // }
  //
  // if (r2_j0 != -1)
  // {
  //   g_jR2[0] = r2_j0;
  //   g_jR[0]  = r2_j0;
  // }
  //
  // if (r2_j1 != -1)
  // {
  //   g_jR2[1] = r2_j1;
  //   g_jR[1]  = r2_j1;
  // }
  //
  // if (r2_j2 != -1)
  // {
  //   g_jR2[2] = r2_j2;
  //   g_jR[2]  = r2_j2;
  // }
  //
  // if (r2_j3 != -1)
  // {
  //   g_jR2[3] = r2_j3;
  //   g_jR[3]  = r2_j3;
  // }
  //
  // if (r2_j4 != -1)
  // {
  //   g_jR2[4] = r2_j4;
  //   g_jR[4]  = r2_j4;
  // }
  //
  // if (r2_j5 != -1)
  // {
  //   g_jR2[5] = r2_j5;
  //   g_jR[5]  = r2_j5;
  // }
  //
  // if (r2_j6 != -1)
  // {
  //   g_jR2[6] = r2_j6;
  //   g_jR[6]  = r2_j6;
  // }
  //
  // if (r2_gR != -1)
  // {
  //   g_gripper_right2 = r2_gR;
  //   g_gripper_right  = r2_gR;
  // }
  //
  // printf("CalcBackground result: %0.1fm, %0.1fm, %0.1frad\n ",g_x2, g_y2, g_t2);
  // ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
  {
    grasping = false;
    return SUCCESS;
  }
  else
    return FAILURE;
}

int8_t VehicleGetPos(double *x_m, double *y_m, double *theta_rad)
{
  *x_m = g_x;
  *y_m = g_y;
  *theta_rad = g_t;

  bool ret = true;

  printf("vehicleGetPose result: %0.1fm, %0.1fm, %0.1frad ", *x_m, *y_m, *theta_rad);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

int8_t VehicleSetPos(double x_m, double y_m, double theta_rad)
{
  bool ret = true;

  printf("vehicleSetPose(%0.1fm, %0.1fm, %0.1frad) result:", x_m, y_m, theta_rad);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

int8_t VehicleSetVel(double velT_mps, double velR_radps)
{
  // Init Value : translation = 100(mm/s),  rotation =10(deg/s)
  // Max  Value : translation = 1000(mm/s), rotation =100(deg/s)
  g_vehicle_velocity_t = velT_mps;    // Velocity of translation (m/s)
  g_vehicle_velocity_r = velR_radps;  // Velocity of rotation   (rad/s)

  bool ret = true;

  printf("vehicleSetVel(%0.1fm/s, %0.1frad/s) result:", g_vehicle_velocity_t, g_vehicle_velocity_r);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

int8_t VehicleMoveLinearAbs(double x_m, double y_m, double theta_rad)
{
  bool ret = true;

  g_x = x_m;
  g_y = y_m;
  g_t = theta_rad;

  printf("vehicleMoveLinearAbs(%0.1fm, %0.1fm, %0.1frad) result:", g_x, g_y, g_t);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

int8_t ArmGetPos(int8_t RL, double frameID, double *posdata)
{
  int8_t ret = SUCCESS;

  if (RL == ArmR)
  {
    ret = SUCCESS;
    printf("armGetPose R: %0.1f %0.1f %0.1f %0.1f %0.1f %0.1f %0.1f\n", g_jR[0], g_jR[1], g_jR[2], g_jR[3], g_jR[4],
           g_jR[5], g_jR[6]);
  }
  else if (RL == ArmL)
  {
    ret = SUCCESS;
    printf("armGetPose L: %0.1f %0.1f %0.1f %0.1f %0.1f %0.1f %0.1f\n", g_jL[0], g_jL[1], g_jL[2], g_jL[3], g_jL[4],
           g_jL[5], g_jL[6]);
  }
  else
  {
    printf("armGetPose RL error\n");
    return RL_ERR;
  }

  return ret;
}

int8_t ArmSetJointAcc(int8_t RL, double acc_mps2)
{
  int8_t ret;

  if (RL == ArmR)
  {
    g_joint_acceleration = acc_mps2;
    ret = SUCCESS;
    printf("armSetJointAcc R result: SUCCESS");
  }
  else if (RL == ArmL)
  {
    g_joint_acceleration = acc_mps2;
    ret = SUCCESS;
    printf("armSetJointAcc L result: SUCCESSSUCCESS");
  }
  else
  {
    printf("armSetJointAcc RL error\n");
    return RL_ERR;
  }

  return ret;
}

int8_t ArmMoveJointAbs(int8_t RL, double *joint_rad, double vel_radps)
{
  int8_t ret;

  // J1   -45   ~ +180   : init  0
  // J2  -109.5 ~  +14.5 : init -10
  // J3  -120   ~ +120   : init  0
  // J4    -1.5 ~ +129.5 : init  0
  // J5  -120   ~ +120   : init  0
  // J6   -15.5 ~  +44   : init  0
  // J7   -89.5 ~  +59   : init  0

  g_joint_velocity = vel_radps;  // Velocity (rad/s): init= (but now 10 deg/s)

  if (RL == ArmR)
  {
    for (int i = 0; i < 7; i++)
      g_jR[i] = joint_rad[i];

    ret = SUCCESS;
    printf("armMoveJointAbs R ( ");
    for (int i = 0; i < 7; i++)
      printf("%0.1f ", g_jR[i]);
    printf("): ");
  }
  else if (RL == ArmL)
  {
    for (int i = 0; i < 7; i++)
      g_jL[i] = joint_rad[i];

    ret = SUCCESS;
    printf("armMoveJointAbs L ( ");
    for (int i = 0; i < 7; i++)
      printf("%0.1f ", g_jL[i]);
    printf("): ");
  }
  else
  {
    printf("armMoveJointAbs RL error\n");
    return RL_ERR;
  }

  return ret;
}

int8_t GripperGetPos(int8_t RL, double *pos)
{
  bool ret;

  if (RL == GripperR)
  {
    *pos = g_gripper_right;
    ret = true;
    printf("gripperGetPos R result: ");
    ret ? printf("Success\n") : printf("Failure\n");
    printf("gripperGetPos R: %0.1f\n", *pos);
  }
  else if (RL == GripperL)
  {
    *pos = g_gripper_left;
    ret = true;
    printf("gripperGetPos L result: ");
    ret ? printf("Success\n") : printf("Failure\n");
    printf("gripperGetPos L: %0.1f\n", *pos);
  }
  else
  {
    printf("gripperGetPos RL error\n");
    return RL_ERR;
  }

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

int8_t GripperMoveAbs(int8_t RL, double j_rad, double vel_radps, double acc_radps2)
{
  bool ret;

  // smartpal5 gripper -58 ∼ +8 degree (open direct : -)

  if (RL == GripperR)
  {
    g_gripper_right = j_rad;
    ret = true;
    printf("gripperMoveAbs R ( %0.1f ): ", j_rad);
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else if (RL == GripperL)
  {
    g_gripper_left = j_rad;
    ret = true;
    printf("gripperMoveAbs L ( %0.1f ): ", j_rad);
    ret ? printf("Success\n") : printf("Failure\n");
  }
  else
  {
    printf("gripperMoveAbs RL error\n");
    return RL_ERR;
  }

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

int8_t LumbaGetPos(double *low_rad, double *high_rad)
{
  *low_rad = g_lumba_high;
  *high_rad = g_lumba_low;

  bool ret = true;

  printf("lumbaGetPos result: %0.1frad, %0.1frad ", *low_rad, *high_rad);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

int8_t LumbaMoveCooperative(double z_rad, double vel_radps, double acc_radps2)
{
  g_lumba_high = 4 * z_rad / 3;
  g_lumba_low = -z_rad / 3;

  bool ret = true;

  printf("lumbaMoveCooperative(%0.1frad, %0.1frad/s, %0.1frdad/s^2) result:", z_rad, vel_radps, acc_radps2);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

int8_t LumbaMove(double low_rad, double high_rad, double vel_radps, double acc_radps2)
{
  g_lumba_high = high_rad;
  g_lumba_low = low_rad;

  bool ret = true;

  printf("lumbaMove(%0.1frad, %0.1frad, %0.1frad/s, %0.1frad/s^2) result:", low_rad, high_rad, vel_radps, acc_radps2);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

bool robotControl(tms_msg_rc::rc_robot_control::Request &req, tms_msg_rc::rc_robot_control::Response &res)
{
  switch (req.unit)
  {
    case 0:  // all
      switch (req.cmd)
      {
        case 0:
          res.result = SUCCESS;
          break;  // clearAlarm
        case 1:
          res.result = SUCCESS;
          break;  // setPower
        case 2:
          res.result = SUCCESS;
          break;  // setServo
        case 3:
          res.result = SUCCESS;
          break;  // pause
        case 4:
          res.result = SUCCESS;
          break;  // resume
        case 5:
          res.result = SUCCESS;
          break;  // abort
        case 6:
          res.result = SUCCESS;
          break;  // stop
        case 7:
          res.result = SUCCESS;
          break;  // setodom for sp5_controller
        case 8:
          res.result = SyncObj(req.arg[0], req.arg[1], req.arg[2], req.arg[3], req.arg[4], req.arg[5], req.arg[6],
                               req.arg[7], req.arg[8], req.arg[9], req.arg[10], req.arg[11], req.arg[12]);
          break;
        case 9:
          res.result =
              CalcBackground(req.arg[0], req.arg[1], req.arg[2], req.arg[3], req.arg[4], req.arg[5], req.arg[6],
                             req.arg[7], req.arg[8], req.arg[9], req.arg[10], req.arg[11], req.arg[12], req.arg[13],
                             req.arg[14], req.arg[15], req.arg[16], req.arg[17], req.arg[18]);
          break;
        default:
          res.result = SRV_CMD_ERR;
          break;
      }
      break;
    //--------------------------------------------------------------------------
    case 1:  // vehicle
      switch (req.cmd)
      {
        case 0:
          res.result = SUCCESS;
          break;
        case 1:
          res.result = SUCCESS;
          break;
        case 2:
          res.result = SUCCESS;
          break;
        case 3:
          res.result = SUCCESS;
          break;
        case 4:
          res.result = SUCCESS;
          break;
        case 6:
          res.result = SUCCESS;
          break;
        case 7:
          res.result = SUCCESS;
          break;
        case 8:
          res.val.resize(3);
          res.result = VehicleGetPos(&res.val[0], &res.val[1], &res.val[2]);
          break;
        case 9:
          res.result = VehicleSetPos(req.arg[0], req.arg[1], req.arg[2]);
          break;
        case 10:
          res.result = VehicleSetVel(req.arg[0], req.arg[1]);
          break;
        case 11:
          res.result = SUCCESS;
          break;
        case 15:
          res.result = VehicleMoveLinearAbs(req.arg[0], req.arg[1], req.arg[2]);
          break;
        case 16:
          res.result = SUCCESS;
          break;
        case 17:
          res.result = SUCCESS;
          break;
        case 18:
          res.result = SUCCESS;
          break;
        case 19:
          res.result = SUCCESS;
          break;
        case 20:
          res.result = SUCCESS;
          break;
        case 25:
          res.result = SUCCESS;
          break;
        case 26:
          res.result = SUCCESS;
          break;
        default:
          res.result = SRV_CMD_ERR;
          break;
      }
      break;
    //--------------------------------------------------------------------------
    case 2:  // ArmR
      switch (req.cmd)
      {
        case 0:
          res.result = SUCCESS;
          break;
        case 1:
          res.result = SUCCESS;
          break;
        case 2:
          res.result = SUCCESS;
          break;
        case 3:
          res.result = SUCCESS;
          break;
        case 4:
          res.result = SUCCESS;
          break;
        case 5:
          res.result = SUCCESS;
          break;
        case 6:
          res.result = SUCCESS;
          break;
        case 7:
          res.result = SUCCESS;
          break;
        case 8:
          res.val.resize(7);
          res.result = ArmGetPos(ArmR, req.arg[0], &res.val[0]);
          break;
        case 10:
          res.result = ArmSetJointAcc(ArmR, req.arg[0]);
          break;
        case 11:
          res.result = SUCCESS;
          break;
        case 15:
          res.result = ArmMoveJointAbs(ArmR, &req.arg[0], req.arg[7]);
          break;
        case 16:
          res.result = SUCCESS;
          break;
        case 17:
          res.result = SUCCESS;
          break;
        case 18:
          res.result = SUCCESS;
          break;
        default:
          res.result = SRV_CMD_ERR;
          break;
      }
      break;
    //--------------------------------------------------------------------------
    case 3:  // ArmL
      switch (req.cmd)
      {
        case 0:
          res.result = SUCCESS;
          break;
        case 1:
          res.result = SUCCESS;
          break;
        case 2:
          res.result = SUCCESS;
          break;
        case 3:
          res.result = SUCCESS;
          break;
        case 4:
          res.result = SUCCESS;
          break;
        case 5:
          res.result = SUCCESS;
          break;
        case 6:
          res.result = SUCCESS;
          break;
        case 7:
          res.result = SUCCESS;
          break;
        case 8:
          res.val.resize(7);
          res.result = ArmGetPos(ArmL, req.arg[0], &res.val[0]);
          break;
        case 10:
          res.result = ArmSetJointAcc(ArmL, req.arg[0]);
          break;
        case 11:
          res.result = SUCCESS;
          break;
        case 15:
          res.result = ArmMoveJointAbs(ArmL, &req.arg[0], req.arg[7]);
          break;
        case 16:
          res.result = SUCCESS;
          break;
        case 17:
          res.result = SUCCESS;
          break;
        case 18:
          res.result = SUCCESS;
          break;
        default:
          res.result = SRV_CMD_ERR;
          break;
      }
      break;
    //--------------------------------------------------------------------------
    case 4:  // GripperR
      switch (req.cmd)
      {
        case 0:
          res.result = SUCCESS;
          break;
        case 1:
          res.result = SUCCESS;
          break;
        case 2:
          res.result = SUCCESS;
          break;
        case 3:
          res.result = SUCCESS;
          break;
        case 4:
          res.result = SUCCESS;
          break;
        case 5:
          res.result = SUCCESS;
          break;
        case 6:
          res.result = SUCCESS;
          break;
        case 7:
          res.result = SUCCESS;
          break;
        case 8:
          res.val.resize(1);
          res.result = GripperGetPos(ArmR, &res.val[0]);
          break;
        case 15:
          res.result = GripperMoveAbs(ArmR, req.arg[0], req.arg[1], req.arg[2]);
          break;
        default:
          res.result = SRV_CMD_ERR;
          break;
      }
      break;
    //--------------------------------------------------------------------------
    case 5:  // GripperL
      switch (req.cmd)
      {
        case 0:
          res.result = SUCCESS;
          break;
        case 1:
          res.result = SUCCESS;
          break;
        case 2:
          res.result = SUCCESS;
          break;
        case 3:
          res.result = SUCCESS;
          break;
        case 4:
          res.result = SUCCESS;
          break;
        case 5:
          res.result = SUCCESS;
          break;
        case 6:
          res.result = SUCCESS;
          break;
        case 7:
          res.result = SUCCESS;
          break;
        case 8:
          res.val.resize(1);
          res.result = GripperGetPos(ArmL, &res.val[0]);
          break;
        case 15:
          res.result = GripperMoveAbs(ArmL, req.arg[0], req.arg[1], req.arg[2]);
          break;
        default:
          res.result = SRV_CMD_ERR;
          break;
      }
      break;
    //--------------------------------------------------------------------------
    case 6:  // Waist
      switch (req.cmd)
      {
        case 0:
          res.result = SUCCESS;
          break;
        case 1:
          res.result = SUCCESS;
          break;
        case 2:
          res.result = SUCCESS;
          break;
        case 3:
          res.result = SUCCESS;
          break;
        case 4:
          res.result = SUCCESS;
          break;
        case 5:
          res.result = SUCCESS;
          break;
        case 6:
          res.result = SUCCESS;
          break;
        case 7:
          res.result = SUCCESS;
          break;
        case 8:
          res.val.resize(2);
          res.result = LumbaGetPos(&res.val[0], &res.val[1]);
          break;
        case 15:
          res.result = LumbaMoveCooperative(req.arg[0], req.arg[1], req.arg[2]);
          break;
        case 16:
          res.result = LumbaMove(req.arg[0], req.arg[1], req.arg[2], req.arg[3]);
          break;
        case 17:
          res.result = SUCCESS;
          break;
        case 18:
          res.result = SUCCESS;
          break;
        default:
          res.result = SRV_CMD_ERR;
          break;
      }
      break;
    //--------------------------------------------------------------------------
    case 7:  // CC
      res.result = SRV_UNIT_ERR;
      break;
      break;
    //--------------------------------------------------------------------------
    default:
      res.result = SRV_UNIT_ERR;
      break;
  }
  return true;
}

//------------------------------------------------------------------------------
void RobotDataUpdate()
{
  ros::Rate loop_rate(10);  // 10Hz frequency (0.1 sec)

  while (ros::ok())
  {
    ros::Time now = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9

    tms_msg_db::TmsdbStamped db_msg;
    tms_msg_db::Tmsdb current_pos_data;

    db_msg.header.frame_id = "/world";
    db_msg.header.stamp = now;

    current_pos_data.time = boost::posix_time::to_iso_extended_string(now.toBoost());
    current_pos_data.id = 2003;  // smartpal5_2
    current_pos_data.name = "smartpal5_2";
    current_pos_data.type = "robot";
    current_pos_data.x = g_x;
    current_pos_data.y = g_y;
    current_pos_data.z = 0.0;
    current_pos_data.rr = 0.0;
    current_pos_data.rp = 0.0;
    current_pos_data.ry = g_t;
    current_pos_data.place = 5001;
    current_pos_data.sensor = 3005;
    current_pos_data.state = g_r_state;

    std::stringstream ss;
    ss << g_lumba_high << ";" << g_lumba_low << ";";

    for (int i = 0; i < 7; i++)
    {
      ss << g_jR[i] << ";";
    }
    ss << g_gripper_right << ";";

    for (int i = 0; i < 7; i++)
    {
      ss << g_jL[i] << ";";
    }
    ss << g_gripper_left;

    // joint information of smartpal5
    // lumba_low, lumba_high, jR[0]...[6], gripper_right, jL[0]...[6], gripper_left
    current_pos_data.joint = ss.str();

    std::stringstream ss2;
    ss2 << "grasping=" << grasping_object_id;

    current_pos_data.note = ss2.str();

    db_msg.tmsdb.push_back(current_pos_data);
    pose_publisher.publish(db_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void armCallback(const sensor_msgs::JointState &msg)
{
  if (msg.name[0] == "l_gripper_thumb_joint")
  {
    g_gripper_left = msg.position[0];
    printf("gripper_left=%f\n", g_gripper_left);
  }
  else
  {
    for (int i = 0; i < 7; i++)
    {
      g_jL[i] = msg.position[i];
      printf("g_jL[%d]=%f ", i, g_jL[i]);
    }
    printf("\n");
  }
}

void ObjectDataUpdate(const moveit_msgs::PlanningScene &msg)
{
  if (msg.robot_state.attached_collision_objects.size() != 0)
  {  // grasped object
    int object_id = atoi(msg.robot_state.attached_collision_objects[0].object.id.c_str());

    g_oid = object_id;

    tms_msg_db::TmsdbGetData srv;
    srv.request.tmsdb.id = object_id + sid_;

    if (get_data_client_.call(srv))
    {
      // g_oz -= srv.response.tmsdb[0].offset_z;
      geometry_msgs::PoseStamped pose, pose2;
      pose.header.frame_id = "/l_end_effector_link";
      pose.pose.position.x = msg.robot_state.attached_collision_objects[0].object.primitive_poses[0].position.x;
      pose.pose.position.y = msg.robot_state.attached_collision_objects[0].object.primitive_poses[0].position.y;
      pose.pose.position.z = msg.robot_state.attached_collision_objects[0].object.primitive_poses[0].position.z -
                             srv.response.tmsdb[0].offset_z;

      pose.pose.orientation.x = msg.robot_state.attached_collision_objects[0].object.primitive_poses[0].orientation.x;
      pose.pose.orientation.y = msg.robot_state.attached_collision_objects[0].object.primitive_poses[0].orientation.y;
      pose.pose.orientation.z = msg.robot_state.attached_collision_objects[0].object.primitive_poses[0].orientation.z;
      pose.pose.orientation.w = msg.robot_state.attached_collision_objects[0].object.primitive_poses[0].orientation.w;

      listener->transformPose("/world_link", pose, pose2);

      g_ox = pose2.pose.position.x;
      g_oy = pose2.pose.position.y;
      g_oz = pose2.pose.position.z;
      tf::Quaternion q2(pose2.pose.orientation.x, pose2.pose.orientation.y, pose2.pose.orientation.z,
                        pose2.pose.orientation.w);
      tf::Matrix3x3 m2(q2);
      m2.getRPY(g_orr, g_orp, g_ory);

      ROS_INFO("object_rot:(%f,%f,%f)", g_orr, g_orp, g_ory);

      ros::Time now = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9

      tms_msg_db::TmsdbStamped db_msg;
      tms_msg_db::Tmsdb current_pos_data;

      db_msg.header.frame_id = "/world";
      db_msg.header.stamp = now;

      current_pos_data.time = boost::posix_time::to_iso_extended_string(now.toBoost());
      current_pos_data.id = g_oid;
      current_pos_data.name = srv.response.tmsdb[0].name;
      current_pos_data.type = srv.response.tmsdb[0].type;
      current_pos_data.x = g_ox;
      current_pos_data.y = g_oy;
      current_pos_data.z = g_oz;
      current_pos_data.rr = g_orr;
      current_pos_data.rp = g_orp;
      current_pos_data.ry = g_ory;
      current_pos_data.offset_x = srv.response.tmsdb[0].offset_x;
      current_pos_data.offset_y = srv.response.tmsdb[0].offset_y;
      current_pos_data.offset_z = srv.response.tmsdb[0].offset_z;
      current_pos_data.place = 2003;
      current_pos_data.sensor = 3005;
      current_pos_data.probability = 1.0;
      current_pos_data.state = 2;

      db_msg.tmsdb.push_back(current_pos_data);
      pose_publisher.publish(db_msg);

      is_grasp = true;
      grasping_object_id = g_oid;
      printf("grasped object id:%d (%0.1f,%0.1f,%0.1f)\n", g_oid, g_ox, g_oy, g_oz);
    }
    else
    {
      ROS_ERROR("failed to get data");
    }
  }
  if (is_grasp == true && msg.world.collision_objects.size() != 0 &&
      msg.world.collision_objects[0].primitive_poses.size() != 0)
  {  // released object
    is_grasp = false;
    grasping_object_id = 0;

    int object_id = atoi(msg.world.collision_objects[0].id.c_str());

    g_oid = object_id;
    g_ox = msg.world.collision_objects[0].primitive_poses[0].position.x;
    g_oy = msg.world.collision_objects[0].primitive_poses[0].position.y;
    g_oz = msg.world.collision_objects[0].primitive_poses[0].position.z;

    tms_msg_db::TmsdbGetData srv;
    srv.request.tmsdb.id = object_id + sid_;

    if (get_data_client_.call(srv))
    {
      g_oz -= srv.response.tmsdb[0].offset_z;

      ros::Time now = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9

      tms_msg_db::TmsdbStamped db_msg;
      tms_msg_db::Tmsdb current_pos_data;

      db_msg.header.frame_id = "/world";
      db_msg.header.stamp = now;

      current_pos_data.time = boost::posix_time::to_iso_extended_string(now.toBoost());
      current_pos_data.id = g_oid;
      current_pos_data.name = srv.response.tmsdb[0].name;
      current_pos_data.type = srv.response.tmsdb[0].type;
      current_pos_data.x = g_ox;
      current_pos_data.y = g_oy;
      current_pos_data.z = g_oz;
      current_pos_data.rr = g_orr;
      current_pos_data.rp = g_orp;
      current_pos_data.ry = g_ory;
      current_pos_data.offset_x = srv.response.tmsdb[0].offset_x;
      current_pos_data.offset_y = srv.response.tmsdb[0].offset_y;
      current_pos_data.offset_z = srv.response.tmsdb[0].offset_z;
      current_pos_data.place = 5002;
      current_pos_data.sensor = 3005;
      current_pos_data.probability = 1.0;
      current_pos_data.state = 0;  // 1;

      db_msg.tmsdb.push_back(current_pos_data);
      pose_publisher.publish(db_msg);

      printf("released object id:%d (%0.1f,%0.1f,%0.1f)\n", g_oid, g_ox, g_oy, g_oz);
    }
    else
    {
      ROS_ERROR("failed to get data");
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "smartpal5_virtual_control");
  ros::NodeHandle nh;

  listener = new tf::TransformListener;

  ros::ServiceServer service = nh.advertiseService("sp5_virtual_control", robotControl);
  pose_publisher = nh.advertise< tms_msg_db::TmsdbStamped >("tms_db_data", 10);
  arm_data_sub = nh.subscribe("/move_group/fake_controller_joint_states", 1, &armCallback);
  object_data_sub = nh.subscribe("/move_group/monitored_planning_scene", 1, &ObjectDataUpdate);
  get_data_client_ = nh.serviceClient< tms_msg_db::TmsdbGetData >("/tms_db_reader");

  nh.setParam("/2003_is_real", false);

  printf("Virtual SmartPal initialization has been completed.\n\n");

  boost::thread thr_rdu(&RobotDataUpdate);

  thr_rdu.join();

  delete listener;

  return (0);
}
