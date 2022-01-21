///-----------------------------------------------------------------------------
/// @FileName smartpal5_control.cpp /// @Date 2014.06.18 / 2013.06.02
/// @author Yoonseok Pyo (passionvirus@gmail.com)
///-----------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <tms_msg_rc/smartpal_control.h>
#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_db/TmsdbGetData.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/PlanningScene.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

//------------------------------------------------------------------------------
#include "sp5_client.h"

//------------------------------------------------------------------------------
Client *smartpal = new Client;
ros::ServiceClient get_data_client;
ros::Publisher pose_publisher;
ros::Subscriber object_data_sub;

tf::TransformListener *listener;

bool is_grasp = false;
int grasping_object_id = 0;

class armInfo
{
public:
  int move;
  double j_L[7];         // 0
  double gripper_left;   // 1
  double j_R[7];         // 2
  double gripper_right;  // 3
};
std::vector< armInfo > trajectory;

const int sid_ = 100000;

int g_oid;
double g_ox;
double g_oy;
double g_oz;
double g_orr;
double g_orp;
double g_ory;
//------------------------------------------------------------------------------
int getRobotCurrentPos(double *x, double *y, double *th, double *waistL, double *waistH, double *jointR,
                       double *gripperR, double *jointL, double *gripperL)
{
  int8_t ret1, ret2, ret3, ret4, ret5, ret6;
  ret1 = smartpal->vehicleGetPos(x, y, th);
  ret2 = smartpal->armGetPos(ArmR, 0, jointR);
  ret3 = smartpal->armGetPos(ArmL, 0, jointL);
  ret4 = smartpal->gripperGetPos(ArmR, gripperR);
  ret5 = smartpal->gripperGetPos(ArmL, gripperL);
  ret6 = smartpal->lumbaGetPos(waistL, waistH);

  if ((ret1 + ret2 + ret3 + ret4 + ret5 + ret6) <= 0)
  {
    ROS_ERROR("Failed to get robot position.\n");
    return 0;
  }
  else
    return 1;
}
//------------------------------------------------------------------------------
bool robotControl(tms_msg_rc::smartpal_control::Request &req, tms_msg_rc::smartpal_control::Response &res)
{
  tms_msg_db::TmsdbGetData getRobotData;

  switch (req.unit)
  {
    //--------------------------------------------------------------------------
    case 0:  // all
      switch (req.cmd)
      {
        case 0:  // clearAlarm
          smartpal->vehicleClearAlarm();
          smartpal->armClearAlarm(ArmR);
          smartpal->armClearAlarm(ArmL);
          smartpal->gripperClearAlarm(GripperR);
          smartpal->gripperClearAlarm(GripperL);
          smartpal->lumbaClearAlarm();
          res.result = SUCCESS;  // temp
          break;
        case 1:  // setPower
          smartpal->vehicleSetPower(req.arg[0]);
          smartpal->armSetPower(ArmR, req.arg[0]);
          smartpal->armSetPower(ArmL, req.arg[0]);
          smartpal->gripperSetPower(GripperR, req.arg[0]);
          smartpal->gripperSetPower(GripperL, req.arg[0]);
          smartpal->lumbaSetPower(req.arg[0]);
          res.result = SUCCESS;  // temp
          break;
        case 2:  // setServo
          smartpal->vehicleSetServo(req.arg[0]);
          smartpal->armSetServo(ArmR, req.arg[0]);
          smartpal->armSetServo(ArmL, req.arg[0]);
          smartpal->gripperSetServo(GripperR, req.arg[0]);
          smartpal->gripperSetServo(GripperL, req.arg[0]);
          smartpal->lumbaSetServo(req.arg[0]);
          res.result = SUCCESS;  // temp
          break;
        case 3:  // pause
          smartpal->vehiclePause();
          smartpal->armPause(ArmR);
          smartpal->armPause(ArmL);
          smartpal->gripperPause(GripperR);
          smartpal->gripperPause(GripperL);
          smartpal->lumbaPause();
          res.result = SUCCESS;  // temp
          break;
        case 4:  // resume
          smartpal->vehicleResume();
          smartpal->armResume(ArmR);
          smartpal->armResume(ArmL);
          smartpal->gripperResume(GripperR);
          smartpal->gripperResume(GripperL);
          smartpal->lumbaResume();
          res.result = SUCCESS;  // temp
          break;
        case 5:  // abort
          smartpal->armAbort(ArmR);
          smartpal->armAbort(ArmL);
          smartpal->gripperAbort(GripperR);
          smartpal->gripperAbort(GripperL);
          smartpal->lumbaAbort();
          res.result = SUCCESS;  // temp
          break;
        case 6:  // stop
          smartpal->vehicleStop();
          smartpal->armStop(ArmR);
          smartpal->armStop(ArmL);
          smartpal->gripperStop(GripperR);
          smartpal->gripperStop(GripperL);
          smartpal->lumbaStop();
          res.result = SUCCESS;  // temp
          break;
        case 7:                                      // getViconData
          getRobotData.request.tmsdb.id = 2003;      // SmartPal5_2 ID
          getRobotData.request.tmsdb.sensor = 3001;  // Vicon ID
          if (get_data_client.call(getRobotData))
          {
            ROS_INFO("Get info of object ID: %d\n", getRobotData.request.tmsdb.id);
          }
          else
          {
            ROS_INFO("Failed to call service getRobotData ID: %d\n", getRobotData.request.tmsdb.id);
            break;
          }
          if (!getRobotData.response.tmsdb.empty())
          {
            if (getRobotData.response.tmsdb[0].x != 0 && getRobotData.response.tmsdb[0].y != 0)
            {
              res.result = smartpal->vehicleSetPos(getRobotData.response.tmsdb[0].x, getRobotData.response.tmsdb[0].y,
                                                   getRobotData.response.tmsdb[0].ry);
            }
          }
        case 8:  // move trajectory
        {
          res.result = SUCCESS;
          if (!trajectory.empty())
          {
            for (int t = 0; t < trajectory.size(); t++)
            {
              if (trajectory.at(t).move == 0)
              {
                ROS_INFO("trajectory[%d]:armL [%f,%f,%f,%f,%f,%f,%f]", t, trajectory.at(t).j_L[0],
                         trajectory.at(t).j_L[1], trajectory.at(t).j_L[2], trajectory.at(t).j_L[3],
                         trajectory.at(t).j_L[4], trajectory.at(t).j_L[5], trajectory.at(t).j_L[6]);
              }
              else if (trajectory.at(t).move == 1)
              {
                ROS_INFO("trajectory[%d]:gripperL %f", t, trajectory.at(t).gripper_left);
              }
            }
            for (int t = 0; t < trajectory.size(); t++)
            {
              ROS_INFO("trajectory[%d]", t);
              if (trajectory.at(t).move == 0)
              {  // armL
                double arg[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.175};
                for (int i = 0; i < 7; i++)
                {
                  arg[i] = trajectory.at(t).j_L[i];
                }
                res.result = smartpal->armMoveJointAbs(ArmL, &arg[0], arg[7]);
                // returnのタイミングをロボットの状態がReadyになるまで待機
                ros::Time begin = ros::Time::now();
                while (1)
                {
                  int state = smartpal->armGetState(ArmL);
                  if (state == Busy)
                  {
                    ROS_INFO("armL state : Busy\n");
                    ros::Duration(1.0).sleep();
                  }
                  else if (state == Ready)
                  {
                    ROS_INFO("Succeed to manipulation action(armL).state : Ready\n");
                    res.result = SUCCESS;
                    break;
                  }
                  else
                  {
                    ROS_ERROR("Failed to get collect armL status:%d\n", state);
                    ros::Duration(1.0).sleep();
                    smartpal->armClearAlarm(ArmL);
                  }
                  ros::Time now = ros::Time::now();
                  // Time out
                  if ((now - begin).toSec() >= 60.0)
                  {
                    ROS_ERROR("TIMEOUT!(armL)\n");
                    res.result = FAILURE;
                    break;
                  }
                }
              }
              else if (trajectory.at(t).move == 1)
              {  // gripperL
                double arg[3] = {0.0, 0.175, 0.175};
                arg[0] = trajectory.at(t).gripper_left;
                res.result = smartpal->gripperMoveAbs(ArmL, arg[0], arg[1], arg[2]);
                if (res.result != SUCCESS)
                  ROS_ERROR("Failed gripperMove(armL)\n");
                while (1)
                {
                  if (smartpal->gripperGetState(ArmL))
                  {
                    ROS_INFO("gripperL state : moving");
                    ros::Duration(1.0).sleep();
                  }
                  else
                  {
                    ROS_INFO("gripperL state : ready");
                    break;
                  }
                }
              }

              if (res.result != SUCCESS)
                break;
              else
                ROS_INFO("succeed");
            }
            trajectory.clear();
          }
          break;
        }
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
          res.result = smartpal->vehicleClearAlarm();
          break;
        case 1:
          res.result = smartpal->vehicleSetPower(req.arg[0]);
          break;
        case 2:
          res.result = smartpal->vehicleSetServo(req.arg[0]);
          break;
        case 3:
          res.result = smartpal->vehiclePause();
          break;
        case 4:
          res.result = smartpal->vehicleResume();
          break;
        case 6:
          res.result = smartpal->vehicleStop();
          break;
        case 7:
          res.result = smartpal->vehicleGetState();
          break;
        case 8:
          res.val.resize(3);
          res.result = smartpal->vehicleGetPos(&res.val[0], &res.val[1], &res.val[2]);
          break;
        case 9:
          res.result = smartpal->vehicleSetPos(req.arg[0], req.arg[1], req.arg[2]);
          break;
        case 10:
          res.result = smartpal->vehicleSetVel(req.arg[0], req.arg[1]);
          break;
        case 11:
          res.result = smartpal->vehicleSetAcc(req.arg[0], req.arg[1]);
          break;
        case 15:
        {
          res.result = smartpal->vehicleMoveLinearAbs(req.arg[0], req.arg[1], req.arg[2]);
          // returnのタイミングをロボットの状態がReadyになるまで待機
          ros::Time begin = ros::Time::now();
          while (1)
          {
            int state = smartpal->vehicleGetState();
            if (state == VehicleBusy)
            {
              ROS_INFO("vehicle state : Busy\n");
              ros::Duration(1.0).sleep();
            }
            else if (state == VehicleReady)
            {
              ROS_INFO("Succeed to move vehicle.state : Ready\n");
              res.result = SUCCESS;
              break;
            }
            else
            {
              ROS_ERROR("Failed to get collect vehicle status:%d\n", state);
            }

            ros::Time now = ros::Time::now();
            // Time out
            if ((now - begin).toSec() >= 60.0)
            {
              ROS_ERROR("TIMEOUT!(vehicle)\n");
              res.result = FAILURE;
              break;
            }
          }
          break;
        }
        case 16:
        {
          res.result = smartpal->vehicleMoveLinearRel(req.arg[0], req.arg[1], req.arg[2]);
          // returnのタイミングをロボットの状態がReadyになるまで待機
          ros::Time begin = ros::Time::now();
          while (1)
          {
            int state = smartpal->vehicleGetState();
            if (state == VehicleBusy)
            {
              ROS_INFO("vehicle state : Busy\n");
              ros::Duration(1.0).sleep();
            }
            else if (state == VehicleReady)
            {
              ROS_INFO("Succeed to move vehicle.state : Ready\n");
              res.result = SUCCESS;
              break;
            }
            else
            {
              ROS_ERROR("Failed to get collect vehicle status:%d\n", state);
            }

            ros::Time now = ros::Time::now();
            // Time out
            if ((now - begin).toSec() >= 60.0)
            {
              ROS_ERROR("TIMEOUT!(vehicle)\n");
              res.result = FAILURE;
              break;
            }
          }
          break;
        }
        case 17:
          res.result = smartpal->vehicleMoveCruiseAbs(req.arg[0], req.arg[1]);
          break;
        case 18:
          res.result = smartpal->vehicleMoveCruiseRel(req.arg[0], req.arg[1]);
          break;
        case 19:
          res.result = smartpal->vehicleMoveContinuousRel(req.arg[0], req.arg[1], req.arg[2]);
          break;
        case 20:
          res.result = smartpal->vehicleMoveCircularRel(req.arg[0], req.arg[1], req.arg[2]);
          break;
        case 25:
          res.result = smartpal->vehicleSetJogTimeout(req.arg[0]);
          break;
        case 26:
          res.result = smartpal->vehicleMoveJog(req.arg[0], req.arg[1], req.arg[2]);
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
          res.result = smartpal->armClearAlarm(ArmR);
          break;
        case 1:
          res.result = smartpal->armSetPower(ArmR, req.arg[0]);
          break;
        case 2:
          res.result = smartpal->armSetServo(ArmR, req.arg[0]);
          break;
        case 3:
          res.result = smartpal->armPause(ArmR);
          break;
        case 4:
          res.result = smartpal->armResume(ArmR);
          break;
        case 5:
          res.result = smartpal->armAbort(ArmR);
          break;
        case 6:
          res.result = smartpal->armStop(ArmR);
          break;
        case 7:
          res.result = smartpal->armGetState(ArmR);
          break;
        case 8:
          res.val.resize(7);
          res.result = smartpal->armGetPos(ArmR, req.arg[0], &res.val[0]);
          break;
        case 9:
          res.result = smartpal->armGetActiveAlarm(ArmR, req.arg[0], &res.val[0]);
          break;
        case 10:
          res.result = smartpal->armSetJointAcc(ArmR, req.arg[0]);
          break;
        case 11:
          res.result = smartpal->armSetLinearAcc(ArmR, req.arg[0], req.arg[1]);
          break;
        case 12:
          res.result = smartpal->armIsPowerOn(ArmR);
          break;
        case 13:
          res.result = smartpal->armIsServoOn(ArmR);
          break;
        case 15:
        {
          res.result = smartpal->armMoveJointAbs(ArmR, &req.arg[0], req.arg[7]);
          // returnのタイミングをロボットの状態がReadyになるまで待機
          ros::Time begin = ros::Time::now();
          while (1)
          {
            int state = smartpal->armGetState(ArmR);
            if (state == Busy)
            {
              ROS_INFO("armR state : Busy\n");
              ros::Duration(1.0).sleep();
            }
            else if (state == Ready)
            {
              ROS_INFO("Succeed to manipulation action(armR).state : Ready\n");
              res.result = SUCCESS;
              break;
            }
            else
            {
              ROS_ERROR("Failed to get collect armR status:%d\n", state);
            }
            ros::Time now = ros::Time::now();
            // Time out
            if ((now - begin).toSec() >= 60.0)
            {
              ROS_ERROR("TIMEOUT!(armR)\n");
              res.result = FAILURE;
              break;
            }
          }
          break;
        }
        case 16:
          res.result = smartpal->armMoveJointRel(ArmR, &req.arg[0], req.arg[7]);
          break;
        case 17:
          res.result = smartpal->armMoveLinearAbs(ArmR, req.arg[0], &req.arg[1], req.arg[7], req.arg[8], req.arg[9]);
          break;
        case 18:
          res.result = smartpal->armMoveLinearRel(ArmR, req.arg[0], &req.arg[1], req.arg[7], req.arg[8], req.arg[9]);
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
          res.result = smartpal->armClearAlarm(ArmL);
          break;
        case 1:
          res.result = smartpal->armSetPower(ArmL, req.arg[0]);
          break;
        case 2:
          res.result = smartpal->armSetServo(ArmL, req.arg[0]);
          break;
        case 3:
          res.result = smartpal->armPause(ArmL);
          break;
        case 4:
          res.result = smartpal->armResume(ArmL);
          break;
        case 5:
          res.result = smartpal->armAbort(ArmL);
          break;
        case 6:
          res.result = smartpal->armStop(ArmL);
          break;
        case 7:
          res.result = smartpal->armGetState(ArmL);
          break;
        case 8:
          res.val.resize(7);
          res.result = smartpal->armGetPos(ArmL, req.arg[0], &res.val[0]);
          break;
        case 9:
          res.result = smartpal->armGetActiveAlarm(ArmL, req.arg[0], &res.val[0]);
          break;
        case 10:
          res.result = smartpal->armSetJointAcc(ArmL, req.arg[0]);
          break;
        case 11:
          res.result = smartpal->armSetLinearAcc(ArmL, req.arg[0], req.arg[1]);
          break;
        case 12:
          res.result = smartpal->armIsPowerOn(ArmL);
          break;
        case 13:
          res.result = smartpal->armIsServoOn(ArmL);
          break;
        case 15:
        {
          res.result = smartpal->armMoveJointAbs(ArmL, &req.arg[0], req.arg[7]);
          // returnのタイミングをロボットの状態がReadyになるまで待機
          ros::Time begin = ros::Time::now();
          while (1)
          {
            int state = smartpal->armGetState(ArmL);
            if (state == Busy)
            {
              ROS_INFO("armL state : Busy\n");
              ros::Duration(3.0).sleep();
            }
            else if (state == Ready)
            {
              ROS_INFO("Succeed to manipulation action(armL).state : Ready\n");
              res.result = SUCCESS;
              break;
            }
            else
            {
              ROS_ERROR("Failed to get collect armL status:%d\n", state);
            }
            ros::Time now = ros::Time::now();
            // Time out
            if ((now - begin).toSec() >= 60.0)
            {
              ROS_ERROR("TIMEOUT!(armL)\n");
              res.result = FAILURE;
              break;
            }
          }
          break;
        }
        case 16:
          res.result = smartpal->armMoveJointRel(ArmL, &req.arg[0], req.arg[7]);
          break;
        case 17:
          res.result = smartpal->armMoveLinearAbs(ArmL, req.arg[0], &req.arg[1], req.arg[7], req.arg[8], req.arg[9]);
          break;
        case 18:
          res.result = smartpal->armMoveLinearRel(ArmL, req.arg[0], &req.arg[1], req.arg[7], req.arg[8], req.arg[9]);
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
          res.result = smartpal->gripperClearAlarm(ArmR);
          break;
        case 1:
          res.result = smartpal->gripperSetPower(ArmR, req.arg[0]);
          break;
        case 2:
          res.result = smartpal->gripperSetServo(ArmR, req.arg[0]);
          break;
        case 3:
          res.result = smartpal->gripperPause(ArmR);
          break;
        case 4:
          res.result = smartpal->gripperResume(ArmR);
          break;
        case 5:
          res.result = smartpal->gripperAbort(ArmR);
          break;
        case 6:
          res.result = smartpal->gripperStop(ArmR);
          break;
        case 7:
          res.result = smartpal->gripperGetState(ArmR);
          break;
        case 8:
          res.val.resize(1);
          res.result = smartpal->gripperGetPos(ArmR, &res.val[0]);
          break;
        case 15:
          res.result = smartpal->gripperMoveAbs(ArmR, req.arg[0], req.arg[1], req.arg[2]);
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
          res.result = smartpal->gripperClearAlarm(ArmL);
          break;
        case 1:
          res.result = smartpal->gripperSetPower(ArmL, req.arg[0]);
          break;
        case 2:
          res.result = smartpal->gripperSetServo(ArmL, req.arg[0]);
          break;
        case 3:
          res.result = smartpal->gripperPause(ArmL);
          break;
        case 4:
          res.result = smartpal->gripperResume(ArmL);
          break;
        case 5:
          res.result = smartpal->gripperAbort(ArmL);
          break;
        case 6:
          res.result = smartpal->gripperStop(ArmL);
          break;
        case 7:
          res.result = smartpal->gripperGetState(ArmL);
          break;
        case 8:
          res.val.resize(1);
          res.result = smartpal->gripperGetPos(ArmL, &res.val[0]);
          break;
        case 15:
          res.result = smartpal->gripperMoveAbs(ArmL, req.arg[0], req.arg[1], req.arg[2]);
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
          res.result = smartpal->lumbaClearAlarm();
          break;
        case 1:
          res.result = smartpal->lumbaSetPower(req.arg[0]);
          break;
        case 2:
          res.result = smartpal->lumbaSetServo(req.arg[0]);
          break;
        case 3:
          res.result = smartpal->lumbaPause();
          break;
        case 4:
          res.result = smartpal->lumbaResume();
          break;
        case 5:
          res.result = smartpal->lumbaAbort();
          break;
        case 6:
          res.result = smartpal->lumbaStop();
          break;
        case 7:
          res.result = smartpal->lumbaGetState();
          break;
        case 8:
          res.val.resize(2);
          res.result = smartpal->lumbaGetPos(&res.val[0], &res.val[1]);
          break;
        case 15:
        {
          res.result = smartpal->lumbaMoveCooperative(req.arg[0], req.arg[1], req.arg[2]);
          // returnのタイミングをロボットの状態がReadyになるまで待機
          ros::Time begin = ros::Time::now();
          while (1)
          {
            int state = smartpal->lumbaGetState();
            if (state == Busy)
            {
              ROS_INFO("waist state : Busy\n");
              ros::Duration(3.0).sleep();
            }
            else if (state == Ready)
            {
              ROS_INFO("Succeed to move waist.state : Ready\n");
              res.result = SUCCESS;
              break;
            }
            else
            {
              ROS_ERROR("Failed to get collect waist status:%d\n", state);
            }
            ros::Time now = ros::Time::now();
            // Time out
            if ((now - begin).toSec() >= 60.0)
            {
              ROS_ERROR("TIMEOUT!(waist)\n");
              res.result = FAILURE;
              break;
            }
          }
          break;
        }
        case 16:
          res.result = smartpal->lumbaMove(req.arg[0], req.arg[1], req.arg[2], req.arg[3]);
          break;
        case 17:
          res.result = smartpal->lumbaMoveLowerAxis(req.arg[0], req.arg[1], req.arg[2]);
          break;
        case 18:
          res.result = smartpal->lumbaMoveUpperAxis(req.arg[0], req.arg[1], req.arg[2]);
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

void armCallback(const sensor_msgs::JointState &msg)
{
  armInfo ai;
  if (msg.name[0] == "l_gripper_thumb_joint")
  {
    ai.gripper_left = msg.position[0];
    ai.move = 1;
  }
  else if (msg.name[0] == "l_arm_j1_joint")
  {
    for (int i = 0; i < 7; i++)
    {
      ai.j_L[i] = msg.position[i];
    }
    ai.move = 0;
  }
  trajectory.push_back(ai);
}

void ObjectDataUpdate(const moveit_msgs::PlanningScene &msg)
{
  if (msg.robot_state.attached_collision_objects.size() != 0)
  {  // grasped object
    int object_id = atoi(msg.robot_state.attached_collision_objects[0].object.id.c_str());

    g_oid = object_id;

    geometry_msgs::PoseStamped pose, pose2;
    pose.header.frame_id = "/l_end_effector_link";
    pose.pose.position.x = msg.robot_state.attached_collision_objects[0].object.primitive_poses[0].position.x;
    pose.pose.position.y = msg.robot_state.attached_collision_objects[0].object.primitive_poses[0].position.y;
    pose.pose.position.z = msg.robot_state.attached_collision_objects[0].object.primitive_poses[0].position.z;

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

    tms_msg_db::TmsdbGetData srv;
    srv.request.tmsdb.id = object_id + sid_;

    if (get_data_client.call(srv))
    {
      // g_oz -= srv.response.tmsdb[0].offset_z;

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
      current_pos_data.sensor = 3003;
      current_pos_data.probability = 1.0;
      current_pos_data.state = 2;

      db_msg.tmsdb.push_back(current_pos_data);
      pose_publisher.publish(db_msg);

      is_grasp = true;
      grasping_object_id = g_oid;
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

    if (get_data_client.call(srv))
    {
      // g_oz -= srv.response.tmsdb[0].offset_z;

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
      current_pos_data.sensor = 3003;
      current_pos_data.probability = 1.0;
      current_pos_data.state = 1;

      db_msg.tmsdb.push_back(current_pos_data);
      pose_publisher.publish(db_msg);
    }
    else
    {
      ROS_ERROR("failed to get data");
    }
  }
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //--------------------------------------------------------------------------
  ros::init(argc, argv, "smartpal5_control");
  ros::NodeHandle nh;
  get_data_client = nh.serviceClient< tms_msg_db::TmsdbGetData >("/tms_db_reader");
  ros::ServiceServer service = nh.advertiseService("sp5_control", robotControl);
  pose_publisher = nh.advertise< tms_msg_db::TmsdbStamped >("tms_db_data", 10);

  ros::Subscriber arm_data_sub = nh.subscribe("/move_group/fake_controller_joint_states", 1, &armCallback);
  object_data_sub = nh.subscribe("/move_group/monitored_planning_scene", 1, &ObjectDataUpdate);

  nh.setParam("/2003_is_real", true);

  listener = new tf::TransformListener;

  int32_t id_robot = 2003;                // SmartPal5_2 ID
  int32_t id_odometry_and_joints = 3003;  // Sensor ID
  int32_t id_place = 5002;                // Place ID

  //--------------------------------------------------------------------------
  // smartpal initialize
  uint8_t cnt = 0;
  while (!smartpal->Initialize())
  {
    cnt++;

    printf("CORBA client object initialization has been failed.\n");
    printf("Retry initializing...\n");

    sleep(3);  // delay 3sec

    if (cnt == 5)
    {
      smartpal->Shutdown();
      return (0);
    }
  }
  printf("CORBA client object initialization has been completed.\n\n");

  //--------------------------------------------------------------------------
  // clear alarm
  smartpal->vehicleClearAlarm();
  smartpal->armClearAlarm(ArmR);
  smartpal->armClearAlarm(ArmL);
  smartpal->gripperClearAlarm(GripperR);
  smartpal->gripperClearAlarm(GripperL);
  smartpal->lumbaClearAlarm();

  sleep(2);

  // power on
  smartpal->lumbaSetPower(ON);
  smartpal->armSetPower(ArmR, ON);
  smartpal->armSetPower(ArmL, ON);
  smartpal->vehicleSetPower(ON);

  sleep(2);

  // servo on
  smartpal->vehicleSetServo(ON);
  smartpal->gripperSetServo(GripperR, ON);
  smartpal->gripperSetServo(GripperL, ON);
  smartpal->lumbaSetServo(ON);

  smartpal->armSetServo(ArmR, ON);
  smartpal->armSetServo(ArmL, ON);

  smartpal->armGetSoftLimit(ArmL);

  ros::Time tNow;
  ros::Rate loop_rate(10);  // 10Hz frequency (0.1 sec)

  double x;
  double y;
  double th;
  double gripperR;
  double gripperL;
  double waistL;
  double waistH;
  double jointR[7];
  double jointL[7];

  //--------------------------------------------------------------------------
  while (ros::ok())
  {
    //----------------------------------------------------------------------
    ros::Time now = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9

    tms_msg_db::TmsdbStamped db_msg;
    tms_msg_db::Tmsdb current_pos_data;

    if (getRobotCurrentPos(&x, &y, &th, &waistL, &waistH, jointR, &gripperR, jointL, &gripperL) == 1)
    {
      db_msg.header.frame_id = "/world";
      db_msg.header.stamp = now;

      current_pos_data.time = boost::posix_time::to_iso_extended_string(now.toBoost());
      current_pos_data.id = id_robot;
      current_pos_data.x = x;
      current_pos_data.y = y;
      current_pos_data.z = 0.0;
      current_pos_data.rr = 0.0;
      current_pos_data.rp = 0.0;
      current_pos_data.ry = th;
      current_pos_data.place = id_place;
      current_pos_data.sensor = id_odometry_and_joints;
      current_pos_data.state = 1;

      std::stringstream ss;
      ss << waistH << ";" << waistL << ";";
      for (int i = 0; i < 7; i++)
      {
        ss << jointR[i] << ";";
      }
      ss << gripperR << ";";
      for (int i = 0; i < 7; i++)
      {
        ss << jointL[i] << ";";
      }
      ss << gripperL;

      // joint information of smartpal5
      // lumba_low, lumba_high, jR[0]...[6], gripper_right, jL[0]...[6], gripper_left
      current_pos_data.joint = ss.str();

      std::stringstream ss2;
      ss2 << "grasping=" << grasping_object_id;

      current_pos_data.note = ss2.str();

      db_msg.tmsdb.push_back(current_pos_data);

      pose_publisher.publish(db_msg);
    }
    else
    {
      ROS_ERROR("Failed to getRobotCurrentPos\n");
    }

    //----------------------------------------------------------------------
    ros::spinOnce();
    loop_rate.sleep();

    //----------------------------------------------------------------------
  }

  //--------------------------------------------------------------------------
  // servo off
  smartpal->vehicleSetServo(OFF);
  smartpal->armSetServo(ArmR, OFF);
  smartpal->armSetServo(ArmL, OFF);
  smartpal->gripperSetServo(GripperR, OFF);
  smartpal->gripperSetServo(GripperL, OFF);
  smartpal->lumbaSetServo(OFF);

  sleep(2);

  // power off
  smartpal->vehicleSetPower(OFF);
  smartpal->armSetPower(ArmR, OFF);
  smartpal->armSetPower(ArmL, OFF);
  smartpal->lumbaSetPower(OFF);

  smartpal->Shutdown();

  delete listener;

  return (0);
  //--------------------------------------------------------------------------
}
//------------------------------------------------------------------------------
