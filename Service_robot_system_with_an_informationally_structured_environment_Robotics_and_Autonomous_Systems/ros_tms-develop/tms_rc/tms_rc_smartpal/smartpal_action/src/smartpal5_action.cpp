#define CNOID_BACKWARD_COMPATIBILITY

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <tms_msg_rc/smartpal_control.h>
#include <tms_msg_rp/action.h>
#include <tms_msg_rp/rps_path_planning.h>

#include <boost/bind.hpp>

#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/LazyCaller>

#include <Grasp/PlanBase.h>
#include <Grasp/PlanInterface.h>
#include <Grasp/VectorMath.h>
#include <PRM/TrajectoryPlanner.h>

#include <Grasp/GraspController.h>

#include <boost/thread.hpp>

#define rad2deg(x) ((x) * (180.0) / M_PI)
#define deg2rad(x) ((x)*M_PI / 180.0)

#define UNIT_ALL 0
#define UNIT_VEHICLE 1
#define UNIT_ARM_R 2
#define UNIT_ARM_L 3
#define UNIT_GRIPPER_R 4
#define UNIT_GRIPPER_L 5
#define UNIT_LUMBA 6
#define UNIT_CC 7

#define CMD_CLEARALARM 0
#define CMD_SETPOWER 1
#define CMD_SETSERVO 2
#define CMD_PAUSE 3
#define CMD_RESUME 4
#define CMD_ABORT 5
#define CMD_STOP 6
#define CMD_GETSTATE 7
#define CMD_GETPOSE 8
#define CMD_MOVE_ABS 15
#define CMD_MOVE_REL 16

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

int argc;
char **argv;
ros::ServiceClient sp5_control_client;
ros::ServiceClient path_planning_client;

bool action(tms_msg_rp::action::Request &req, tms_msg_rp::action::Response &res)
{
  tms_msg_rc::smartpal_control sp_control_srv;

  //----------------------------------------------------------------------
  sp_control_srv.request.unit = UNIT_ARM_R;
  sp_control_srv.request.cmd = CMD_MOVE_ABS;
  sp_control_srv.request.arg.resize(8);
  sp_control_srv.request.arg[0] = 0;
  sp_control_srv.request.arg[1] = -10;
  sp_control_srv.request.arg[2] = 0;
  sp_control_srv.request.arg[3] = 0;
  sp_control_srv.request.arg[4] = 0;
  sp_control_srv.request.arg[5] = 0;
  sp_control_srv.request.arg[6] = 0;
  sp_control_srv.request.arg[7] = 10;

  if (sp5_control_client.call(sp_control_srv))
    ROS_INFO("result: %d", sp_control_srv.response.result);
  else
    ROS_ERROR("Failed to call service sp5_control");

  sp_control_srv.request.unit = UNIT_ARM_L;
  sp_control_srv.request.cmd = CMD_MOVE_ABS;
  sp_control_srv.request.arg.resize(8);
  sp_control_srv.request.arg[0] = 0;
  sp_control_srv.request.arg[1] = -10;
  sp_control_srv.request.arg[2] = 0;
  sp_control_srv.request.arg[3] = 0;
  sp_control_srv.request.arg[4] = 0;
  sp_control_srv.request.arg[5] = 0;
  sp_control_srv.request.arg[6] = 0;
  sp_control_srv.request.arg[7] = 10;

  if (sp5_control_client.call(sp_control_srv))
    ROS_INFO("result: %d", sp_control_srv.response.result);
  else
    ROS_ERROR("Failed to call service sp5_control");

  cout << "PRM start" << endl;
  if (!PlanBase::instance()->targetArmFinger)
  {
    MessageView::mainInstance()->putln("error: Set Robot");
  }
  cout << "check setRobot" << endl;

  cnoid::Link *rl = PlanBase::instance()->targetArmFinger->bodyItemRobot->body()->rootLink();
  cout << "make linkt" << endl;

  MessageView::mainInstance()->putln("run SmartpalAction!");

  PlanBase *pb = PlanBase::instance();
  // setrobot
  pb->setTrajectoryPlanDOF();
  // setobject

  for (int i = 0; i < 18; i++)
  {
    pb->bodyItemRobot()->body()->joint(i)->q() = 0;
  }
  pb->bodyItemRobot()->body()->joint(3)->q() = deg2rad(-10.0);
  pb->bodyItemRobot()->body()->joint(11)->q() = deg2rad(10.0);

  pb->initial();
  pb->graspMotionSeq.clear();

  pb->setGraspingState(PlanBase::NOT_GRASPING);
  pb->setObjectContactState(PlanBase::ON_ENVIRONMENT);
  pb->graspMotionSeq.push_back(pb->getMotionState());

  bool success = GraspController::instance()->loadAndSelectGraspPattern();
  if (!success)
  {
    cout << "Error: Cannot find grasping posure" << endl;
  }

  MotionState graspMotionState = pb->getMotionState();

  Vector3 Pp_(pb->palm()->p());
  Matrix3 Rp_(pb->palm()->R());

  //==== Approach Point
  pb->setMotionState(graspMotionState);
  pb->arm()->IK_arm(Rp_ * pb->arm()->approachOffset + Pp_ + Vector3(0, 0, 0.05), Rp_);
  pb->setGraspingState(PlanBase::UNDER_GRASPING);
  for (int i = 0; i < pb->nFing(); i++)
  {
    for (int j = 0; j < pb->fingers(i)->fing_path->numJoints(); j++)
    {
      pb->fingers(i)->fing_path->joint(j)->q() = pb->fingers(i)->fingerOpenPose[j];
    }
  }
  pb->graspMotionSeq.push_back(pb->getMotionState());

  //==== Grasp Point and Hand open
  pb->setMotionState(graspMotionState);
  pb->setGraspingState(PlanBase::UNDER_GRASPING);
  for (int i = 0; i < pb->nFing(); i++)
  {
    for (int j = 0; j < pb->fingers(i)->fing_path->numJoints(); j++)
    {
      pb->fingers(i)->fing_path->joint(j)->q() = pb->fingers(i)->fingerOpenPose[j];
    }
  }
  pb->graspMotionSeq.push_back(pb->getMotionState());

  //==== hand close
  pb->setMotionState(graspMotionState);
  pb->setGraspingState(PlanBase::GRASPING);
  pb->graspMotionSeq.push_back(pb->getMotionState());

  //==== lift up
  pb->arm()->IK_arm(Vector3(Vector3(0, 0, 0.05) + Pp_), Rp_);

  pb->setObjectContactState(PlanBase::OFF_ENVIRONMENT);
  pb->graspMotionSeq.push_back(pb->getMotionState());

  //==== end point
  vector< double > closefinger;
  for (int i = 0; i < pb->nFing(); i++)
  {
    for (int j = 0; j < pb->fingers(i)->fing_path->numJoints(); j++)
    {
      closefinger.push_back(pb->fingers(i)->fing_path->joint(j)->q());
    }
  }

  for (int i = 0; i < 18; i++)
  {
    pb->bodyItemRobot()->body()->joint(i)->q() = 0;
  }
  pb->bodyItemRobot()->body()->joint(3)->q() = deg2rad(-10.0);
  pb->bodyItemRobot()->body()->joint(11)->q() = deg2rad(10.0);

  int cnt = 0;
  for (int i = 0; i < pb->nFing(); i++)
  {
    for (int j = 0; j < pb->fingers(i)->fing_path->numJoints(); j++)
    {
      pb->fingers(i)->fing_path->joint(j)->q() = closefinger[cnt++];
    }
  }
  pb->graspMotionSeq.push_back(pb->getMotionState());

  TrajectoryPlanner tp;
  success = tp.doTrajectoryPlanning();

  if (!success)
  {
    cout << "Error: Cannot find motion path" << endl;
  }

  res.result = (float)success;

  double jonit[18];

  for (int i = 0; i < tp.motionSeq.size(); i++)
  {
    for (int j = 0; j < tp.motionSeq[i].jointSeq.size(); j++)
    {
      jonit[j] = rad2deg(tp.motionSeq[i].jointSeq(j));
      cout << "jonit[" << j << "] : " << jonit[j] << endl;
    }

    sp_control_srv.request.unit = UNIT_ARM_R;
    sp_control_srv.request.cmd = CMD_MOVE_ABS;
    sp_control_srv.request.arg.resize(8);
    sp_control_srv.request.arg[0] = jonit[2];
    sp_control_srv.request.arg[1] = jonit[3];
    sp_control_srv.request.arg[2] = jonit[4];
    sp_control_srv.request.arg[3] = jonit[5];
    sp_control_srv.request.arg[4] = jonit[6];
    sp_control_srv.request.arg[5] = jonit[7];
    sp_control_srv.request.arg[6] = jonit[8];
    sp_control_srv.request.arg[7] = 10;

    if (sp5_control_client.call(sp_control_srv))
      ROS_INFO("result: %d", sp_control_srv.response.result);
    else
      ROS_ERROR("Failed to call service sp5_control");

    //----------------------------------------------------------------------------
    sp_control_srv.request.unit = UNIT_LUMBA;
    sp_control_srv.request.cmd = 16;
    sp_control_srv.request.arg.resize(4);
    sp_control_srv.request.arg[0] = jonit[0];
    sp_control_srv.request.arg[1] = jonit[1];
    sp_control_srv.request.arg[2] = 10;
    sp_control_srv.request.arg[3] = 10;

    if (sp5_control_client.call(sp_control_srv))
      ROS_INFO("result: %d", sp_control_srv.response.result);
    else
      ROS_ERROR("Failed to call service sp5_control");

    //----------------------------------------------------------------------------
    sp_control_srv.request.unit = UNIT_GRIPPER_R;
    sp_control_srv.request.cmd = CMD_MOVE_ABS;
    sp_control_srv.request.arg.resize(3);
    sp_control_srv.request.arg[0] = jonit[9];
    sp_control_srv.request.arg[1] = 10;
    sp_control_srv.request.arg[2] = 10;

    if (sp5_control_client.call(sp_control_srv))
      ROS_INFO("result: %d", sp_control_srv.response.result);
    else
      ROS_ERROR("Failed to call service sp5_control");

    sleep(3);
  }

  return true;
}

class SmartpalActionPlugin : public Plugin
{
public:
  void LoopRosTest(void)
  {
    //----------------------------------------------------------------------
    tms_msg_rp::rps_path_planning rplsrv;

    rplsrv.request.robot_id = 1;
    rplsrv.request.rps_goal_candidate.rps_route.resize(1);
    rplsrv.request.rps_goal_candidate.rps_route[0].x = 3000;
    rplsrv.request.rps_goal_candidate.rps_route[0].y = 1000;
    rplsrv.request.rps_goal_candidate.rps_route[0].th = 0;

    if (path_planning_client.call(rplsrv))
    {
      ROS_INFO("result: %d message: %s", rplsrv.response.success, rplsrv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call service rps_path_planning");
    }

    if (rplsrv.response.rps_path.size() != 0)
    {
      for (int i = 0; i < rplsrv.response.rps_path[0].rps_route.size(); i++)
      {
        ROS_INFO("[%d]: x=%f, y=%f, th=%f", i, rplsrv.response.rps_path[0].rps_route[i].x,
                 rplsrv.response.rps_path[0].rps_route[i].y, rplsrv.response.rps_path[0].rps_route[i].th);
      }
    }
    else
    {
      ROS_ERROR("Noting rps_path_planning data");
    }

    static ros::Rate loop_rate(10);  // 0.1sec
    while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  //----------------------------------------------------------------------------
  void OnRosInterfaceActivated()
  {
    cout << "SmartPal action node init for choreonoid" << endl;
    if (!is_ros_init)
    {
      ros::init(argc, argv, "smartpal_action");
      is_ros_init = true;
    }

    //----------------------------------------------------------------------
    cout << "set ros srv" << endl;
    static ros::NodeHandle nh;
    static ros::ServiceServer action_server = nh.advertiseService("rp_action_srv", action);
    sp5_control_client = nh.serviceClient< tms_msg_rc::smartpal_control >("sp5_control");
    path_planning_client = nh.serviceClient< tms_msg_rp::rps_path_planning >("rps_path_planning");

    static boost::thread t(boost::bind(&SmartpalActionPlugin::LoopRosTest, this));

    MessageView::mainInstance()->putln("stop SmartpalAction!");
  }

public:
  SmartpalActionPlugin() : Plugin("SmartpalAction")
  {
  }

  bool is_ros_init;

  virtual bool initialize()
  {
    is_ros_init = false;
    menuManager()
        .setPath("/Tools")
        .addItem("SmartpalAction")
        ->sigTriggered()
        .connect(bind(&SmartpalActionPlugin::OnRosInterfaceActivated, this));

    return true;
  }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(SmartpalActionPlugin)
