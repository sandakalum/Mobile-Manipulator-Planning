#include "ros/ros.h"
#include "../../rps.h"
#include <tms_msg_rp/rps_goal_planning.h>
#include <tms_msg_rp/rps_select_path.h>
#include <tms_msg_rp/rps_voronoi_path_planning.h>
#include <tms_msg_rp/rps_push_wagon_path_planning.h>
#include <tms_msg_rp/rps_give_obj_command.h>
#include <tms_msg_rp/rps_cnoid_grasp_obj_planning.h>
#include <tms_msg_rp/rps_cnoid_grasp_wagon_planning.h>
#include <tms_msg_rp/rps_cnoid_PRM_planning.h>
#include <tms_msg_rp/rps_robot_command.h>

#include <tms_msg_ss/ods_wagon.h>

#include <sstream>

#define SEND_COMMAND
//~ #define USE_VISION

const double open_gripper_angle = -40.0, close_gripper_angle = 0.0;

using namespace std;

ros::ServiceClient client_rc_gop;
tms_msg_rp::rps_cnoid_grasp_obj_planning rc_gop;
ros::ServiceClient client_rc_giop;
tms_msg_rp::rps_cnoid_grasp_obj_planning rc_giop;
ros::ServiceClient client_rc_gwp;
tms_msg_rp::rps_cnoid_grasp_wagon_planning rc_gwp;
ros::ServiceClient client_rc_cgwp;
tms_msg_rp::rps_cnoid_grasp_wagon_planning rc_cgwp;
ros::ServiceClient client_rc_rwp;
tms_msg_rp::rps_cnoid_grasp_wagon_planning rc_rwp;
ros::ServiceClient client_rc_PRM;
tms_msg_rp::rps_cnoid_PRM_planning rc_PRM;
ros::ServiceClient client_robot_command;
tms_msg_rp::rps_robot_command robot_command;

ros::ServiceClient client_ods_wagon;
tms_msg_ss::ods_wagon ods_wagon;

bool start_give_obj_commander(tms_msg_rp::rps_give_obj_command::Request& req,
                              tms_msg_rp::rps_give_obj_command::Response& res)
{
  req.grasp_obj_Robot_path.clear();
  req.give_obj_Robot_path.clear();

  req.grasp_wagon_Robot_joint_angle.clear();
  req.push_wagon_Robot_joint_angle.clear();
  req.release_wagon_Robot_joint_angle.clear();
  req.grasp_obj_Robot_joint_angle.clear();
  req.give_obj_Robot_joint_angle.clear();
  req.return_init_pose_Robot_joint_angle.clear();

  robot_command.request.robot_id = req.robot_id;
  robot_command.request.param_array.clear();

  vector< tms_msg_rp::rps_pos_param > rps_temp_param_array;
  rps_temp_param_array.clear();
  tms_msg_rp::rps_pos_param rps_temp_param;
  rps_temp_param.rps_pos_param.resize(21);

  /////////MOVE TO GRASP WAGON POS//////////
  //////////SEND COMMAND//////////
  robot_command.request.param_array.clear();
  rps_temp_param_array.clear();

  rps_temp_param.rps_pos_param[0] = req.start_Robot_pos.x;
  rps_temp_param.rps_pos_param[1] = req.start_Robot_pos.y;
  rps_temp_param.rps_pos_param[2] = req.start_Robot_pos.th;

  for (int i = 0; i < jointNum; i++)
  {
    rps_temp_param.rps_pos_param[i + 3] = 0.0;
  }
  rps_temp_param.rps_pos_param[12] = rps_temp_param.rps_pos_param[20] = open_gripper_angle;

  rps_temp_param_array.push_back(rps_temp_param);

  for (int i = 0; i < jointNum; i++)
  {
    rps_temp_param.rps_pos_param[i + 3] = sp5_arm_init_pose[i];
  }

  for (unsigned int i = 0; i < req.grasp_wagon_Robot_path.size(); i++)
  {
    rps_temp_param.rps_pos_param[0] = req.grasp_wagon_Robot_path[i].x;
    rps_temp_param.rps_pos_param[1] = req.grasp_wagon_Robot_path[i].y;
    rps_temp_param.rps_pos_param[2] = req.grasp_wagon_Robot_path[i].th;

    rps_temp_param_array.push_back(rps_temp_param);
  }

  robot_command.request.param_array = rps_temp_param_array;

#ifdef SEND_COMMAND
  if (!client_robot_command.call(robot_command))
  {
    ROS_ERROR("Failed to call robot command");
    res.success = 0;
    res.message = "Failed to call robot command";
    return false;
  }
#endif
  ////////////////////////////////
  //////////////////////////////////////////

  /////////GET WAGON POS FROM XTION///////////
  tms_msg_rp::rps_position d_pos, rotate_pos;
  double rotate_th = 0.0;

#ifdef USE_VISION
  ods_wagon.request.robot.x = req.grasp_wagon_Robot_path[req.grasp_wagon_Robot_path.size() - 1].x;
  ods_wagon.request.robot.y = req.grasp_wagon_Robot_path[req.grasp_wagon_Robot_path.size() - 1].y;
  ods_wagon.request.robot.theta = req.grasp_wagon_Robot_path[req.grasp_wagon_Robot_path.size() - 1].th;

  if (!client_ods_wagon.call(ods_wagon))
  {
    ROS_ERROR("Failed to call ods_wagon");
    res.success = 0;
    res.message = "Failed to call ods_wagon";
    return false;
  }

  cout << "ods_Wagon	x:" << ods_wagon.response.wagon.x << "	y:" << ods_wagon.response.wagon.y
       << "	th:" << ods_wagon.response.wagon.theta << endl;

  d_pos.x = req.grasp_wagon_Robot_pos.x - req.start_Wagon_pos.x;
  d_pos.y = req.grasp_wagon_Robot_pos.y - req.start_Wagon_pos.y;

  rotate_th = deg2rad(ods_wagon.response.wagon.theta - req.start_Wagon_pos.th);
  rotate_pos.x = d_pos.x * cos(rotate_th) - d_pos.y * sin(rotate_th);
  rotate_pos.y = d_pos.x * sin(rotate_th) + d_pos.y * cos(rotate_th);

  rotate_pos.x += ods_wagon.response.wagon.x;
  rotate_pos.y += ods_wagon.response.wagon.y;
  rotate_pos.th = req.grasp_wagon_Robot_pos.th + (ods_wagon.response.wagon.theta - req.start_Wagon_pos.th);

////////////////////////////////////////////
#endif

  /////////GRASP WAGON PLANNING///////////
  rc_gwp.request.robot_id = req.robot_id;
  rc_gwp.request.wagon_id = req.wagon_id;

#ifdef USE_VISION
  rc_gwp.request.wagon_pos.x = ods_wagon.response.wagon.x;
  rc_gwp.request.wagon_pos.y = ods_wagon.response.wagon.y;
  rc_gwp.request.wagon_pos.th = ods_wagon.response.wagon.theta;
#else
  rc_gwp.request.wagon_pos = req.start_Wagon_pos;
  //~ rc_gwp.request.wagon_pos.x += -6.0;
  //~ rc_gwp.request.wagon_pos.y += 140.0;
  //~ rc_gwp.request.wagon_pos.th += -4.0;

  d_pos.x = req.grasp_wagon_Robot_pos.x - req.start_Wagon_pos.x;
  d_pos.y = req.grasp_wagon_Robot_pos.y - req.start_Wagon_pos.y;

  rotate_th = deg2rad(rc_gwp.request.wagon_pos.th - req.start_Wagon_pos.th);
  rotate_pos.x = d_pos.x * cos(rotate_th) - d_pos.y * sin(rotate_th);
  rotate_pos.y = d_pos.x * sin(rotate_th) + d_pos.y * cos(rotate_th);

  rotate_pos.x += rc_gwp.request.wagon_pos.x;
  rotate_pos.y += rc_gwp.request.wagon_pos.y;
  rotate_pos.th = req.grasp_wagon_Robot_pos.th + (rc_gwp.request.wagon_pos.th - req.start_Wagon_pos.th);
#endif

  rc_gwp.request.robot_pos = rotate_pos;
  rc_gwp.request.pre_robot_joint_angle.joint_angle.clear();
  for (int i = 0; i < jointNum; i++)
  {
    rc_gwp.request.pre_robot_joint_angle.joint_angle.push_back(sp5_arm_init_pose[i]);
  }

  if (client_rc_gwp.call(rc_gwp))
  {
    if (!rc_gwp.response.success)
    {
      ROS_ERROR("Grasp Wagon is failed : ");
      cout << "	" << rc_gwp.response.message << endl;
      return false;
    }
    if (rc_gwp.response.success)
    {
      cout << "Grasp Wagon Pose Success" << endl;
      //~ for(unsigned int i=0;i<rc_gwp.response.robot_joint_angle.size();i++){
      //~ for(unsigned int j=0;j<rc_gwp.response.robot_joint_angle[i].joint_angle.size();j++){
      //~ cout<<i<<"-"<<j<<":"<<rc_gwp.response.robot_joint_angle[i].joint_angle[j]<<"	";
      //~ }
      //~ cout<<endl;
      //~ }
    }
    // return 1;
  }
  else
  {
    ROS_ERROR("Failed to call rps_cnoid_grasp_wagon_planning");
    cout << rc_gwp.response.message << endl;
    return false;
  }
  ///////////////////////////////////

  /////////PRM PLANNING///////////
  rc_PRM.request.in_motion_state.clear();
  tms_msg_rp::rps_cnoid_wagon_motion_state temp_state;

  temp_state.pushing_wagon_state = 0;
  temp_state.grasping_object_state = 3;

  temp_state.robot_pos = rotate_pos;
  temp_state.robot_pos.yaw = temp_state.robot_pos.th;
#ifdef USE_VISION
  temp_state.wagon_pos.x = ods_wagon.response.wagon.x;
  temp_state.wagon_pos.y = ods_wagon.response.wagon.y;
  temp_state.wagon_pos.th = ods_wagon.response.wagon.theta;
  temp_state.wagon_pos.yaw = temp_state.wagon_pos.th;
#else
  temp_state.wagon_pos = rc_gwp.request.wagon_pos;
  temp_state.wagon_pos.yaw = temp_state.wagon_pos.th;
#endif
  for (unsigned int i = 0; i < rc_gwp.response.robot_joint_angle.size() - 2; i++)
  {
    temp_state.robot_joint_angle = rc_gwp.response.robot_joint_angle[i];
    //~ temp_state.robot_joint_angle.joint_angle[9] = temp_state.robot_joint_angle.joint_angle[17] = open_gripper_angle;
    if (i > 2)
    {
      temp_state.pushing_wagon_state = 1;
      //~ temp_state.robot_joint_angle.joint_angle[9] = temp_state.robot_joint_angle.joint_angle[17] =
      // close_gripper_angle;
    }
    rc_PRM.request.in_motion_state.push_back(temp_state);
  }

  if (client_rc_PRM.call(rc_PRM))
  {
    if (!rc_PRM.response.success)
    {
      ROS_ERROR("Cnoid PRM is failed : ");
      cout << "	" << rc_PRM.response.message << endl;
      return false;
    }
    if (rc_PRM.response.success)
    {
      cout << "PRM Success" << endl;
      for (unsigned int i = 0; i < rc_PRM.response.out_motion_state.size(); i++)
      {
        req.grasp_wagon_Robot_joint_angle.push_back(rc_PRM.response.out_motion_state[i].robot_joint_angle);
        //~ for(unsigned int j=0;j<rc_PRM.response.out_motion_state[i].robot_joint_angle.joint_angle.size();j++){
        //~ cout<<i<<"-"<<j<<":"<<rc_PRM.response.out_motion_state[i].robot_joint_angle.joint_angle[j]<<"	";
        //~ }
        //~ cout<<endl;
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to call rps_cnoid_PRM_planning");
    cout << rc_PRM.response.message << endl;
    return false;
  }

  //////////SEND COMMAND(ADJUST GRASP WAGON POS)//////////
  robot_command.request.param_array.clear();
  rps_temp_param_array.clear();

  rps_temp_param_array.push_back(rps_temp_param);

  rps_temp_param.rps_pos_param[0] = req.grasp_wagon_Robot_pos.x;
  rps_temp_param.rps_pos_param[1] = req.grasp_wagon_Robot_pos.y;
  rps_temp_param.rps_pos_param[2] = req.grasp_wagon_Robot_pos.th;

  for (int i = 0; i < jointNum; i++)
  {
    rps_temp_param.rps_pos_param[i + 3] = sp5_arm_init_pose[i];
  }
  rps_temp_param_array.push_back(rps_temp_param);

  rps_temp_param.rps_pos_param[0] = rotate_pos.x;
  rps_temp_param.rps_pos_param[1] = rotate_pos.y;
  rps_temp_param.rps_pos_param[2] = rotate_pos.th;
  rps_temp_param_array.push_back(rps_temp_param);

  robot_command.request.param_array = rps_temp_param_array;

#ifdef SEND_COMMAND
  if (!client_robot_command.call(robot_command))
  {
    ROS_ERROR("Failed to call robot command");
    res.success = 0;
    res.message = "Failed to call robot command";
    return false;
  }
#endif
  ////////////////////////////////

  //////////SEND COMMAND//////////
  robot_command.request.param_array.clear();
  rps_temp_param_array.clear();

  rps_temp_param_array.push_back(rps_temp_param);

  rps_temp_param.rps_pos_param[0] = rc_gwp.request.robot_pos.x;
  rps_temp_param.rps_pos_param[1] = rc_gwp.request.robot_pos.y;
  rps_temp_param.rps_pos_param[2] = rc_gwp.request.robot_pos.th;

  for (unsigned int i = 0; i < req.grasp_wagon_Robot_joint_angle.size(); i++)
  {
    for (unsigned int j = 0; j < req.grasp_wagon_Robot_joint_angle[i].joint_angle.size(); j++)
    {
      rps_temp_param.rps_pos_param[j + 3] = req.grasp_wagon_Robot_joint_angle[i].joint_angle[j];
    }
    rps_temp_param.rps_pos_param[12] = rps_temp_param.rps_pos_param[20] = open_gripper_angle;
    //~ if(i>4)rps_temp_param.rps_pos_param[12] = rps_temp_param.rps_pos_param[20] = close_gripper_angle;
    rps_temp_param_array.push_back(rps_temp_param);
  }
  rps_temp_param.rps_pos_param[12] = rps_temp_param.rps_pos_param[20] = close_gripper_angle;
  rps_temp_param_array.push_back(rps_temp_param);

  robot_command.request.param_array = rps_temp_param_array;

#ifdef SEND_COMMAND
  if (!client_robot_command.call(robot_command))
  {
    ROS_ERROR("Failed to call robot command");
    res.success = 0;
    res.message = "Failed to call robot command";
    return false;
  }
#endif
  ////////////////////////////////
  ///////////////////////////////////

  //////////SEND COMMAND(MOVE TO GRASP WAGON POS)//////////
  robot_command.request.param_array.clear();
  rps_temp_param_array.clear();

  rps_temp_param_array.push_back(rps_temp_param);

  rps_temp_param.rps_pos_param[0] = rc_gwp.request.robot_pos.x;
  rps_temp_param.rps_pos_param[1] = rc_gwp.request.robot_pos.y;
  rps_temp_param.rps_pos_param[2] = rc_gwp.request.robot_pos.th;
  rps_temp_param_array.push_back(rps_temp_param);

  rps_temp_param.rps_pos_param[0] = req.grasp_wagon_Robot_pos.x;
  rps_temp_param.rps_pos_param[1] = req.grasp_wagon_Robot_pos.y;
  rps_temp_param.rps_pos_param[2] = req.grasp_wagon_Robot_pos.th;
  rps_temp_param_array.push_back(rps_temp_param);

  robot_command.request.param_array = rps_temp_param_array;

#ifdef SEND_COMMAND
  if (!client_robot_command.call(robot_command))
  {
    ROS_ERROR("Failed to call robot command");
    res.success = 0;
    res.message = "Failed to call robot command";
    return false;
  }
#endif
  ////////////////////////////////

  /////////PUSH WAGON PATH JOINT ANGLE PLANNING///////////
  for (unsigned int i = 0; i < req.push_wagon_Robot_path.size(); i++)
  {
    rc_cgwp.request.robot_pos = req.push_wagon_Robot_path[i];
    rc_cgwp.request.wagon_pos = req.push_wagon_Wagon_path[i];

    if (client_rc_cgwp.call(rc_cgwp))
    {
      req.push_wagon_Robot_joint_angle.push_back(rc_cgwp.response.robot_joint_angle[0]);
    }
    else
    {
      ROS_ERROR("Failed to call rps_cnoid_calc_grasp_wagon_pose");
      cout << rc_PRM.response.message << endl;
      return false;
    }
  }

  //////////SEND COMMAND//////////
  robot_command.request.param_array.clear();
  rps_temp_param_array.clear();

  rps_temp_param_array.push_back(rps_temp_param);

  for (unsigned int i = 0; i < req.push_wagon_Robot_path.size(); i++)
  {
    rps_temp_param.rps_pos_param[0] = req.push_wagon_Robot_path[i].x;
    rps_temp_param.rps_pos_param[1] = req.push_wagon_Robot_path[i].y;
    rps_temp_param.rps_pos_param[2] = req.push_wagon_Robot_path[i].th;
    for (unsigned int j = 0; j < req.push_wagon_Robot_joint_angle[i].joint_angle.size(); j++)
    {
      rps_temp_param.rps_pos_param[j + 3] = req.push_wagon_Robot_joint_angle[i].joint_angle[j];
    }
    rps_temp_param.rps_pos_param[12] = rps_temp_param.rps_pos_param[20] = close_gripper_angle;
    rps_temp_param_array.push_back(rps_temp_param);
  }

  robot_command.request.param_array = rps_temp_param_array;

#ifdef SEND_COMMAND
  if (!client_robot_command.call(robot_command))
  {
    ROS_ERROR("Failed to call robot command");
    res.success = 0;
    res.message = "Failed to call robot command";
    return false;
  }
#endif
  ////////////////////////////////
  ////////////////////////////////////////////////////////

  /////////RELEASE WAGON PLANNING///////////
  rc_rwp.request.robot_id = req.robot_id;
  rc_rwp.request.wagon_id = req.wagon_id;

  rc_rwp.request.robot_pos = req.push_wagon_Robot_path[req.push_wagon_Robot_path.size() - 1];
  rc_rwp.request.pre_robot_joint_angle = req.push_wagon_Robot_joint_angle[req.push_wagon_Robot_joint_angle.size() - 1];
  rc_rwp.request.wagon_pos = req.push_wagon_Wagon_path[req.push_wagon_Wagon_path.size() - 1];

  if (client_rc_rwp.call(rc_rwp))
  {
    if (!rc_rwp.response.success)
    {
      ROS_ERROR("Release Wagon is failed : ");
      cout << "	" << rc_rwp.response.message << endl;
      return false;
    }
    if (rc_rwp.response.success)
    {
      cout << "Release Wagon Pose Success" << endl;
      //~ for(unsigned int i=0;i<rc_rwp.response.robot_joint_angle.size();i++){
      //~ for(unsigned int j=0;j<rc_rwp.response.robot_joint_angle[i].joint_angle.size();j++){
      //~ cout<<i<<"-"<<j<<":"<<rc_rwp.response.robot_joint_angle[i].joint_angle[j]<<"	";
      //~ }
      //~ cout<<endl;
      //~ }
    }
    // return 1;
  }
  else
  {
    ROS_ERROR("Failed to call rps_cnoid_release_wagon_planning");
    cout << rc_rwp.response.message << endl;
    return false;
  }
  ///////////////////////////////////

  /////////PRM PLANNING///////////
  rc_PRM.request.in_motion_state.clear();

  temp_state.pushing_wagon_state = 0;
  temp_state.grasping_object_state = 3;

  temp_state.robot_pos = req.push_wagon_Robot_path[req.push_wagon_Robot_path.size() - 1];
  temp_state.robot_pos.yaw = temp_state.robot_pos.th;
  temp_state.wagon_pos = req.push_wagon_Wagon_path[req.push_wagon_Wagon_path.size() - 1];
  temp_state.wagon_pos.yaw = temp_state.wagon_pos.th;
  for (unsigned int i = 0; i < rc_rwp.response.robot_joint_angle.size(); i++)
  {
    temp_state.robot_joint_angle = rc_rwp.response.robot_joint_angle[i];
    rc_PRM.request.in_motion_state.push_back(temp_state);
  }

  if (client_rc_PRM.call(rc_PRM))
  {
    if (!rc_PRM.response.success)
    {
      ROS_ERROR("Cnoid PRM is failed : ");
      cout << "	" << rc_PRM.response.message << endl;
      return false;
    }
    if (rc_PRM.response.success)
    {
      cout << "PRM Success" << endl;
      for (unsigned int i = 0; i < rc_PRM.response.out_motion_state.size(); i++)
      {
        req.release_wagon_Robot_joint_angle.push_back(rc_PRM.response.out_motion_state[i].robot_joint_angle);
        //~ for(unsigned int j=0;j<rc_PRM.response.out_motion_state[i].robot_joint_angle.joint_angle.size();j++){
        //~ cout<<i<<"-"<<j<<":"<<rc_PRM.response.out_motion_state[i].robot_joint_angle.joint_angle[j]<<"	";
        //~ }
        //~ cout<<endl;
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to call rps_cnoid_PRM_planning");
    cout << rc_PRM.response.message << endl;
    return false;
  }

  //////////SEND COMMAND//////////
  robot_command.request.param_array.clear();
  rps_temp_param_array.clear();

  rps_temp_param_array.push_back(rps_temp_param);

  rps_temp_param.rps_pos_param[0] = temp_state.robot_pos.x;
  rps_temp_param.rps_pos_param[1] = temp_state.robot_pos.y;
  rps_temp_param.rps_pos_param[2] = temp_state.robot_pos.th;

  for (unsigned int i = 0; i < req.release_wagon_Robot_joint_angle.size(); i++)
  {
    for (unsigned int j = 0; j < req.release_wagon_Robot_joint_angle[i].joint_angle.size(); j++)
    {
      rps_temp_param.rps_pos_param[j + 3] = req.release_wagon_Robot_joint_angle[i].joint_angle[j];
    }
    //~ rps_temp_param.rps_pos_param[12] = rps_temp_param.rps_pos_param[20] = open_gripper_angle;
    rps_temp_param_array.push_back(rps_temp_param);
  }

  robot_command.request.param_array = rps_temp_param_array;

#ifdef SEND_COMMAND
  if (!client_robot_command.call(robot_command))
  {
    ROS_ERROR("Failed to call robot command");
    res.success = 0;
    res.message = "Failed to call robot command";
    return false;
  }
#endif
  ////////////////////////////////
  ///////////////////////////////////

  /////////GRASP OBJECT PLANNING///////////
  rc_gop.request.robot_id = req.robot_id;
  rc_gop.request.obj_id = req.object_id;

  rc_gop.request.robot_pos = req.push_wagon_Robot_path[req.push_wagon_Robot_path.size() - 1];
  rc_gop.request.pre_robot_joint_angle =
      req.release_wagon_Robot_joint_angle[req.release_wagon_Robot_joint_angle.size() - 1];

  if (client_rc_gop.call(rc_gop))
  {
    if (!rc_gop.response.success)
    {
      ROS_ERROR("Grasp Obj is failed : ");
      cout << "	" << rc_gop.response.message << endl;
      return false;
    }
    if (rc_gop.response.success)
    {
      cout << "Grasp Obj Pose Success" << endl;
      //~ for(unsigned int i=0;i<rc_gop.response.robot_joint_angle.size();i++){
      //~ for(unsigned int j=0;j<rc_gop.response.robot_joint_angle[i].joint_angle.size();j++){
      //~ cout<<i<<"-"<<j<<":"<<rc_gop.response.robot_joint_angle[i].joint_angle[j]<<"	";
      //~ }
      //~ cout<<endl;
      //~ }
    }
    // return 1;
  }
  else
  {
    ROS_ERROR("Failed to call rps_cnoid_grasp_obj_planning");
    cout << rc_gop.response.message << endl;
    return false;
  }
  ///////////////////////////////////

  /////////PRM PLANNING///////////
  rc_PRM.request.in_motion_state.clear();

  temp_state.pushing_wagon_state = 0;
  temp_state.grasping_object_state = 3;

  temp_state.robot_pos = req.push_wagon_Robot_path[req.push_wagon_Robot_path.size() - 1];
  temp_state.robot_pos.yaw = temp_state.robot_pos.th;
  temp_state.wagon_pos = req.push_wagon_Wagon_path[req.push_wagon_Wagon_path.size() - 1];
  temp_state.wagon_pos.yaw = temp_state.wagon_pos.th;
  temp_state.robot_joint_angle = rc_gop.response.robot_joint_angle[0];
  temp_state.robot_joint_angle.joint_angle[9] = open_gripper_angle;
  rc_PRM.request.in_motion_state.push_back(temp_state);

  temp_state.robot_joint_angle.joint_angle.clear();
  for (int i = 0; i < jointNum; i++)
  {
    temp_state.robot_joint_angle.joint_angle.push_back(sp5_arm_give_pre_pose[i]);
  }
  temp_state.robot_joint_angle.joint_angle[9] = open_gripper_angle;
  rc_PRM.request.in_motion_state.push_back(temp_state);

  for (unsigned int i = 1; i < rc_gop.response.robot_joint_angle.size() - 1; i++)
  {
    temp_state.robot_joint_angle = rc_gop.response.robot_joint_angle[i];
    temp_state.robot_joint_angle.joint_angle[9] = open_gripper_angle;
    rc_PRM.request.in_motion_state.push_back(temp_state);
  }

  if (client_rc_PRM.call(rc_PRM))
  {
    if (!rc_PRM.response.success)
    {
      ROS_ERROR("Cnoid PRM is failed : ");
      cout << "	" << rc_PRM.response.message << endl;
      return false;
    }
    if (rc_PRM.response.success)
    {
      cout << "PRM Success" << endl;
      for (unsigned int i = 0; i < rc_PRM.response.out_motion_state.size(); i++)
      {
        req.grasp_obj_Robot_joint_angle.push_back(rc_PRM.response.out_motion_state[i].robot_joint_angle);
        //~ for(unsigned int j=0;j<rc_PRM.response.out_motion_state[i].robot_joint_angle.joint_angle.size();j++){
        //~ cout<<i<<"-"<<j<<":"<<rc_PRM.response.out_motion_state[i].robot_joint_angle.joint_angle[j]<<"	";
        //~ }
        //~ cout<<endl;
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to call rps_cnoid_PRM_planning");
    cout << rc_PRM.response.message << endl;
    return false;
  }

  //////////SEND COMMAND//////////
  robot_command.request.param_array.clear();
  rps_temp_param_array.clear();

  rps_temp_param_array.push_back(rps_temp_param);

  rps_temp_param.rps_pos_param[0] = temp_state.robot_pos.x;
  rps_temp_param.rps_pos_param[1] = temp_state.robot_pos.y;
  rps_temp_param.rps_pos_param[2] = temp_state.robot_pos.th;

  for (unsigned int i = 0; i < req.grasp_obj_Robot_joint_angle.size(); i++)
  {
    for (unsigned int j = 0; j < req.grasp_obj_Robot_joint_angle[i].joint_angle.size(); j++)
    {
      rps_temp_param.rps_pos_param[j + 3] = req.grasp_obj_Robot_joint_angle[i].joint_angle[j];
    }
    //~ rps_temp_param.rps_pos_param[12] = open_gripper_angle;
    //~ rps_temp_param.rps_pos_param[20] = close_gripper_angle;
    //~ if(i>0)rps_temp_param.rps_pos_param[12] = open_gripper_angle-10.0;
    rps_temp_param_array.push_back(rps_temp_param);
  }
  rps_temp_param.rps_pos_param[12] =
      req.grasp_obj_Robot_joint_angle[req.grasp_obj_Robot_joint_angle.size() - 1].joint_angle[9];
  rps_temp_param_array.push_back(rps_temp_param);

  robot_command.request.param_array = rps_temp_param_array;

#ifdef SEND_COMMAND
  if (!client_robot_command.call(robot_command))
  {
    ROS_ERROR("Failed to call robot command");
    res.success = 0;
    res.message = "Failed to call robot command";
    return false;
  }
#endif
  ////////////////////////////////
  ///////////////////////////////////

  /////////GIVE OBJECT PLANNING///////////
  rc_giop.request.robot_id = req.robot_id;
  rc_giop.request.obj_id = req.object_id;

  rc_giop.request.robot_pos = req.give_obj_Robot_pos;
  rc_giop.request.pre_robot_joint_angle = req.grasp_obj_Robot_joint_angle[req.grasp_obj_Robot_joint_angle.size() - 1];
  rc_giop.request.obj_pos = req.give_obj_Obj_pos;

  if (client_rc_giop.call(rc_giop))
  {
    if (!rc_giop.response.success)
    {
      ROS_ERROR("Give Obj is failed : ");
      cout << "	" << rc_giop.response.message << endl;
      return false;
    }
    if (rc_giop.response.success)
    {
      cout << "Give Obj Pose Success" << endl;
      //~ for(unsigned int i=0;i<rc_giop.response.robot_joint_angle.size();i++){
      //~ for(unsigned int j=0;j<rc_giop.response.robot_joint_angle[i].joint_angle.size();j++){
      //~ cout<<i<<"-"<<j<<":"<<rc_giop.response.robot_joint_angle[i].joint_angle[j]<<"	";
      //~ }
      //~ cout<<endl;
      //~ }
      // return 1;
    }
  }
  else
  {
    ROS_ERROR("Failed to call rps_cnoid_give_obj_planning");
    cout << rc_giop.response.message << endl;
    return false;
  }
  ///////////////////////////////////

  /////////PRM PLANNING///////////
  rc_PRM.request.in_motion_state.clear();

  temp_state.pushing_wagon_state = 0;
  temp_state.grasping_object_state = 2;

  // grasp object (on wagon) pose
  temp_state.robot_pos = req.push_wagon_Robot_path[req.push_wagon_Robot_path.size() - 1];
  temp_state.robot_pos.yaw = temp_state.robot_pos.th;
  temp_state.wagon_pos = req.push_wagon_Wagon_path[req.push_wagon_Wagon_path.size() - 1];
  temp_state.wagon_pos.yaw = temp_state.wagon_pos.th;
  temp_state.robot_joint_angle = rc_giop.response.robot_joint_angle[0];
  rc_PRM.request.in_motion_state.push_back(temp_state);

  temp_state.robot_joint_angle = rc_gop.response.robot_joint_angle[rc_gop.response.robot_joint_angle.size() - 1];
  temp_state.robot_joint_angle.joint_angle[9] =
      rc_gop.response.robot_joint_angle[rc_gop.response.robot_joint_angle.size() - 1].joint_angle[9];
  rc_PRM.request.in_motion_state.push_back(temp_state);

  temp_state.robot_joint_angle.joint_angle.clear();
  for (int i = 0; i < jointNum; i++)
  {
    temp_state.robot_joint_angle.joint_angle.push_back(sp5_arm_give_pre_pose[i]);
  }
  temp_state.robot_joint_angle.joint_angle[9] =
      rc_gop.response.robot_joint_angle[rc_gop.response.robot_joint_angle.size() - 1].joint_angle[9];
  rc_PRM.request.in_motion_state.push_back(temp_state);

  temp_state.robot_pos.th = req.give_obj_Robot_pos.th;
  temp_state.robot_pos.yaw = temp_state.robot_pos.th;
  rc_PRM.request.in_motion_state.push_back(temp_state);

  temp_state.robot_joint_angle = rc_giop.response.robot_joint_angle[1];
  temp_state.robot_joint_angle.joint_angle[0] = temp_state.robot_joint_angle.joint_angle[1] = 0.0;
  rc_PRM.request.in_motion_state.push_back(temp_state);

  temp_state.robot_joint_angle = rc_giop.response.robot_joint_angle[1];
  rc_PRM.request.in_motion_state.push_back(temp_state);

  if (client_rc_PRM.call(rc_PRM))
  {
    if (!rc_PRM.response.success)
    {
      ROS_ERROR("Cnoid PRM is failed : ");
      cout << "	" << rc_PRM.response.message << endl;
      return false;
    }
    if (rc_PRM.response.success)
    {
      cout << "PRM Success" << endl;
      for (unsigned int i = 0; i < rc_PRM.response.out_motion_state.size(); i++)
      {
        req.give_obj_Robot_path.push_back(rc_PRM.response.out_motion_state[i].robot_pos);
        req.give_obj_Robot_joint_angle.push_back(rc_PRM.response.out_motion_state[i].robot_joint_angle);
        //~ for(unsigned int j=0;j<rc_PRM.response.out_motion_state[i].robot_joint_angle.joint_angle.size();j++){
        //~ cout<<i<<"-"<<j<<":"<<rc_PRM.response.out_motion_state[i].robot_joint_angle.joint_angle[j]<<"	";
        //~ }
        //~ cout<<endl;
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to call rps_cnoid_PRM_planning");
    cout << rc_PRM.response.message << endl;
    return false;
  }

  //////////SEND COMMAND//////////
  robot_command.request.param_array.clear();
  rps_temp_param_array.clear();

  rps_temp_param_array.push_back(rps_temp_param);

  for (unsigned int i = 0; i < req.give_obj_Robot_joint_angle.size(); i++)
  {
    rps_temp_param.rps_pos_param[0] = req.give_obj_Robot_path[i].x;
    rps_temp_param.rps_pos_param[1] = req.give_obj_Robot_path[i].y;
    rps_temp_param.rps_pos_param[2] = req.give_obj_Robot_path[i].th;
    for (unsigned int j = 0; j < req.give_obj_Robot_joint_angle[i].joint_angle.size(); j++)
    {
      rps_temp_param.rps_pos_param[j + 3] = req.give_obj_Robot_joint_angle[i].joint_angle[j];
    }
    //~ rps_temp_param.rps_pos_param[12] =
    // rc_gop.response.robot_joint_angle[rc_gop.response.robot_joint_angle.size()-1].joint_angle[9];
    rps_temp_param_array.push_back(rps_temp_param);
  }
  rps_temp_param.rps_pos_param[12] = open_gripper_angle;
  rps_temp_param_array.push_back(rps_temp_param);

  robot_command.request.param_array = rps_temp_param_array;

#ifdef SEND_COMMAND
  if (!client_robot_command.call(robot_command))
  {
    ROS_ERROR("Failed to call robot command");
    res.success = 0;
    res.message = "Failed to call robot command";
    return false;
  }
#endif
  ////////////////////////////////
  ///////////////////////////////////

  /////////RETURN INIT POSE//////////////////////////
  /////////PRM PLANNING///////////
  rc_PRM.request.in_motion_state.clear();

  temp_state.pushing_wagon_state = 0;
  temp_state.grasping_object_state = 0;

  temp_state.robot_pos = req.give_obj_Robot_pos;
  temp_state.robot_pos.yaw = temp_state.robot_pos.th;
  temp_state.wagon_pos = req.push_wagon_Wagon_path[req.push_wagon_Wagon_path.size() - 1];
  temp_state.wagon_pos.yaw = temp_state.wagon_pos.th;
  temp_state.object_pos = req.give_obj_Obj_pos;
  temp_state.object_pos.yaw = temp_state.object_pos.th;
  temp_state.robot_joint_angle = req.give_obj_Robot_joint_angle[req.give_obj_Robot_joint_angle.size() - 1];
  temp_state.robot_joint_angle.joint_angle[9] = open_gripper_angle;
  rc_PRM.request.in_motion_state.push_back(temp_state);

  temp_state.robot_joint_angle.joint_angle[0] = temp_state.robot_joint_angle.joint_angle[1] = 0.0;

  rc_PRM.request.in_motion_state.push_back(temp_state);

  temp_state.robot_joint_angle.joint_angle.clear();
  for (int i = 0; i < jointNum; i++)
  {
    temp_state.robot_joint_angle.joint_angle.push_back(sp5_arm_give_pre_pose[i]);
  }
  //~ temp_state.robot_joint_angle.joint_angle[9] = close_gripper_angle;
  rc_PRM.request.in_motion_state.push_back(temp_state);

  temp_state.robot_joint_angle.joint_angle.clear();
  for (int i = 0; i < jointNum; i++)
  {
    temp_state.robot_joint_angle.joint_angle.push_back(sp5_arm_init_pose[i]);
  }
  rc_PRM.request.in_motion_state.push_back(temp_state);

  if (client_rc_PRM.call(rc_PRM))
  {
    if (!rc_PRM.response.success)
    {
      ROS_ERROR("Cnoid PRM is failed : ");
      cout << "	" << rc_PRM.response.message << endl;
      return false;
    }
    if (rc_PRM.response.success)
    {
      cout << "PRM Success" << endl;
      for (unsigned int i = 0; i < rc_PRM.response.out_motion_state.size(); i++)
      {
        req.return_init_pose_Robot_joint_angle.push_back(rc_PRM.response.out_motion_state[i].robot_joint_angle);
        //~ for(unsigned int j=0;j<rc_PRM.response.out_motion_state[i].robot_joint_angle.joint_angle.size();j++){
        //~ cout<<i<<"-"<<j<<":"<<rc_PRM.response.out_motion_state[i].robot_joint_angle.joint_angle[j]<<"	";
        //~ }
        //~ cout<<endl;
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to call rps_cnoid_PRM_planning");
    cout << rc_PRM.response.message << endl;
    return false;
  }

  //////////SEND COMMAND//////////
  robot_command.request.param_array.clear();
  rps_temp_param_array.clear();

  rps_temp_param_array.push_back(rps_temp_param);

  rps_temp_param.rps_pos_param[0] = req.give_obj_Robot_pos.x;
  rps_temp_param.rps_pos_param[1] = req.give_obj_Robot_pos.y;
  rps_temp_param.rps_pos_param[2] = req.give_obj_Robot_pos.th;

  for (unsigned int i = 0; i < req.return_init_pose_Robot_joint_angle.size(); i++)
  {
    for (unsigned int j = 0; j < req.return_init_pose_Robot_joint_angle[i].joint_angle.size(); j++)
    {
      rps_temp_param.rps_pos_param[j + 3] = req.return_init_pose_Robot_joint_angle[i].joint_angle[j];
    }
    rps_temp_param_array.push_back(rps_temp_param);
  }
  rps_temp_param_array.push_back(rps_temp_param);

  robot_command.request.param_array = rps_temp_param_array;

#ifdef SEND_COMMAND
  if (!client_robot_command.call(robot_command))
  {
    ROS_ERROR("Failed to call robot command");
    res.success = 0;
    res.message = "Failed to call robot command";
    return false;
  }
#endif
  ////////////////////////////////
  ///////////////////////////////////

  ROS_INFO("FINISH");

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rps_give_obj_commander");
  ros::NodeHandle n;

  //~ ros::Subscriber	rps_map_subscriber = n.subscribe("rps_map_data", 1, set_RPS_MAP);
  ros::ServiceServer server_give_obj_command = n.advertiseService("rps_give_obj_command", start_give_obj_commander);

  client_rc_gop = n.serviceClient< tms_msg_rp::rps_cnoid_grasp_obj_planning >("rps_cnoid_grasp_obj_planning");
  client_rc_giop = n.serviceClient< tms_msg_rp::rps_cnoid_grasp_obj_planning >("rps_cnoid_give_obj_planning");
  client_rc_gwp = n.serviceClient< tms_msg_rp::rps_cnoid_grasp_wagon_planning >("rps_cnoid_grasp_wagon_planning");
  client_rc_cgwp = n.serviceClient< tms_msg_rp::rps_cnoid_grasp_wagon_planning >("rps_cnoid_calc_grasp_wagon_pose");
  client_rc_rwp = n.serviceClient< tms_msg_rp::rps_cnoid_grasp_wagon_planning >("rps_cnoid_release_wagon_planning");
  client_rc_PRM = n.serviceClient< tms_msg_rp::rps_cnoid_PRM_planning >("rps_cnoid_PRM_planning");

  client_ods_wagon = n.serviceClient< tms_msg_ss::ods_wagon >("ods_wagon");

  client_robot_command = n.serviceClient< tms_msg_rp::rps_robot_command >("rps_robot_command");

  ros::spin();

  return 0;
}
