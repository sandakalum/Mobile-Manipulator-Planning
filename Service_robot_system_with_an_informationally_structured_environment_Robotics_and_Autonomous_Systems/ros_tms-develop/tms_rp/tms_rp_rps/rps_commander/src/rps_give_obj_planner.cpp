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
#include <tms_msg_db/tmsdb_get_robots_info.h>
#include <tms_msg_db/tmsdb_get_movable_furnitures_info.h>

#define USE_TMS_DB

//~ #define deg2rad(x)	((x)*M_PI/180.0)
//~ #define rad2deg(x)	((x)*180.0/M_PI)

//~ const double sp4_arm_init_pose[16] = {		//degree
//~ 0.0, -4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, /*right arm*/
//~ 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 /*left arm*/};	//degree
//~ const double sp5_arm_init_pose[18] = {		//degree
//~ 0.0, 0.0,	/*waist*/
//~ 0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, /*right arm*/
//~ 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 /*left arm*/};	//degree

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rps_give_obj_plan_commander");
  ros::NodeHandle n;

  ros::ServiceClient commander_to_get_robots_info =
      n.serviceClient< tms_msg_db::tmsdb_get_robots_info >("tmsdb_get_robots_info");
  tms_msg_db::tmsdb_get_robots_info srv_get_r_info;
  ros::ServiceClient commander_to_get_movable_furnitures_info =
      n.serviceClient< tms_msg_db::tmsdb_get_movable_furnitures_info >("tmsdb_get_movable_furnitures_info");
  tms_msg_db::tmsdb_get_movable_furnitures_info srv_get_f_info;

  ros::ServiceClient client_gwpp = n.serviceClient< tms_msg_rp::rps_goal_planning >("rps_grasp_wagon_pos_planning");
  tms_msg_rp::rps_goal_planning rgwp;
  ros::ServiceClient client_gopp = n.serviceClient< tms_msg_rp::rps_goal_planning >("rps_give_obj_pos_planning");
  tms_msg_rp::rps_goal_planning rgop;
  ros::ServiceClient client_vpp = n.serviceClient< tms_msg_rp::rps_voronoi_path_planning >("rps_voronoi_path_planning");
  tms_msg_rp::rps_voronoi_path_planning rvpp;
  ros::ServiceClient client_pwpp =
      n.serviceClient< tms_msg_rp::rps_push_wagon_path_planning >("rps_push_wagon_path_planning");
  tms_msg_rp::rps_push_wagon_path_planning rpwp;
  ros::ServiceClient client_sp = n.serviceClient< tms_msg_rp::rps_select_path >("rps_select_path");
  tms_msg_rp::rps_select_path rsp;

  ros::ServiceClient client_rc_goc = n.serviceClient< tms_msg_rp::rps_give_obj_command >("rps_give_obj_command");
  tms_msg_rp::rps_give_obj_command rc_goc;
  ros::ServiceClient client_rc_gop =
      n.serviceClient< tms_msg_rp::rps_cnoid_grasp_obj_planning >("rps_cnoid_grasp_obj_planning");
  tms_msg_rp::rps_cnoid_grasp_obj_planning rc_gop;
  ros::ServiceClient client_rc_giop =
      n.serviceClient< tms_msg_rp::rps_cnoid_grasp_obj_planning >("rps_cnoid_give_obj_planning");
  tms_msg_rp::rps_cnoid_grasp_obj_planning rc_giop;
  ros::ServiceClient client_rc_gwp =
      n.serviceClient< tms_msg_rp::rps_cnoid_grasp_wagon_planning >("rps_cnoid_grasp_wagon_planning");
  tms_msg_rp::rps_cnoid_grasp_wagon_planning rc_gwp;
  ros::ServiceClient client_rc_cgwp =
      n.serviceClient< tms_msg_rp::rps_cnoid_grasp_wagon_planning >("rps_cnoid_calc_grasp_wagon_pose");
  tms_msg_rp::rps_cnoid_grasp_wagon_planning rc_cgwp;
  ros::ServiceClient client_rc_rwp =
      n.serviceClient< tms_msg_rp::rps_cnoid_grasp_wagon_planning >("rps_cnoid_release_wagon_planning");
  tms_msg_rp::rps_cnoid_grasp_wagon_planning rc_rwp;
  ros::ServiceClient client_rc_PRM = n.serviceClient< tms_msg_rp::rps_cnoid_PRM_planning >("rps_cnoid_PRM_planning");
  tms_msg_rp::rps_cnoid_PRM_planning rc_PRM;

  vector< tms_msg_rp::rps_route > grasp_wagon_path, push_wagon_path;
  grasp_wagon_path.clear();
  push_wagon_path.clear();

  int robot_id = 2;
  int person_id = 31;
  int wagon_id = 22;
  int object_id = 51;

  tms_msg_rp::rps_position robot_pos, wagon_pos;
  srv_get_r_info.request.robots_id = robot_id;
#ifdef USE_TMS_DB
  if (commander_to_get_robots_info.call(srv_get_r_info))
  {
    ROS_INFO("Success robots_x = %lf, y = %lf, theta = %lf", srv_get_r_info.response.robots_x,
             srv_get_r_info.response.robots_y, srv_get_r_info.response.robots_theta);
    robot_pos.x = srv_get_r_info.response.robots_x;
    robot_pos.y = srv_get_r_info.response.robots_y;
    robot_pos.th = srv_get_r_info.response.robots_theta;
    robot_pos.yaw = srv_get_r_info.response.robots_theta;
  }
  else
  {
    ROS_ERROR("get robot info is failed");
    return 0;
  }

  srv_get_f_info.request.furnitures_id = wagon_id;
  if (commander_to_get_movable_furnitures_info.call(srv_get_f_info))
  {
    ROS_INFO("Success wagon_x = %lf, y = %lf, th = %lf, width = %lf, depth = %lf, height = %lf",
             srv_get_f_info.response.furniture_x, srv_get_f_info.response.furniture_y,
             srv_get_f_info.response.furnitures_theta, srv_get_f_info.response.furnitures_width,
             srv_get_f_info.response.furnitures_depth, srv_get_f_info.response.furnitures_height);
    wagon_pos.x = srv_get_f_info.response.furniture_x;
    wagon_pos.y = srv_get_f_info.response.furniture_y;
    wagon_pos.th = srv_get_f_info.response.furnitures_theta;
    wagon_pos.yaw = srv_get_f_info.response.furnitures_theta;
  }
  else
  {
    ROS_ERROR("Failed to call service get_furnitures_info");
    return 0;
  }
#else
  robot_pos.x = 1000;
  robot_pos.y = 1000;
  robot_pos.th = 0;
  robot_pos.yaw = robot_pos.th;

  wagon_pos.x = 3000;
  wagon_pos.y = 1000;
  wagon_pos.th = 0;
  wagon_pos.yaw = robot_pos.th;
#endif

  /////////PUSH WAGON PATH DATA INITIAL/////////
  PushWagonPathData planning_path_list;
  planning_path_list.robot_id = robot_id;
  planning_path_list.wagon_id = wagon_id;
  planning_path_list.person_id = person_id;
  planning_path_list.object_id = object_id;

  planning_path_list.start_Robot_pos = robot_pos;
  planning_path_list.start_Wagon_pos = wagon_pos;

  planning_path_list.grasp_wagon_Robot_pos.clear();
  planning_path_list.give_obj_Robot_pos.clear();
  planning_path_list.give_obj_Robot_joint_angle.clear();

  planning_path_list.grasp_wagon_Robot_path.clear();
  planning_path_list.push_wagon_Robot_path.clear();
  planning_path_list.push_wagon_Wagon_path.clear();

  vector< vector< tms_msg_rp::rps_position > > temp_push_wagon_Robot_path, temp_push_wagon_Wagon_path;
  temp_push_wagon_Robot_path.clear();
  temp_push_wagon_Wagon_path.clear();
  ////////////////////////////////////////////

  /////////SELECT PATH DATA INITIAL/////////
  rsp.request.robot_id = robot_id;
  rsp.request.person_id = person_id;
  rsp.request.in_path.clear();
  rsp.request.weight_Length = 1.0;
  rsp.request.weight_Rotation = 1.0;
  rsp.request.weight_View = 1.0;
  ////////////////////////////////////////////

  /////////GRASP WAGON POS PLANNING///////////
  rgwp.request.robot_id = robot_id;
  rgwp.request.target_id = wagon_id;

  if (client_gwpp.call(rgwp))
  {
    if (!rgwp.response.success)
    {
      ROS_ERROR("Grap Wagon Pos is not found : ");
      cout << "	" << rgwp.response.message << endl;
    }
    if ((rgwp.response.success) && (rgwp.response.goal_pos.size() != 0))
    {
      for (int i = 0; i < rgwp.response.goal_pos.size(); i++)
      {
        cout << resetiosflags(ios_base::floatfield) << "GOAL." << i << "	x:" << setw(4) << setprecision(4)
             << rgwp.response.goal_pos[i].x << "	y:" << setw(4) << setprecision(4) << rgwp.response.goal_pos[i].y
             << "	th:" << fixed << setprecision(1) << rgwp.response.goal_pos[i].th << endl;
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to call goal grasp wagon pos planning");
    cout << rgwp.response.message << endl;
  }

  planning_path_list.grasp_wagon_Robot_pos = rgwp.response.goal_pos;
  ///////////////////////////////////

  /////////VORONOI PATH PLANNING///////////
  grasp_wagon_path.resize(rgwp.response.goal_pos.size());

  for (unsigned int n = 0; n < rgwp.response.goal_pos.size(); n++)
  {
    rvpp.request.robot_id = robot_id;
    rvpp.request.start_pos = robot_pos;
    rvpp.request.goal_pos = rgwp.response.goal_pos[n];
    if (client_vpp.call(rvpp))
    {
      if (!rvpp.response.success)
      {
        ROS_ERROR("Voronoi Path is not found : ");
        cout << "	" << rvpp.response.message << endl;
      }
      if ((rvpp.response.success) && (rvpp.response.VoronoiPath.size() != 0))
      {
        cout << "Path." << n << endl;
        for (int i = 0; i < rvpp.response.VoronoiPath.size(); i++)
        {
          grasp_wagon_path[n].rps_route.push_back(rvpp.response.VoronoiPath[i]);

          //~ cout<< resetiosflags(ios_base::floatfield) <<"Point."<<setw(3)<<i<<"
          // x:"<<setw(4)<<setprecision(4)<<rvpp.response.VoronoiPath[i].x<<"
          // y:"<<setw(4)<<setprecision(4)<<rvpp.response.VoronoiPath[i].y<<"
          // th:"<<fixed<<setprecision(1)<<rvpp.response.VoronoiPath[i].th<<endl;
        }
      }
    }
    else
    {
      ROS_ERROR("Failed to call voronoi path planning");
    }

    planning_path_list.grasp_wagon_Robot_path.push_back(rvpp.response.VoronoiPath);
  }
  ///////////////////////////////////

  /////////GIVE OBJ POS PLANNING///////////
  rgop.request.robot_id = robot_id;
  rgop.request.target_id = person_id;

  if (client_gopp.call(rgop))
  {
    if (!rgop.response.success)
    {
      ROS_ERROR("Give Obj Pos is not found : ");
      cout << "	" << rgop.response.message << endl;
    }
    if ((rgop.response.success) && (rgop.response.goal_pos.size() != 0))
    {
      for (int i = 0; i < rgop.response.goal_pos.size(); i++)
      {
        cout << resetiosflags(ios_base::floatfield) << "GOAL." << i << "	x:" << setw(4) << setprecision(4)
             << rgop.response.goal_pos[i].x << "	y:" << setw(4) << setprecision(4) << rgop.response.goal_pos[i].y
             << "	th:" << fixed << setprecision(1) << rgop.response.goal_pos[i].th << endl;
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to call goal give obj pos planning");
  }

  planning_path_list.give_obj_Obj_pos = rgop.response.target_pos;
  planning_path_list.give_obj_Robot_pos = rgop.response.goal_pos;
  planning_path_list.give_obj_Robot_joint_angle = rgop.response.joint_angle_array;
  ///////////////////////////////////

  /////////PUSH WAGON PATH PLANNING///////////

  //~ push_wagon_path.resize(rgop.response.goal_pos.size());
  rsp.request.in_path.resize(grasp_wagon_path.size() * rgop.response.goal_pos.size());
  int k = 0;
  for (unsigned int n = 0; n < rgwp.response.goal_pos.size(); n++)
  {
    for (unsigned int m = 0; m < rgop.response.goal_pos.size(); m++)
    {
      rsp.request.in_path[k] = grasp_wagon_path[n];
      k++;
    }
  }
  k = 0;

  for (unsigned int n = 0; n < rgwp.response.goal_pos.size(); n++)
  {
    temp_push_wagon_Robot_path.clear();
    temp_push_wagon_Wagon_path.clear();
    for (unsigned int m = 0; m < rgop.response.goal_pos.size(); m++)
    {
      rpwp.request.robot_id = robot_id;
      rpwp.request.wagon_id = 22;
      rpwp.request.start_robot_pos = rgwp.response.goal_pos[n];
      rpwp.request.start_wagon_pos = wagon_pos;
      rpwp.request.goal_robot_pos = rgop.response.goal_pos[m];
      if (client_pwpp.call(rpwp))
      {
        if (!rpwp.response.success)
        {
          ROS_ERROR("Push Wagon Path is not found : ");
          std::cout << "	" << rpwp.response.message << std::endl;
        }
        if ((rpwp.response.success) && (rpwp.response.RobotPath.size() != 0))
        {
          cout << "Path." << n << "-" << m << endl;
          for (int i = 0; i < rpwp.response.RobotPath.size(); i++)
          {
            rsp.request.in_path[k].rps_route.push_back(rpwp.response.RobotPath[i]);
            //~ std::cout<< resetiosflags(ios_base::floatfield) <<"Point."<<setw(3)<<i<<"
            // x:"<<setw(4)<<setprecision(4)<<rpwp.response.RobotPath[i].x<<"
            // y:"<<setw(4)<<setprecision(4)<<rpwp.response.RobotPath[i].y<<"
            // th:"<<fixed<<setprecision(1)<<rpwp.response.RobotPath[i].th<<std::endl;
          }
          k++;
        }
      }
      else
      {
        ROS_ERROR("Failed to call push wagon path planning");
      }

      temp_push_wagon_Robot_path.push_back(rpwp.response.RobotPath);
      temp_push_wagon_Wagon_path.push_back(rpwp.response.WagonPath);
    }

    planning_path_list.push_wagon_Robot_path.push_back(temp_push_wagon_Robot_path);
    planning_path_list.push_wagon_Wagon_path.push_back(temp_push_wagon_Wagon_path);
  }
  k = 0;
  ///////////////////////////////////

  /////////SELECT PATH///////////
  if (client_sp.call(rsp))
  {
    //~ for(unsigned int n=0;n<rsp.response.out_path.size();n++){
    //~ for(unsigned int m=0;m<rsp.response.out_path[n].rps_route.size();m++){
    //~ std::cout<< resetiosflags(ios_base::floatfield) <<"Point."<<setw(3)<<m<<"
    // x:"<<setw(4)<<setprecision(4)<<rsp.response.out_path[n].rps_route[m].x<<"
    // y:"<<setw(4)<<setprecision(4)<<rsp.response.out_path[n].rps_route[m].y<<"
    // th:"<<fixed<<setprecision(1)<<rsp.response.out_path[n].rps_route[m].th<<std::endl;
    //~ }
    //~ }
  }
  else
  {
    ROS_ERROR("Failed to call select path");
    return 0;
  }
  //~ return 0;
  ///////////////////////////////////

  /////////CALC SELECT PATH NO//////////
  int grasp_wagon_pos_No, give_obj_pos_No;

  grasp_wagon_pos_No = rsp.response.out_path_rank[0] / planning_path_list.give_obj_Robot_pos.size();
  give_obj_pos_No = rsp.response.out_path_rank[0] % planning_path_list.give_obj_Robot_pos.size();

  cout << "Select Path:" << grasp_wagon_pos_No << "-" << give_obj_pos_No << endl;

  //~ for(int i=0;i<rsp.response.out_path_rank.size();i++){
  //~ grasp_wagon_pos_No = rsp.response.out_path_rank[i]/planning_path_list.give_obj_Robot_pos.size();
  //~ give_obj_pos_No = rsp.response.out_path_rank[i]%planning_path_list.give_obj_Robot_pos.size();
  //~
  //~ cout<<"Select Path:"<<grasp_wagon_pos_No<<"-"<<give_obj_pos_No<<endl;
  //~ }
  //////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /////////SET COMMAND PATH//////////
  rc_goc.request.robot_id = planning_path_list.robot_id;
  rc_goc.request.wagon_id = planning_path_list.wagon_id;
  rc_goc.request.person_id = planning_path_list.person_id;
  rc_goc.request.object_id = planning_path_list.object_id;

  rc_goc.request.start_Robot_pos = planning_path_list.start_Robot_pos;
  rc_goc.request.start_Wagon_pos = planning_path_list.start_Wagon_pos;
  rc_goc.request.give_obj_Obj_pos = planning_path_list.give_obj_Obj_pos;
  rc_goc.request.grasp_wagon_Robot_pos = planning_path_list.grasp_wagon_Robot_pos[grasp_wagon_pos_No];
  rc_goc.request.give_obj_Robot_pos = planning_path_list.give_obj_Robot_pos[give_obj_pos_No];

  rc_goc.request.grasp_wagon_Robot_path = planning_path_list.grasp_wagon_Robot_path[grasp_wagon_pos_No];
  rc_goc.request.push_wagon_Robot_path = planning_path_list.push_wagon_Robot_path[grasp_wagon_pos_No][give_obj_pos_No];
  rc_goc.request.push_wagon_Wagon_path = planning_path_list.push_wagon_Wagon_path[grasp_wagon_pos_No][give_obj_pos_No];

  rc_goc.request.grasp_obj_Robot_path.clear();
  rc_goc.request.give_obj_Robot_path.clear();

  rc_goc.request.grasp_wagon_Robot_joint_angle.clear();
  rc_goc.request.push_wagon_Robot_joint_angle.clear();
  rc_goc.request.release_wagon_Robot_joint_angle.clear();
  rc_goc.request.grasp_obj_Robot_joint_angle.clear();
  rc_goc.request.give_obj_Robot_joint_angle.clear();
  ///////////////////////////////////

  /////////SEND COMMAND PATH//////////
  if (client_rc_goc.call(rc_goc))
  {
    ROS_INFO("Success to call send command");
    return 1;
  }
  else
  {
    ROS_ERROR("Failed to call send command");
    return 0;
  }
  ////////////////////////////////////

  /*
  /////////SET EXECUTE PATH//////////
    ExecutePushWagonPath execute_path;
    execute_path.robot_id = planning_path_list.robot_id;
    execute_path.wagon_id = planning_path_list.wagon_id;
    execute_path.person_id = planning_path_list.person_id;
    execute_path.object_id = planning_path_list.object_id;

    execute_path.start_Robot_pos = planning_path_list.start_Robot_pos;
    execute_path.start_Wagon_pos = planning_path_list.start_Wagon_pos;
    execute_path.give_obj_Obj_pos = planning_path_list.give_obj_Obj_pos;
    execute_path.grasp_wagon_Robot_pos = planning_path_list.grasp_wagon_Robot_pos[grasp_wagon_pos_No];
    execute_path.give_obj_Robot_pos = planning_path_list.give_obj_Robot_pos[give_obj_pos_No];

    execute_path.grasp_wagon_Robot_path = planning_path_list.grasp_wagon_Robot_path[grasp_wagon_pos_No];
    execute_path.push_wagon_Robot_path = planning_path_list.push_wagon_Robot_path[grasp_wagon_pos_No][give_obj_pos_No];
    execute_path.push_wagon_Wagon_path = planning_path_list.push_wagon_Wagon_path[grasp_wagon_pos_No][give_obj_pos_No];

    execute_path.grasp_wagon_Robot_joint_angle.clear();
    execute_path.push_wagon_Robot_joint_angle.clear();
    execute_path.release_wagon_Robot_joint_angle.clear();
    execute_path.grasp_obj_Robot_joint_angle.clear();
    execute_path.give_obj_Robot_joint_angle.clear();
  ///////////////////////////////////

  /////////GRASP WAGON PLANNING///////////
    rc_gwp.request.robot_id = execute_path.robot_id;
    rc_gwp.request.wagon_id = execute_path.wagon_id;

    rc_gwp.request.robot_pos = execute_path.grasp_wagon_Robot_pos;
    rc_gwp.request.pre_robot_joint_angle.joint_angle.clear();
    for(int i=0;i<jointNum;i++){
      rc_gwp.request.pre_robot_joint_angle.joint_angle.push_back(sp5_arm_init_pose[i]);
    }
    rc_gwp.request.wagon_pos = execute_path.start_Wagon_pos;

    if(client_rc_gwp.call(rc_gwp))
    {
      if(!rc_gwp.response.success){
        ROS_ERROR("Grasp Wagon is failed : ");
        cout<<"	"<<rc_gwp.response.message<<endl;
        return 1;
      }
      if(rc_gwp.response.success){
        cout<<"Grasp Wagon Pose Success"<<endl;
        //~ for(unsigned int i=0;i<rc_gwp.response.robot_joint_angle.size();i++){
          //~ for(unsigned int j=0;j<rc_gwp.response.robot_joint_angle[i].joint_angle.size();j++){
            //~ cout<<i<<"-"<<j<<":"<<rc_gwp.response.robot_joint_angle[i].joint_angle[j]<<"	";
          //~ }
          //~ cout<<endl;
        //~ }
      }
      //return 1;
    }
    else
    {
      ROS_ERROR("Failed to call rps_cnoid_grasp_wagon_planning");
      cout<<rc_gwp.response.message<<endl;
      return 1;
    }
  ///////////////////////////////////

  /////////PRM PLANNING///////////
    rc_PRM.request.in_motion_state.clear();
    tms_msg_rp::rps_cnoid_wagon_motion_state temp_state;

    temp_state.pushing_wagon_state = 0;
    temp_state.grasping_object_state = 3;

    temp_state.robot_pos = execute_path.grasp_wagon_Robot_pos;
    temp_state.robot_pos.yaw = temp_state.robot_pos.th;
    temp_state.wagon_pos = execute_path.start_Wagon_pos;
    temp_state.wagon_pos.yaw = temp_state.wagon_pos.th;
    for(unsigned int i=0;i<rc_gwp.response.robot_joint_angle.size();i++){
      temp_state.robot_joint_angle = rc_gwp.response.robot_joint_angle[i];
      rc_PRM.request.in_motion_state.push_back(temp_state);
    }

    if (client_rc_PRM.call(rc_PRM))
    {
      if(!rc_PRM.response.success){
        ROS_ERROR("Cnoid PRM is failed : ");
        cout<<"	"<<rc_PRM.response.message<<endl;
        return 1;
      }
      if(rc_PRM.response.success){
        cout<<"PRM Success"<<endl;
        for(unsigned int i=0;i<rc_PRM.response.out_motion_state.size();i++){
          execute_path.grasp_wagon_Robot_joint_angle.push_back(rc_PRM.response.out_motion_state[i].robot_joint_angle);
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
      cout<<rc_PRM.response.message<<endl;
      return 1;
    }
  ///////////////////////////////////

  /////////PUSH WAGON PATH JOINT ANGLE PLANNING///////////
    for(unsigned int i=0;i<execute_path.push_wagon_Robot_path.size();i++){
      rc_cgwp.request.robot_pos = execute_path.push_wagon_Robot_path[i];
      rc_cgwp.request.wagon_pos = execute_path.push_wagon_Wagon_path[i];

      if(client_rc_cgwp.call(rc_cgwp)){
        execute_path.push_wagon_Robot_joint_angle.push_back(rc_cgwp.response.robot_joint_angle[0]);
      }
      else{
        ROS_ERROR("Failed to call rps_cnoid_calc_grasp_wagon_pose");
        cout<<rc_PRM.response.message<<endl;
        return 1;
      }
    }
  ////////////////////////////////////////////////////////

  /////////RELEASE WAGON PLANNING///////////
    rc_rwp.request.robot_id = execute_path.robot_id;
    rc_rwp.request.wagon_id = execute_path.wagon_id;

    rc_rwp.request.robot_pos = execute_path.push_wagon_Robot_path[execute_path.push_wagon_Robot_path.size()-1];
    rc_rwp.request.pre_robot_joint_angle =
  execute_path.push_wagon_Robot_joint_angle[execute_path.push_wagon_Robot_joint_angle.size()-1];
    rc_rwp.request.wagon_pos = execute_path.push_wagon_Wagon_path[execute_path.push_wagon_Wagon_path.size()-1];

    if (client_rc_rwp.call(rc_rwp))
    {
      if(!rc_rwp.response.success){
        ROS_ERROR("Release Wagon is failed : ");
        cout<<"	"<<rc_rwp.response.message<<endl;
        return 1;
      }
      if(rc_rwp.response.success){
        cout<<"Release Wagon Pose Success"<<endl;
        //~ for(unsigned int i=0;i<rc_rwp.response.robot_joint_angle.size();i++){
          //~ for(unsigned int j=0;j<rc_rwp.response.robot_joint_angle[i].joint_angle.size();j++){
            //~ cout<<i<<"-"<<j<<":"<<rc_rwp.response.robot_joint_angle[i].joint_angle[j]<<"	";
          //~ }
          //~ cout<<endl;
        //~ }
      }
      //return 1;
    }
    else
    {
      ROS_ERROR("Failed to call rps_cnoid_release_wagon_planning");
      cout<<rc_rwp.response.message<<endl;
      return 1;
    }
  ///////////////////////////////////

  /////////PRM PLANNING///////////
    rc_PRM.request.in_motion_state.clear();

    temp_state.pushing_wagon_state = 0;
    temp_state.grasping_object_state = 3;

    temp_state.robot_pos = execute_path.push_wagon_Robot_path[execute_path.push_wagon_Robot_path.size()-1];
    temp_state.robot_pos.yaw = temp_state.robot_pos.th;
    temp_state.wagon_pos = execute_path.push_wagon_Wagon_path[execute_path.push_wagon_Wagon_path.size()-1];
    temp_state.wagon_pos.yaw = temp_state.wagon_pos.th;
    for(unsigned int i=0;i<rc_rwp.response.robot_joint_angle.size();i++){
      temp_state.robot_joint_angle = rc_rwp.response.robot_joint_angle[i];
      rc_PRM.request.in_motion_state.push_back(temp_state);
    }

    if (client_rc_PRM.call(rc_PRM))
    {
      if(!rc_PRM.response.success){
        ROS_ERROR("Cnoid PRM is failed : ");
        cout<<"	"<<rc_PRM.response.message<<endl;
        return 1;
      }
      if(rc_PRM.response.success){
        cout<<"PRM Success"<<endl;
        for(unsigned int i=0;i<rc_PRM.response.out_motion_state.size();i++){
          execute_path.release_wagon_Robot_joint_angle.push_back(rc_PRM.response.out_motion_state[i].robot_joint_angle);
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
      cout<<rc_PRM.response.message<<endl;
      return 1;
    }
  ///////////////////////////////////

  /////////GRASP OBJECT PLANNING///////////
    rc_gop.request.robot_id = execute_path.robot_id;
    rc_gop.request.obj_id = execute_path.object_id;

    rc_gop.request.robot_pos = execute_path.push_wagon_Robot_path[execute_path.push_wagon_Robot_path.size()-1];
    rc_gop.request.pre_robot_joint_angle =
  execute_path.release_wagon_Robot_joint_angle[execute_path.release_wagon_Robot_joint_angle.size()-1];

    if(client_rc_gop.call(rc_gop))
    {
      if(!rc_gop.response.success){
        ROS_ERROR("Grasp Obj is failed : ");
        cout<<"	"<<rc_gop.response.message<<endl;
        return 1;
      }
      if(rc_gop.response.success){
        cout<<"Grasp Obj Pose Success"<<endl;
        //~ for(unsigned int i=0;i<rc_gop.response.robot_joint_angle.size();i++){
          //~ for(unsigned int j=0;j<rc_gop.response.robot_joint_angle[i].joint_angle.size();j++){
            //~ cout<<i<<"-"<<j<<":"<<rc_gop.response.robot_joint_angle[i].joint_angle[j]<<"	";
          //~ }
          //~ cout<<endl;
        //~ }
      }
      //return 1;
    }
    else
    {
      ROS_ERROR("Failed to call rps_cnoid_grasp_obj_planning");
      cout<<rc_gop.response.message<<endl;
      return 1;
    }
  ///////////////////////////////////

  /////////PRM PLANNING///////////
    rc_PRM.request.in_motion_state.clear();

    temp_state.pushing_wagon_state = 0;
    temp_state.grasping_object_state = 3;

    temp_state.robot_pos = execute_path.push_wagon_Robot_path[execute_path.push_wagon_Robot_path.size()-1];
    temp_state.robot_pos.yaw= temp_state.robot_pos.th;
    temp_state.wagon_pos = execute_path.push_wagon_Wagon_path[execute_path.push_wagon_Wagon_path.size()-1];
    temp_state.wagon_pos.yaw = temp_state.wagon_pos.th;
    for(unsigned int i=0;i<rc_gop.response.robot_joint_angle.size();i++){
      temp_state.robot_joint_angle = rc_gop.response.robot_joint_angle[i];
      rc_PRM.request.in_motion_state.push_back(temp_state);
    }

    if (client_rc_PRM.call(rc_PRM))
    {
      if(!rc_PRM.response.success){
        ROS_ERROR("Cnoid PRM is failed : ");
        cout<<"	"<<rc_PRM.response.message<<endl;
        return 1;
      }
      if(rc_PRM.response.success){
        cout<<"PRM Success"<<endl;
        for(unsigned int i=0;i<rc_PRM.response.out_motion_state.size();i++){
          execute_path.grasp_obj_Robot_joint_angle.push_back(rc_PRM.response.out_motion_state[i].robot_joint_angle);
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
      cout<<rc_PRM.response.message<<endl;
      return 1;
    }
  ///////////////////////////////////

  /////////GIVE OBJECT PLANNING///////////
    rc_giop.request.robot_id = execute_path.robot_id;
    rc_giop.request.obj_id = execute_path.object_id;

    rc_giop.request.robot_pos = execute_path.give_obj_Robot_pos;
    rc_giop.request.pre_robot_joint_angle =
  execute_path.grasp_obj_Robot_joint_angle[execute_path.grasp_obj_Robot_joint_angle.size()-1];
    rc_giop.request.obj_pos = execute_path.give_obj_Obj_pos;

    if(client_rc_giop.call(rc_giop))
    {
      if(!rc_giop.response.success){
        ROS_ERROR("Give Obj is failed : ");
        cout<<"	"<<rc_giop.response.message<<endl;
        return 1;
      }
      if(rc_giop.response.success){
        cout<<"Give Obj Pose Success"<<endl;
        //~ for(unsigned int i=0;i<rc_giop.response.robot_joint_angle.size();i++){
          //~ for(unsigned int j=0;j<rc_giop.response.robot_joint_angle[i].joint_angle.size();j++){
            //~ cout<<i<<"-"<<j<<":"<<rc_giop.response.robot_joint_angle[i].joint_angle[j]<<"	";
          //~ }
          //~ cout<<endl;
        //~ }
        //return 1;
      }
    }
    else
    {
      ROS_ERROR("Failed to call rps_cnoid_give_obj_planning");
      cout<<rc_giop.response.message<<endl;
      return 1;
    }
  ///////////////////////////////////

  /////////PRM PLANNING///////////
    rc_PRM.request.in_motion_state.clear();

    temp_state.pushing_wagon_state = 0;
    temp_state.grasping_object_state = 2;

    temp_state.robot_pos = execute_path.push_wagon_Robot_path[execute_path.push_wagon_Robot_path.size()-1];
    temp_state.robot_pos.yaw= temp_state.robot_pos.th;
    temp_state.wagon_pos = execute_path.push_wagon_Wagon_path[execute_path.push_wagon_Wagon_path.size()-1];
    temp_state.wagon_pos.yaw= temp_state.wagon_pos.th;
    temp_state.robot_joint_angle = rc_giop.response.robot_joint_angle[0];
    rc_PRM.request.in_motion_state.push_back(temp_state);

    //~ temp_state.robot_joint_angle = rc_gop.response.robot_joint_angle[rc_gop.response.robot_joint_angle.size()-1];
    temp_state.robot_joint_angle.joint_angle.clear();
    for(int i=0;i<jointNum;i++){
      temp_state.robot_joint_angle.joint_angle.push_back(sp5_arm_give_pre_pose[i]);
    }
    temp_state.robot_joint_angle.joint_angle[9] =
  rc_gop.response.robot_joint_angle[rc_gop.response.robot_joint_angle.size()-1].joint_angle[9];
    rc_PRM.request.in_motion_state.push_back(temp_state);

    temp_state.robot_pos.th = execute_path.give_obj_Robot_pos.th;
    temp_state.robot_pos.yaw= temp_state.robot_pos.th;
    rc_PRM.request.in_motion_state.push_back(temp_state);

    temp_state.robot_joint_angle = rc_giop.response.robot_joint_angle[1];
    temp_state.robot_joint_angle.joint_angle[0] = temp_state.robot_joint_angle.joint_angle[1] = 0.0;
    rc_PRM.request.in_motion_state.push_back(temp_state);

    temp_state.robot_joint_angle = rc_giop.response.robot_joint_angle[1];
    rc_PRM.request.in_motion_state.push_back(temp_state);

    if (client_rc_PRM.call(rc_PRM))
    {
      if(!rc_PRM.response.success){
        ROS_ERROR("Cnoid PRM is failed : ");
        cout<<"	"<<rc_PRM.response.message<<endl;
        return 1;
      }
      if(rc_PRM.response.success){
        cout<<"PRM Success"<<endl;
        for(unsigned int i=0;i<rc_PRM.response.out_motion_state.size();i++){
          execute_path.give_obj_Robot_joint_angle.push_back(rc_PRM.response.out_motion_state[i].robot_joint_angle);
          //~ for(unsigned int j=0;j<rc_PRM.response.out_motion_state[i].robot_joint_angle.joint_angle.size();j++){
            //~ cout<<i<<"-"<<j<<":"<<rc_PRM.response.out_motion_state[i].robot_joint_angle.joint_angle[j]<<"	";
          //~ }
          //~ cout<<endl;
        }
        return 1;
      }
    }
    else
    {
      ROS_ERROR("Failed to call rps_cnoid_PRM_planning");
      cout<<rc_PRM.response.message<<endl;
      return 1;
    }
  ///////////////////////////////////
  */
  ros::spin();

  return 0;
}
