/*
 * tms_rs_greeting.cpp
 *
 *  Created on: 2014/07/28
 *      Author: hashiguchi
 */
#include <ros/ros.h>
#include <string>
#include <vector>

#include <tms_msg_rs/rs_task.h>
#include <tms_msg_rc/smartpal_control.h>
#include <tms_msg_rc/robot_tts.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#define UNIT_VEHICLE 1
#define UNIT_ARM_R 2
#define UNIT_ARM_L 3
#define UNIT_GRIPPER_R 4
#define UNIT_GRIPPER_L 5
#define UNIT_LUMBA 6

#define CMD_MOVE_ABS 15
#define CMD_MOVE_REL 16

ros::ServiceClient motion_client;
ros::ServiceClient speech_client;

// bool sp5_control_function(int unit, int cmd, int arg_size, double* arg) {
//	tms_msg_rc::smartpal_control sp_control_srv;
//
//	sp_control_srv.request.unit = unit;
//	sp_control_srv.request.cmd  = cmd;
//	sp_control_srv.request.arg.resize(arg_size);
//	for (int i=0; i<sp_control_srv.request.arg.size(); i++) {
//		sp_control_srv.request.arg[i] = arg[i];
//		ROS_INFO("arg[%d]=%f", i, sp_control_srv.request.arg[i]);
//	}
//
//	if (motion_client.call(sp_control_srv)) ROS_INFO("result: %d", sp_control_srv.response.result);
//	else ROS_ERROR("Failed to call service sp5_control");
//
//	return true;
//}
//
// bool sp5_control(const std::vector<std::vector<double> >& arg, int row, int column) {
//	for (int i=0; i<row; i++) {
//		double arg_vehicle[3] = {0.0, 0.0, arg[i][0]};
//		double arg_waist[4] = {arg[i][1], arg[i][2], 10.0, 10.0};
//		double arg_armR[8] = {arg[i][3], arg[i][4], arg[i][5], arg[i][6], arg[i][7], arg[i][8], arg[i][9], 10.0};
//		double arg_gripperR[3] = {arg[i][10], 10.0, 10.0};
//		double arg_armL[8] = {arg[i][11], arg[i][12], arg[i][13], arg[i][14], arg[i][15], arg[i][16], arg[i][17], 10.0};
//		double arg_gripperL[3] = {arg[i][18], 10.0, 10.0};
//
//		boost::thread thr_vehicle(boost::bind(&sp5_control_function, 1, 16, 3, arg_vehicle));
//		boost::thread thr_waist(boost::bind(&sp5_control_function, 6, 16, 4, arg_waist));
//		boost::thread thr_armR(boost::bind(&sp5_control_function, 2, 15, 8, arg_armR));
//		boost::thread thr_gripperR(boost::bind(&sp5_control_function, 4, 15, 3, arg_gripperR));
//		boost::thread thr_armL(boost::bind(&sp5_control_function, 3, 15, 8, arg_armL));
//		boost::thread thr_gripperL(boost::bind(&sp5_control_function, 5, 15, 3, arg_gripperL));
//
//		thr_vehicle.join();
//		thr_waist.join();
//		thr_armR.join();
//		thr_gripperR.join();
//		thr_armL.join();
//		thr_gripperL.join();
//		ros::Duration(2.0).sleep();
//	}
//	return true;
//}

bool sp5_control_function(int unit, int cmd, int arg_size, double* arg)
{
  tms_msg_rc::smartpal_control sp_control_srv;

  sp_control_srv.request.unit = unit;
  sp_control_srv.request.cmd = cmd;
  sp_control_srv.request.arg.resize(arg_size);
  for (int i = 0; i < sp_control_srv.request.arg.size(); i++)
  {
    sp_control_srv.request.arg[i] = arg[i];
    ROS_INFO("arg[%d]=%f", i, sp_control_srv.request.arg[i]);
  }

  if (motion_client.call(sp_control_srv))
    ROS_INFO("result: %d", sp_control_srv.response.result);
  else
    ROS_ERROR("Failed to call service sp5_control");

  return true;
}

bool sp5_control(const std::vector< std::vector< double > >& arg, int row, int column)
{
  for (int i = 0; i < row; i++)
  {
    double arg_vehicle[3] = {0.0, 0.0, arg[i][0]};
    sp5_control_function(UNIT_VEHICLE, CMD_MOVE_REL, 3, arg_vehicle);

    double arg_armR[8] = {arg[i][3], arg[i][4], arg[i][5], arg[i][6], arg[i][7], arg[i][8], arg[i][9], 10.0};
    sp5_control_function(UNIT_ARM_R, CMD_MOVE_ABS, 8, arg_armR);

    //		double arg_gripperR[3] = {arg[i][10], 10.0, 10.0};
    //		sp5_control_function(UNIT_GRIPPER_R, CMD_MOVE_ABS, 3, arg_gripperR);

    double arg_armL[8] = {arg[i][11], arg[i][12], arg[i][13], arg[i][14], arg[i][15], arg[i][16], arg[i][17], 10.0};
    sp5_control_function(UNIT_ARM_L, CMD_MOVE_ABS, 8, arg_armL);

    //		double arg_gripperL[3] = {arg[i][18], 10.0, 10.0};
    //		sp5_control_function(UNIT_GRIPPER_L, CMD_MOVE_ABS, 3, arg_gripperL);

    double arg_waist[4] = {arg[i][1], arg[i][2], 10.0, 10.0};
    sp5_control_function(UNIT_LUMBA, CMD_MOVE_REL, 4, arg_waist);

    ros::Duration(2.0).sleep();
  }
  return true;
}

bool sp5_tts(const std::vector< std::string >& sentene, const std::vector< double >& timing, int size)
{
  tms_msg_rc::robot_tts sp_tts_srv;

  for (int i = 0; i < size; i++)
  {
    sp_tts_srv.request.text = sentene[i];
    if (speech_client.call(sp_tts_srv))
      ROS_INFO("result: %d", sp_tts_srv.response.result);
    else
    {
      ROS_ERROR("Failed to call service smartpal5_tts");
      return false;
    }
    ros::Duration(timing[i]).sleep();
  }
  return true;
}

bool callback(tms_msg_rs::rs_task::Request& req, tms_msg_rs::rs_task::Response& res)
{
  // Create a ROS node handle
  ros::NodeHandle n;
  static std::string str("/tms_rs_greeting/");

  // get parameters from the parameter server(main block)
  std::vector< std::string > s_motion_list, s_speech_list;
  std::vector< double > timing_list;

  std::string main_name = req.task;
  std::string motion_name = str + main_name + "M";
  std::string speech_name = str + main_name + "S";
  std::string timing = speech_name + "_t";

  n.getParam(motion_name, s_motion_list);
  n.getParam(speech_name, s_speech_list);
  n.getParam(timing, timing_list);
  //	n.getParam("/tms_rs_greeting/greeting_for_opencampus0M", s_motion_list);
  //	n.getParam("/tms_rs_greeting/greeting_for_opencampus0S", s_speech_list);
  //	n.getParam("/tms_rs_greeting/greeting_for_opencampus0S_t", s_speech_list);

  if (s_motion_list.size() != s_speech_list.size() || s_speech_list.size() != timing_list.size())
  {
    ROS_ERROR("Parameter size is wrong.");
    return false;
  }

  // get parameters from the parameter server(sub block)
  std::vector< std::string > s_sentence_list;
  std::vector< std::vector< std::string > > s_submotion_list;

  s_submotion_list.resize(s_motion_list.size());
  for (int i = 0; i < s_motion_list.size(); i++)
  {
    std::string m_arg = str + s_motion_list[i];
    ROS_INFO("param name: %s", m_arg.c_str());
    n.getParam(m_arg, s_submotion_list[i]);
  }
  s_sentence_list.resize(s_speech_list.size());
  for (int i = 0; i < s_speech_list.size(); i++)
  {
    std::string s_arg = str + s_speech_list[i];
    ROS_INFO("param name: %s", s_arg.c_str());
    n.getParam(s_arg, s_sentence_list[i]);
    ROS_INFO("speech%d: %s", i, s_sentence_list[i].c_str());
  }

  std::vector< double > tmp_d;
  std::vector< std::vector< double > > d_motion_list;

  for (int i = 0; i < s_submotion_list.size(); i++)
  {
    for (int j = 0; j < s_submotion_list[i].size(); j++)
    {
      std::string m2_arg = str + s_submotion_list[i][j];
      ROS_INFO("param name: %s", m2_arg.c_str());
      tmp_d.clear();
      n.getParam(m2_arg, tmp_d);
      d_motion_list.push_back(tmp_d);
    }
  }

  // 確認
  //	for (int j=0; j<d_motion_list.size(); j++) {
  //		ROS_INFO("d_motion_list[%d].size=%d", j, d_motion_list[j].size());
  //		for (int l=0; l<d_motion_list[j].size(); l++) {
  //			ROS_INFO("motion%d %d: %f", j, l, d_motion_list[j][l]);
  //		}
  //	}

  int row = d_motion_list.size();
  int column = d_motion_list[0].size();
  int size = s_sentence_list.size();

  // Service call for greeting
  boost::thread thr_motion(boost::bind(&sp5_control, d_motion_list, row, column));
  boost::thread thr_speech(boost::bind(&sp5_tts, s_sentence_list, timing_list, size));

  thr_motion.join();
  thr_speech.join();

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tms_rs_greeting");
  ros::NodeHandle n;

  motion_client = n.serviceClient< tms_msg_rc::smartpal_control >("sp5_control");
  // motion_client = n.serviceClient<tms_msg_rc::smartpal_control>("sp5_virtual_control");
  speech_client = n.serviceClient< tms_msg_rc::robot_tts >("smartpal5_tts");

  ros::ServiceServer service = n.advertiseService("opencampus_greeting", callback);
  ROS_INFO("Ready to do greeting demonstration.");
  ros::spin();

  return 0;
}
