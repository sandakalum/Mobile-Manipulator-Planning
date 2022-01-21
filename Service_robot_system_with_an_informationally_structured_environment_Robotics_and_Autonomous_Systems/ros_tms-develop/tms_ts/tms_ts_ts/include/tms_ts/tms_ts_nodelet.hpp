/*
 * tms_ts_nodelet.hpp
 *
 *  Created on: 2013/10/01
 *      Author: hashiguchi
 */

#ifndef TMS_TS_NODELET_HPP_
#define TMS_TS_NODELET_HPP_

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <assert.h>

#include <sstream>
#include <fstream>
#include <boost/date_time/local_time/local_time.hpp>
#include <time.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <tms_msg_ts/ts_req.h>        // from UR
#include <tms_msg_db/TmsdbGetData.h>  // to DB

namespace tms_ts_nodelet
{
class ROS_TMS_TS : public nodelet::Nodelet
{
public:
  ROS_TMS_TS()
    : sid(100000)
    , robot_id(0)
    , object_id(0)
    , user_id(0)
    , place_id(0)
    , import("#!/usr/bin/env python\n\n"
             "import roslib; roslib.load_manifest('tms_ts_smach')\n"
             "import rospy\n"
             "import smach\n"
             "import smach_ros\n\n"
             "from smach_ros import ServiceState\n"
             "from smach import Concurrence\n\n"
             "from tms_msg_rp.srv import *\n"
             "from tms_msg_ts.srv import *\n\n")
    , main_function1("def main():\n"
                     "    rospy.init_node('tms_ts_smach_executive")
    , main_function2("')\n\n"
                     "    sm_root = smach.StateMachine(['succeeded','aborted','preempted'])\n\n"
                     "    with sm_root:\n\n")
    , introspection_server("    sis = smach_ros.IntrospectionServer('tms_ts_smach_test',"
                           " sm_root, '/ROS_TMS')\n"
                           "    sis.start()\n\n"
                           "    outcome = sm_root.execute()\n\n"
                           "    rospy.spin()\n"
                           "    sis.stop()\n\n"
                           "if __name__ == '__main__':\n"
                           "    main()\n")
  {
  }

private:
  virtual void onInit();

  std::string rosCheckTime(boost::posix_time::ptime time);

  std::string IntToString(int number);
  int StringToInt(std::string str);

  std::string BoolToString(bool b);

  int ArrayPush(std::string *stack, std::string data, int *sp, size_t n);
  std::string ArrayPop(std::string *stack, int *sp);

  int ConvertArgType(std::string arg_type);
  int JudgeArgType(std::string state1, std::string state2);
  void GenerateContainer(std::string f_name, std::string state_name1, std::string state_name2);
  int GenerateCC(std::string state1, std::string state2, int cc_count);
  int AddOneStateSQ(std::string state1);
  int AddStateCC(int cc_count, int sub_count);
  int BuildStateVector(std::string state1, std::string state2);
  int AddStateSQ(std::string state1, std::string state2);
  void GenerateScript(void);

  bool ExeScript(void);

  bool tsCallback(tms_msg_ts::ts_req::Request &req, tms_msg_ts::ts_req::Response &res);
  static int count_callback;

  const int sid;
  int robot_id;
  int object_id;
  int user_id;
  int place_id;

  const std::string import;
  const std::string main_function1, main_function2;
  const std::string introspection_server;

  struct StateData
  {
    int state_id;
    std::string state_name;
    std::vector< int > arg;
    int cc_subtasks;  // num of CC subtasks
  };

  std::vector< StateData > state_data;

  std::string file_name;
  std::string generated_container;
  std::string generated_main;

  ros::ServiceServer service;           // connect to UR
  ros::ServiceClient db_reader_client;  // connect to DB
};

PLUGINLIB_DECLARE_CLASS(tms_ts_nodelet, ROS_TMS_TS, tms_ts_nodelet::ROS_TMS_TS, nodelet::Nodelet);
}

#endif /* ROS_TMS_TS_NODELET_HPP_ */
