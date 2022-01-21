/*
 * ts_master.hpp
 *
 *  Created on: 2013/10/29
 *      Author: hashiguchi
 */

#ifndef TS_MASTER_HPP_
#define TS_MASTER_HPP_

#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <string.h>
#include <vector>
#include <list>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <tms_msg_ts/ts_req.h>
#include <tms_msg_ts/ts_state_control.h>

// using namespace std;
// using namespace boost;

#define MAX_TASK_NUM 30

class TmsTsMaster
{
public:
  // for generating commands
  static std::string run_nodelet_manager;
  static std::string rosrun;
  static std::string node_name;
  static std::string service_name;

  TmsTsMaster(int argc, char **argv);
  void ros_spin(void);
  std::string CreateSrvCall(long long int rostime, int task_id, int robot_id, int object_id, int user_id, int place_id,
                            int priority, int thread_num);
  std::string CreateRunCmd(int thread_num);
  bool ExecuteCmd(const char *buf);
  boost::thread *threads[MAX_TASK_NUM * 2];
  bool addThread(int thread_num,const char* arg1,const char* arg2);

  // TMS TASK
  struct Task
  {
    long long int rostime;
    int task_id;
    int robot_id;
    int object_id;
    int user_id;
    int place_id;
    //		// 条件
    int priority;
    //		long long int time; // for punctual task
    //		int dependence;  // task dependency
  };

  struct TaskManager
  {
    std::string num;
    bool flag;
  };
  TaskManager task_manager[MAX_TASK_NUM * 2];

private:
  // sort
  struct pred
  {
    bool operator()(const Task &l, const Task &r) const
    {
      return l.priority == r.priority ? l.rostime < r.rostime : l.priority < r.priority;
    }
  };

  bool ts_master_callback(tms_msg_ts::ts_req::Request &req, tms_msg_ts::ts_req::Response &res);
  bool stsCallback(tms_msg_ts::ts_state_control::Request &req, tms_msg_ts::ts_state_control::Response &res);

  boost::mutex mtx;
  static int state_condition, loop_counter;
  static bool abort;
  ros::ServiceServer service, state_control_srv;
};

#endif /* TS_MASTER_HPP_ */
