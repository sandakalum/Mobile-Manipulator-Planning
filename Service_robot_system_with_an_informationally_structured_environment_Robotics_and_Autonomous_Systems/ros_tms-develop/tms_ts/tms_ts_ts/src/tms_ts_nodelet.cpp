#include <tms_ts/tms_ts_nodelet.hpp>
#define N 20  // num of stack

int tms_ts_nodelet::ROS_TMS_TS::count_callback = 0;

void tms_ts_nodelet::ROS_TMS_TS::onInit()
{
  // initialize member variable
  state_data.clear();
  file_name = "";
  generated_container = "";
  generated_main = "";

  ros::NodeHandle nh;
  ros::NodeHandle &private_nh = getPrivateNodeHandle();
  service = private_nh.advertiseService("tms_ts", &ROS_TMS_TS::tsCallback, this);
  // for connecting to ROSTMS_DB task,subtask
  db_reader_client = nh.serviceClient< tms_msg_db::TmsdbGetData >("tms_db_reader");
}

std::string tms_ts_nodelet::ROS_TMS_TS::rosCheckTime(boost::posix_time::ptime time)
{
  struct tm T = boost::posix_time::to_tm(time);  // ptime -> struct tm
  std::ostringstream oss;
  oss << T.tm_year + 1900 << T.tm_mon + 1 << T.tm_mday << "_" << T.tm_hour << T.tm_min << T.tm_sec;
  return oss.str();
}

std::string tms_ts_nodelet::ROS_TMS_TS::BoolToString(bool b)
{
  return b ? "True" : "False";
}

std::string tms_ts_nodelet::ROS_TMS_TS::IntToString(int number)
{
  std::stringstream ss;
  ss << number;
  return ss.str();
}

int tms_ts_nodelet::ROS_TMS_TS::StringToInt(std::string str)
{
  std::istringstream iss(str);
  int num = 0;
  iss >> num;
  return num;
}

int tms_ts_nodelet::ROS_TMS_TS::ArrayPush(std::string *stack, std::string data, int *sp, size_t n)
{
  if (*sp < n)
  {
    stack[(*sp)++] = data;
    ROS_INFO("Push: %s\n", data.c_str());
    return *sp;
  }
  else
  {
    return 0;
  }
}

std::string tms_ts_nodelet::ROS_TMS_TS::ArrayPop(std::string *stack, int *sp)
{
  if (*sp > 0)
  {
    return stack[--(*sp)];
  }
  else
  {
    return "False";
  }
}

int tms_ts_nodelet::ROS_TMS_TS::ConvertArgType(std::string arg_type)
{
  ROS_INFO("arg:%s", arg_type.c_str());

  if (arg_type == "oid")  // object_id
  {
    return object_id;
  }
  else if (arg_type == "uid")  // user_id
  {
    return user_id;
  }
  else if (arg_type == "pid")  // place_id
  {
    return place_id;
  }
  else if (arg_type == "rid")  // robot_id
  {
    return robot_id;
  }
  else
  {
    return StringToInt(arg_type);
  }
}

// token: サブタスクを表す
// token_block: 既に演算子で1回以上接続されたトークンのまとまり
/* return 0: token_block [+/|] token_block
 *        1: token [+/|] token_block
 *        2: token_block [+/|] token
 *        3: token [+/|] token*/
int tms_ts_nodelet::ROS_TMS_TS::JudgeArgType(std::string state1, std::string state2)
{
  std::vector< std::string > seq_of_argument1, seq_of_argument2;
  seq_of_argument1.clear();
  seq_of_argument2.clear();

  boost::split(seq_of_argument1, state1, boost::is_any_of("$"));
  boost::split(seq_of_argument2, state2, boost::is_any_of("$"));

  if (!seq_of_argument1.empty() && !seq_of_argument2.empty())
  {
    if (seq_of_argument1.size() == 1 && seq_of_argument2.size() == 1)
      return 0;
    else if (seq_of_argument1.size() != 1 && seq_of_argument2.size() == 1)
      return 1;
    else if (seq_of_argument1.size() == 1 && seq_of_argument2.size() != 1)
      return 2;
    else if (seq_of_argument1.size() != 1 && seq_of_argument2.size() != 1)
      return 3;
  }
}

void tms_ts_nodelet::ROS_TMS_TS::GenerateContainer(std::string f_name, std::string state_name1, std::string state_name2)
{
  generated_container +=
      "def " + f_name + "():\n    " + f_name + " = smach.Concurrence"
                                               "( outcomes=['succeeded', 'aborted'],\n"
                                               "                            default_outcome = 'aborted',\n"
                                               "                            outcome_map = {'succeeded': {'" +
      state_name1 + "':'succeeded', '" + state_name2 + "':'succeeded'}},\n"
                                                       "                            child_termination_cb = lambda arg: "
                                                       "True )\n\n"
                                                       "    with " +
      f_name + ":\n";
}

int tms_ts_nodelet::ROS_TMS_TS::GenerateCC(std::string state1, std::string state2, int cc_count)
{
  int state_count = cc_count * 2;
  int sn1 = state_count;
  int sn2 = state_count + 1;
  int arg_type = JudgeArgType(state1, state2);
  int subtasks = 0;

  tms_msg_db::TmsdbGetData srv;
  std::vector< std::string > seq_of_argument1, seq_of_argument2;
  seq_of_argument1.clear();
  seq_of_argument2.clear();

  boost::split(seq_of_argument1, state1, boost::is_any_of("$"));
  boost::split(seq_of_argument2, state2, boost::is_any_of("$"));

  std::string state_id1, state_id2;
  state_id1 = seq_of_argument1.at(0);
  state_id2 = seq_of_argument2.at(0);

  std::string state_name1, state_name2;
  std::string f_name = "smc" + IntToString(cc_count);

  ros::NodeHandle nh;
  bool type;
  nh.getParam("/2003_is_real", type);

  if (arg_type == 0)
  {
    ROS_INFO("type=0\n");
    std::string state_right = IntToString(sn1) + "CC";
    std::string state_left = IntToString(sn2) + "CC";
    GenerateContainer(f_name, state_right, state_left);

    generated_container += "        smach.Concurrence.add('" + state_right + "', " +
                           state_data.at(StringToInt(state_id1) - 1).state_name + "())\n\n"
                                                                                  "        smach.Concurrence.add('" +
                           state_left + "', " + state_data.at(StringToInt(state_id2) - 1).state_name + "())\n\n"
                                                                                                       "    return " +
                           f_name + "\n\n";
    subtasks = state_data.at(state_data.size() - 2).cc_subtasks + state_data.at(state_data.size() - 1).cc_subtasks;
    state_data.pop_back();
    state_data.pop_back();
  }
  else if (arg_type == 1)
  {
    ROS_INFO("type=1\n");
    srv.request.tmsdb.id = StringToInt(seq_of_argument1.at(0)) + sid;
    if (db_reader_client.call(srv))
      state_name1 = srv.response.tmsdb[0].name;

    std::string state_right = IntToString(sn1) + state_name1;
    std::string state_left = IntToString(sn2) + "CC";
    GenerateContainer(f_name, state_right, state_left);
    generated_container += "        smach.Concurrence.add('" + state_right +
                           "',\n"
                           "                           ServiceState('rp_cmd',\n"
                           "                                        rp_cmd,\n"
                           "                                        request = rp_cmdRequest(" +
                           state_id1 + ", " + BoolToString(type) + ", " + IntToString(robot_id) + ", [";
    int check = 0;
    for (int i = 1; i < seq_of_argument1.size(); i++)
    {
      if (check >= 1)
      {
        generated_container += ", ";
      }
      generated_container += IntToString(ConvertArgType(seq_of_argument1.at(i)));
      check++;
    }
    generated_container += "])))\n"
                           "        smach.Concurrence.add('" +
                           state_left + "', " + state_data.at(StringToInt(state_id2) - 1).state_name + "())\n\n";

    generated_container += "    return " + f_name + "\n\n";
    subtasks = 1 + state_data.at(state_data.size() - 1).cc_subtasks;
    state_data.pop_back();
  }
  else if (arg_type == 2)
  {
    ROS_INFO("type=2\n");
    srv.request.tmsdb.id = StringToInt(seq_of_argument2.at(0)) + sid;
    if (db_reader_client.call(srv))
      state_name2 = srv.response.tmsdb[0].name;

    std::string state_right = IntToString(sn1) + "CC";
    std::string state_left = IntToString(sn2) + state_name2;
    GenerateContainer(f_name, state_right, state_left);

    generated_container += "        smach.Concurrence.add('" + state_right + "', " +
                           state_data.at(StringToInt(state_id1) - 1).state_name + "())\n\n"
                                                                                  "        smach.Concurrence.add('" +
                           state_left + "',\n"
                                        "                           ServiceState('rp_cmd',\n"
                                        "                                        rp_cmd,\n"
                                        "                                        request = rp_cmdRequest(" +
                           state_id2 + ", " + BoolToString(type) + ", " + IntToString(robot_id) + ", [";
    int check = 0;
    for (int j = 1; j < seq_of_argument2.size(); j++)
    {
      if (check >= 1)
      {
        generated_container += ", ";
      }
      generated_container += IntToString(ConvertArgType(seq_of_argument2.at(j)));
      check++;
    }
    generated_container += "])))\n"
                           "    return " +
                           f_name + "\n\n";
    subtasks = state_data.at(state_data.size() - 1).cc_subtasks + 1;
    state_data.pop_back();
    //			state_data.erase(state_data.end() - 1);
  }
  else if (arg_type == 3)
  {
    ROS_INFO("type=3\n");
    srv.request.tmsdb.id = StringToInt(seq_of_argument1.at(0)) + sid;
    if (db_reader_client.call(srv))
      state_name1 = srv.response.tmsdb[0].name;

    srv.request.tmsdb.id = StringToInt(seq_of_argument2.at(0)) + sid;
    if (db_reader_client.call(srv))
      state_name2 = srv.response.tmsdb[0].name;

    std::string state_right = IntToString(sn1) + state_name1;
    std::string state_left = IntToString(sn2) + state_name2;
    GenerateContainer(f_name, state_right, state_left);

    generated_container += "        smach.Concurrence.add('" + state_right +
                           "',\n"
                           "                           ServiceState('rp_cmd',\n"
                           "                                        rp_cmd,\n"
                           "                                        request = rp_cmdRequest(" +
                           state_id1 + ", " + BoolToString(type) + ", " + IntToString(robot_id) + ", [";
    int check = 0;
    for (int i = 1; i < seq_of_argument1.size(); i++)
    {
      if (check >= 1)
      {
        generated_container += ", ";
      }
      generated_container += IntToString(ConvertArgType(seq_of_argument1.at(i)));
      check++;
    }
    generated_container += "])))\n"
                           "        smach.Concurrence.add('" +
                           state_left + "',\n"
                                        "                           ServiceState('rp_cmd',\n"
                                        "                                        rp_cmd,\n"
                                        "                                        request = rp_cmdRequest(" +
                           state_id2 + ", " + BoolToString(type) + ", " + IntToString(robot_id) + ", [";
    check = 0;
    for (int j = 1; j < seq_of_argument2.size(); j++)
    {
      if (check >= 1)
      {
        generated_container += ", ";
      }
      generated_container += IntToString(ConvertArgType(seq_of_argument2.at(j)));
      check++;
    }
    generated_container += "])))\n"
                           "    return " +
                           f_name + "\n\n";
    subtasks = 2;
  }
  return subtasks;
}

int tms_ts_nodelet::ROS_TMS_TS::AddStateCC(int cc_count, int sub_count)
{
  std::string function_name = "smc" + IntToString(cc_count);

  StateData sd;
  sd.state_id = cc_count;
  sd.state_name = function_name;
  sd.arg.clear();
  sd.arg.push_back(-10000);  // differentiate CC and SQ
  sd.cc_subtasks = sub_count;
  state_data.push_back(sd);
  return state_data.size();
}

int tms_ts_nodelet::ROS_TMS_TS::BuildStateVector(std::string state1, std::string state2)
{
  tms_msg_db::TmsdbGetData srv;
  StateData sd;
  std::vector< std::string > seq_of_argument1, seq_of_argument2;
  seq_of_argument1.clear();
  seq_of_argument2.clear();

  boost::split(seq_of_argument1, state1, boost::is_any_of("$"));
  boost::split(seq_of_argument2, state2, boost::is_any_of("$"));

  int arg_type = JudgeArgType(state1, state2);
  if (arg_type == 0)
  {
  }
  else if (arg_type == 1)
  {
    sd.state_id = StringToInt(seq_of_argument1.at(0));

    srv.request.tmsdb.id = sd.state_id + sid;
    if (db_reader_client.call(srv))
      sd.state_name = srv.response.tmsdb[0].name;

    sd.arg.clear();
    for (int i = 1; i < seq_of_argument1.size(); i++)
      sd.arg.push_back(ConvertArgType(seq_of_argument1.at(i)));

    std::vector< StateData >::iterator it = state_data.begin();
    it = state_data.insert(it, sd);
  }
  else if (arg_type == 2)
  {
    // state2
    sd.state_id = StringToInt(seq_of_argument2.at(0));

    srv.request.tmsdb.id = sd.state_id + sid;
    if (db_reader_client.call(srv))
      sd.state_name = srv.response.tmsdb[0].name;

    sd.arg.clear();
    for (int i = 1; i < seq_of_argument2.size(); i++)
      sd.arg.push_back(ConvertArgType(seq_of_argument2.at(i)));

    state_data.push_back(sd);
  }
  else if (arg_type == 3)
  {
    // state1
    sd.state_id = StringToInt(seq_of_argument1.at(0));

    srv.request.tmsdb.id = sd.state_id + sid;
    if (db_reader_client.call(srv))
      sd.state_name = srv.response.tmsdb[0].name;

    sd.arg.clear();
    for (int i = 1; i < seq_of_argument1.size(); i++)
      sd.arg.push_back(ConvertArgType(seq_of_argument1.at(i)));

    state_data.push_back(sd);

    // state2
    sd.state_id = StringToInt(seq_of_argument2.at(0));

    srv.request.tmsdb.id = sd.state_id + sid;
    if (db_reader_client.call(srv))
      sd.state_name = srv.response.tmsdb[0].name;

    sd.arg.clear();
    for (int i = 1; i < seq_of_argument2.size(); i++)
      sd.arg.push_back(ConvertArgType(seq_of_argument2.at(i)));

    state_data.push_back(sd);
  }
  return state_data.size();
}

int tms_ts_nodelet::ROS_TMS_TS::AddStateSQ(std::string state1, std::string state2)
{
  int vector_size = BuildStateVector(state1, state2);
  return vector_size;
}

int tms_ts_nodelet::ROS_TMS_TS::AddOneStateSQ(std::string state1)
{
  tms_msg_db::TmsdbGetData srv;
  StateData sd;
  std::vector< std::string > seq_of_argument1;
  seq_of_argument1.clear();

  boost::split(seq_of_argument1, state1, boost::is_any_of("$"));

  // state1
  sd.state_id = StringToInt(seq_of_argument1.at(0));

  srv.request.tmsdb.id = sd.state_id + sid;
  if (db_reader_client.call(srv))
    sd.state_name = srv.response.tmsdb[0].name;

  sd.arg.clear();
  for (int i = 1; i < seq_of_argument1.size(); i++)
    sd.arg.push_back(ConvertArgType(seq_of_argument1.at(i)));

  state_data.push_back(sd);
}

void tms_ts_nodelet::ROS_TMS_TS::GenerateScript()
{
  ros::NodeHandle nh;
  bool type;
  nh.getParam("/2003_is_real", type);

  for (int j = 0; j < state_data.size(); j++)
  {
    if (state_data.at(j).arg.at(0) == -10000)
    {  // CC
      generated_main += "        smach.StateMachine.add('" + state_data.at(j).state_name + "', "
                                                                                           "" +
                        state_data.at(j).state_name + "(), transitions={'succeeded':'";
      generated_main += "control" + IntToString(j) + "'})\n\n" + "        smach.StateMachine.add('control" +
                        IntToString(j) + "',\n"
                                         "                           ServiceState('ts_state_control',\n"
                                         "                                        ts_state_control,\n"
                                         "                                        request = ts_state_controlRequest(0, "
                                         "0, 0, " +
                        IntToString(state_data.at(j).cc_subtasks) + ", \"\")),\n";
      generated_main += "                           transitions={'succeeded':'";
      if (j == state_data.size() - 1)
      {
        generated_main += "succeeded', 'aborted':'aborted'})\n\n";
      }
      else
      {
        if (state_data.at(j + 1).arg.at(0) == -10000)
          generated_main += state_data.at(j + 1).state_name + "', 'aborted':'aborted'})\n\n";
        else
        {
          generated_main += state_data.at(j + 1).state_name + IntToString(j + 1) + "', 'aborted':'aborted'})\n\n";
        }
      }
    }
    else
    {  // SQ
      generated_main += "        smach.StateMachine.add('" + state_data.at(j).state_name + IntToString(j) +
                        "',\n"
                        "                           ServiceState('rp_cmd',\n"
                        "                                        rp_cmd,\n"
                        "                                        request = rp_cmdRequest(" +
                        IntToString(state_data.at(j).state_id) + ", " + BoolToString(type) + ", " +
                        IntToString(robot_id) + ", [";
      int check = 0;

      for (int k = 0; k < state_data.at(j).arg.size(); k++)
      {
        if (check >= 1)
        {
          generated_main += ", ";
        }
        generated_main += IntToString(state_data.at(j).arg.at(k));
        check++;
      }

      generated_main += "])),\n";
      generated_main += "                           transitions={'succeeded':'control" + IntToString(j) + "'})\n\n" +
                        "        smach.StateMachine.add('control" + IntToString(j) +
                        "',\n"
                        "                           ServiceState('ts_state_control',\n"
                        "                                        ts_state_control,\n"
                        "                                        request = ts_state_controlRequest(0, 0, 0, 0, "
                        "\"\")),\n";
      // final state
      if (j == state_data.size() - 1)
      {
        generated_main += "                           transitions={'succeeded':'succeeded', "
                          "'aborted':'aborted'})\n\n";
      }
      else
      {
        if (state_data.at(j + 1).arg.at(0) == -10000)
          generated_main += "                           transitions={'succeeded':'" + state_data.at(j + 1).state_name +
                            "', 'aborted':'aborted'})\n\n";
        else
          generated_main += "                           transitions={'succeeded':'" + state_data.at(j + 1).state_name +
                            IntToString(j + 1) + "', 'aborted':'aborted'})\n\n";
      }
    }
  }

  // ファイル名生成(time)
  std::ostringstream oss_filename;
  oss_filename.str("");  // initialize

  // 現在の日時を取得
  ros::Time tNow = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9
  oss_filename << ROS_TMS_TS::rosCheckTime(tNow.toBoost()) << ".py";
  ROS_TMS_TS::file_name = oss_filename.str();

  char home_dir[255];
  strcpy(home_dir, getenv("HOME"));
  std::string file_name_full = home_dir;
  file_name_full += "/catkin_ws/src/ros_tms/tms_ts/tms_ts_smach/scripts/";
  file_name_full += ROS_TMS_TS::file_name;

  std::ofstream ofs(file_name_full.c_str(), std::ios::in);

  if (ofs)
  {
    // error, file exists!
    ROS_ERROR("There is a file named \'%s\'", file_name_full.c_str());
    exit(1);
  }
  else
  {
    ofs.close();
    ofs.open(file_name_full.c_str(), std::ios::out | std::ios::ate);  // OK now

    if (!ofs)
    {
      ROS_ERROR("***error  Cannot open this file!\n");
      exit(1);
    }

    ofs << import << generated_container << main_function1 << count_callback << main_function2 << generated_main
        << introspection_server;
    ofs.close();
  }
  ROS_INFO("Finish generating python script to tms_ts_smach/scripts/%s\n", ROS_TMS_TS::file_name.c_str());
}

bool tms_ts_nodelet::ROS_TMS_TS::ExeScript()
{
  int ret;

  std::string chmod("");    // move directory & chmod
  std::string command("");  // running command

  chmod = "cd ~/catkin_ws/src/ros_tms/tms_ts/tms_ts_smach && chmod +x scripts/" + ROS_TMS_TS::file_name;
  command = "cd ~/catkin_ws/src/ros_tms/tms_ts/tms_ts_smach && ./scripts/" + ROS_TMS_TS::file_name;

  const char *buf1 = chmod.c_str();
  const char *buf2 = command.c_str();
  ROS_INFO("CMD1:%s\n", buf1);
  ROS_INFO("CMD2:%s\n", buf2);

  ret = system(buf1);
  if (ret != 0)
  {
    ROS_ERROR("Execute chmod error\n");
    return false;
  }

  ros::Duration(0.5).sleep();  // temp

  ret = system(buf2);
  if (ret != 0)
  {
    ROS_ERROR("Execute script error\n");
    return false;
  }

  return true;
}

bool tms_ts_nodelet::ROS_TMS_TS::tsCallback(tms_msg_ts::ts_req::Request &req, tms_msg_ts::ts_req::Response &res)
{
  count_callback++;
  robot_id = req.robot_id;
  object_id = req.object_id;
  user_id = req.user_id;
  place_id = req.place_id;

  // get task's data from id table
  tms_msg_db::TmsdbGetData srv;
  srv.request.tmsdb.id = req.task_id + sid;
  ROS_INFO("%d",sid);
  if (db_reader_client.call(srv))
  {
    ROS_INFO("get task data from tms_db\n");
    std::string stack[N];  // stack
    int sp = 0;            // stack pointer

    std::string subtask = srv.response.tmsdb[0].etcdata;
    
    std::vector< std::string > variable_subtask;
    variable_subtask.clear();
    boost::split(variable_subtask, subtask, boost::is_any_of(";"));
    if(variable_subtask.size() == 1){
      std::vector< std::string > seq_of_subtask;
      seq_of_subtask.clear();
      boost::split(seq_of_subtask, subtask, boost::is_any_of(" "));
      int cc_count = 0;
      int index = 0;
      int type;
      int num_of_subtasks;

      for (int cnt = 0; cnt < seq_of_subtask.size(); cnt++)
      {
        // distinguish operator
        if (seq_of_subtask.at(cnt) == "+")
        {  // sequential subtask
          ROS_INFO("+\n");
          // add state to main function
          std::string state1 = ArrayPop(stack, &sp);
          std::string state2 = ArrayPop(stack, &sp);
          if (state2 == "False")
          {  // There is one state
            index = AddOneStateSQ(state1);
          }
          else
          {
            ROS_INFO("state1:%s, state2:%s", state1.c_str(), state2.c_str());
            index = AddStateSQ(state2, state1);
            ArrayPush(stack, IntToString(index), &sp, N);
          }
        }
        else if (seq_of_subtask.at(cnt) == "|")
        {  // concurrent subtask
          // create concurrence container
          std::string state1 = ArrayPop(stack, &sp);
          std::string state2 = ArrayPop(stack, &sp);
          num_of_subtasks = GenerateCC(state2, state1, cc_count);
          // add state to main function
          index = AddStateCC(cc_count, num_of_subtasks);
          ArrayPush(stack, IntToString(index), &sp, N);
          cc_count++;
        }
        else
        {
          ArrayPush(stack, seq_of_subtask.at(cnt), &sp, N);
        }
      }
    }
    else{
      for(int task_cnt = 0; task_cnt < variable_subtask.size(); task_cnt++){
        std::vector< std::string > seq_of_subtask;

        seq_of_subtask.clear();
        boost::split(seq_of_subtask, variable_subtask.at(task_cnt), boost::is_any_of(" "));
        int cc_count = 0;
        int index = 0;
        int type;
        int num_of_subtasks;
        
        std::vector< std::string > task_args;
        task_args.clear();
        
        boost::split(task_args, seq_of_subtask.at(0), boost::is_any_of("$"));
        ROS_INFO("%s", task_args.at(1));
        if(ConvertArgType(task_args.at(1)) == 0){
          continue;
        }

        for (int cnt = 0; cnt < seq_of_subtask.size(); cnt++)
        {
          // distinguish operator
          if (seq_of_subtask.at(cnt) == "+")
          {  // sequential subtask
            ROS_INFO("+\n");
            // add state to main function
            std::string state1 = ArrayPop(stack, &sp);
            std::string state2 = ArrayPop(stack, &sp);
            if (state2 == "False")
            {  // There is one state
              index = AddOneStateSQ(state1);
            }
            else
            {
              ROS_INFO("state1:%s, state2:%s", state1.c_str(), state2.c_str());
              index = AddStateSQ(state2, state1);
              ArrayPush(stack, IntToString(index), &sp, N);
            }
          }
          else if (seq_of_subtask.at(cnt) == "|")
          {  // concurrent subtask
            // create concurrence container
            std::string state1 = ArrayPop(stack, &sp);
            std::string state2 = ArrayPop(stack, &sp);
            num_of_subtasks = GenerateCC(state2, state1, cc_count);
            // add state to main function
            index = AddStateCC(cc_count, num_of_subtasks);
            ArrayPush(stack, IntToString(index), &sp, N);
            cc_count++;
          }
          else
          {
            ArrayPush(stack, seq_of_subtask.at(cnt), &sp, N);
          }
        }
        break;
        
      }
    }
  }
  else
  {
    ROS_INFO("Failed to call service tms_db_get_task_data.\n");
    res.result = 0;  // false1
    return true;
  }

  ROS_TMS_TS::GenerateScript();
  ROS_TMS_TS::ExeScript();

  res.result = 1;  // success
  return true;
}
