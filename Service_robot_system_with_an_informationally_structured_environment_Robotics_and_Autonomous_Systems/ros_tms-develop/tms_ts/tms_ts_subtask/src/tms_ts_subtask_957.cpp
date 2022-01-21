#include <tms_ts_subtask/tms_ts_subtask.h>

using namespace std;
using namespace boost;

double sp5arm_init_arg[26] = {
    0.0, 0.0, 0.175, 0.175, // waist
    0.0, -0.17, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.175, -0.2, 0.175, 0.175, // right arm
    0.0, -0.17, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.175, -0.2, 0.175, 0.175 // left arm
};                                 // rad

tms_rp::TmsRpSubtask *tms_rp::TmsRpSubtask::instance()
{
  static tms_rp::TmsRpSubtask *instance = new tms_rp::TmsRpSubtask();
  return instance;
}

tms_rp::TmsRpSubtask::TmsRpSubtask()
{
  sid_ = 100000;
  rp_subtask_server = nh1.advertiseService("rp_cmd", &TmsRpSubtask::subtask, this);

  rp_cmd_client = nh1.serviceClient<tms_msg_rp::rp_cmd>("rp_cmd");
  get_data_client_ = nh1.serviceClient<tms_msg_db::TmsdbGetData>("/tms_db_reader");
  sp5_control_client_ = nh1.serviceClient<tms_msg_rc::rc_robot_control>("sp5_control");
  sp5_virtual_control_client = nh1.serviceClient<tms_msg_rc::rc_robot_control>("sp5_virtual_control");
  subtask_pick_client = nh1.serviceClient<tms_msg_rp::rp_pick>("subtask_pick");
  subtask_place_client = nh1.serviceClient<tms_msg_rp::rp_place>("subtask_place");
  subtask_release_client = nh1.serviceClient<tms_msg_rp::rp_release>("subtask_release");
  //  kxp_virtual_control_client    = nh1.serviceClient<tms_msg_rc::rc_robot_control>("kxp_virtual_control");
  //  kxp_mbase_client       ama       = nh1.serviceClient<tms_msg_rc::tms_rc_pmove>("pmove");
  //  v_kxp_mbase_client            = nh1.serviceClient<tms_msg_rc::tms_rc_pmove>("virtual_pmove");
  //  kxp_setpose_client            = nh1.serviceClient<std_srvs::Empty>("vicon_psetodom");
  //  katana_client                 = nh1.serviceClient<tms_msg_rc::katana_pos_array>("katana_move_angle_array");
  //  v_katana_client               =
  //  nh1.serviceClient<tms_msg_rc::katana_pos_array>("virtual_katana_move_angle_array");
  //  kobuki_virtual_control_client = nh1.serviceClient<tms_msg_rc::rc_robot_control>("kobuki_virtual_control");
  //  kobuki_actual_control_client  = nh1.serviceClient<tms_msg_rc::rc_robot_control>("kobuki_control");
  //  mkun_virtual_control_client   = nh1.serviceClient<tms_msg_rc::rc_robot_control>("mimamorukun_virtual_control");
  mkun_control_client = nh1.serviceClient<tms_msg_rc::rc_robot_control>("mkun_goal_pose");
  double_goal_pub = nh1.advertise<geometry_msgs::PoseStamped>("/tms_rc_double/room957/move_base_simple/goal", 10);

  turtlebot3_control_client = nh1.serviceClient<tms_msg_rc::rc_robot_control>("turtlebot3_goal_pose");
  voronoi_path_planning_client_ =
      nh1.serviceClient<tms_msg_rp::rps_voronoi_path_planning>("rps_voronoi_path_planning");
  give_obj_client = nh1.serviceClient<tms_msg_rp::rps_goal_planning>("rps_give_obj_pos_planning");
  //  refrigerator_client           = nh1.serviceClient<tms_msg_rs::rs_home_appliances>("refrigerator_controller");
  state_client = nh1.serviceClient<tms_msg_ts::ts_state_control>("ts_state_control");

  db_pub = nh1.advertise<tms_msg_db::TmsdbStamped>("tms_db_data", 10);
  //  kobuki_sound                  = nh1.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1);
  //  kobuki_motorpower             = nh1.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 1);

  //  sensing_sub                   = nh1.subscribe("/ods_realtime_persondt", 10, &TmsRpSubtask::sensingCallback, this);
}

tms_rp::TmsRpSubtask::~TmsRpSubtask()
{
}

double tms_rp::TmsRpSubtask::RadianNormalize(double rad)
{
  while (rad > PI)
    rad -= 2 * PI;
  while (rad < -PI)
    rad += 2 * PI;

  return rad;
}

double tms_rp::TmsRpSubtask::DiffRadian(double rad1, double rad2)
{
  rad1 = RadianNormalize(rad1);
  rad2 = RadianNormalize(rad2);

  double rad = rad2 - rad1;
  rad = RadianNormalize(rad);

  return rad;
}

double tms_rp::TmsRpSubtask::distance(double x1, double y1, double x2, double y2)
{
  double dis;
  dis = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  return dis;
}

std::string tms_rp::TmsRpSubtask::DoubleToString(double number)
{
  std::stringstream ss;
  ss << number;
  return ss.str();
}

void tms_rp::TmsRpSubtask::send_rc_exception(int error_type)
{
  tms_msg_ts::ts_state_control rc_s_srv;
  rc_s_srv.request.type = 2; // for subtask state update;
  rc_s_srv.request.state = -1;

  switch (error_type)
  {
  case 0:
    rc_s_srv.request.error_msg = "RC exception. Cannot set odometry";
    break;
  case 1:
    rc_s_srv.request.error_msg = "RC exception. Cannot move vehicle";
    break;
  case 2:
    rc_s_srv.request.error_msg = "RC exception. Cannot move left arm";
    break;
  case 3:
    rc_s_srv.request.error_msg = "RC exception. Cannot move lumba";
    break;
  case 4:
    rc_s_srv.request.error_msg = "RC exception. Cannot move left gripper";
    break;
  case 5:
    rc_s_srv.request.error_msg = "RC exception. Cannot run command sync obj";
    break;
  case 6:
    rc_s_srv.request.error_msg = "RC exception. Cannot run command calc background";
    break;
  case 7:
    rc_s_srv.request.error_msg = "RC exception. Cannot move along trajectory";
    break;
  }
  state_client.call(rc_s_srv);
}

bool tms_rp::TmsRpSubtask::get_robot_pos(bool type, int robot_id, std::string &robot_name, tms_msg_rp::rps_voronoi_path_planning &rp_srv)
{
  tms_msg_db::TmsdbGetData srv;

  // get robot's present location from TMSDB
  srv.request.tmsdb.id = robot_id;

  if (type == true)                  // production version
    srv.request.tmsdb.sensor = 3001; // vicon system
  else                               // simulation
    srv.request.tmsdb.sensor = 3005; // fake odometry
  //TODO
  //modify to use pozyx as a sensor for robot 
  int ref_i = 0;
  if (get_data_client_.call(srv))
  {
    if (srv.response.tmsdb.empty())
      return false;
    for (int j = 0; j < srv.response.tmsdb.size() - 1; j++)
    {
      if (srv.response.tmsdb[j].time < srv.response.tmsdb[j + 1].time)
        ref_i = j + 1;
    }

    robot_name = srv.response.tmsdb[0].name;
    rp_srv.request.start_pos.x = srv.response.tmsdb[0].x;
    rp_srv.request.start_pos.y = srv.response.tmsdb[0].y;
    rp_srv.request.start_pos.z = 0.0;
    rp_srv.request.start_pos.th = srv.response.tmsdb[0].ry;  // =yaw
    rp_srv.request.start_pos.roll = 0.0;                     // srv.response.tmsdb[0].rr
    rp_srv.request.start_pos.pitch = 0.0;                    // srv.response.tmsdb[0].rp
    rp_srv.request.start_pos.yaw = srv.response.tmsdb[0].ry; // srv.response.tmsdb[0].ry

    ROS_INFO("current position %f, %f", rp_srv.request.start_pos.x, rp_srv.request.start_pos.y);

    // set odom for production version
    if (type == true && (robot_id == 2002 || robot_id == 2003))
    {
      ROS_INFO("setOdom");
      double arg[1] = {0.0};
      if (!sp5_control(true, UNIT_ALL, SET_ODOM, 1, arg))
        send_rc_exception(0);
    }
  }
  else
  {
    ROS_INFO("Failed to call service tms_db_get_current_robot_data.\n");

    // use odometry when the robot cannot get data from vicon system
    if (type == true)
    {
      srv.request.tmsdb.sensor = 3003; // odometory

      if (!get_data_client_.call(srv))
      {
        ROS_INFO("Failed to get robot current data\n");
        return false;
      }
      ref_i = 0;

      for (int j = 0; j < srv.response.tmsdb.size() - 1; j++)
      {
        if (srv.response.tmsdb[j].time < srv.response.tmsdb[j + 1].time)
          ref_i = j + 1;
      }

      robot_name = srv.response.tmsdb[ref_i].name;
      rp_srv.request.start_pos.x = srv.response.tmsdb[ref_i].x;
      rp_srv.request.start_pos.y = srv.response.tmsdb[ref_i].y;
      rp_srv.request.start_pos.z = 0.0;
      rp_srv.request.start_pos.th = srv.response.tmsdb[ref_i].ry;
      rp_srv.request.start_pos.roll = 0.0;                         // srv.response.tmsdb[ref_i].rr
      rp_srv.request.start_pos.pitch = 0.0;                        // srv.response.tmsdb[ref_i].rp
      rp_srv.request.start_pos.yaw = srv.response.tmsdb[ref_i].ry; // srv.response.tmsdb[ref_i].ry
    }
    return false;
  }
  return true;
}

bool tms_rp::TmsRpSubtask::subtask(tms_msg_rp::rp_cmd::Request &req, tms_msg_rp::rp_cmd::Response &res)
{
  SubtaskData sd;
  //    sd.type = grasp::TmsRpBar::production_version_;
  bool type;
  // sd.type = type; //sim = false , real = true
  sd.type = req.type;
  sd.robot_id = req.robot_id;
  sd.arg_type = (int)req.arg.at(0);
  sd.v_arg.clear();

  for (int i = 0; i < req.arg.size(); i++)
    sd.v_arg.push_back(req.arg.at(i));

  switch (req.command)
  {
  case 9001: // move
  {
    ROS_INFO("[tms_rp]move command\n");
    boost::thread mo_th(boost::bind(&TmsRpSubtask::move, this, sd));
    break;
  }
  case 9002: // grasp
  {
    ROS_INFO("[tms_rp]grasp command\n");
    boost::thread gr_th(boost::bind(&TmsRpSubtask::grasp, this, sd));
    break;
  }
  case 9003: // release
  {
    ROS_INFO("[tms_rp]release command\n");
    boost::thread re_th(boost::bind(&TmsRpSubtask::release, this, sd));
    break;
  }
  case 9004: // open the refrigerator
  {
    ROS_INFO("[tms_rp]open command\n");
    if (sd.robot_id != 2009)
    {
      ROS_ERROR("It is an illegal robot_id.\n");
      res.result = 0;
      return false;
    }
    // boost::thread or_th(boost::bind(&TmsRpSubtask::open_ref, this));
    break;
  }
  case 9005: // close the refrigerator
  {
    ROS_INFO("[tms_rp]close command\n");
    if (sd.robot_id != 2009)
    {
      ROS_ERROR("It is an illegal robot_id.\n");
      res.result = 0;
      return false;
    }
    // boost::thread cr_th(boost::bind(&TmsRpSubtask::close_ref, this));
    break;
  }
  case 9006: // kobuki_random_walker
  {
    ROS_INFO("[tms_rp]random_move command\n");
    if (sd.robot_id != 2005)
    {
      ROS_ERROR("It is an illegal robot_id.\n");
      res.result = 0;
      return false;
    }
    // boost::thread kw_th(boost::bind(&TmsRpSubtask::random_move, this));
    break;
  }
  case 9007: // sensing using kinect
  {
    ROS_INFO("[tms_rp]sensing command\n");
    if (sd.robot_id != 2005)
    {
      ROS_ERROR("It is an illegal robot_id.\n");
      res.result = 0;
      return false;
    }
    // boost::thread se_th(boost::bind(&TmsRpSubtask::sensing, this));
    break;
  }
  default:
  {
    ROS_ERROR("No such subtask (ID : %d)\n", req.command);
    return false;
  }
  }
  ROS_INFO("Exit from subtask function.");
  res.result = 1;
  return true;
}

// service caller to sp5_(virtual_)control
bool tms_rp::TmsRpSubtask::sp5_control(bool type, int unit, int cmd, int arg_size, double *arg)
{
  tms_msg_rc::rc_robot_control sp_control_srv;

  sp_control_srv.request.unit = unit;
  sp_control_srv.request.cmd = cmd;
  sp_control_srv.request.arg.resize(arg_size);

  for (int i = 0; i < sp_control_srv.request.arg.size(); i++)
  {
    sp_control_srv.request.arg[i] = arg[i];
    // ROS_INFO("arg[%d]=%f", i, sp_control_srv.request.arg[i]);
  }

  if (type == true)
  {
    if (sp5_control_client_.call(sp_control_srv))
    {
      ROS_INFO("result: %d", sp_control_srv.response.result);
      if (sp_control_srv.response.result == 1)
        return true;
      else
        return false;
    }
    else
    {
      ROS_ERROR("Failed to call service sp5_control");
      return false;
    }
  }
  else
  {
    if (sp5_virtual_control_client.call(sp_control_srv))
    {
      ROS_INFO("result: %d", sp_control_srv.response.result);
      return true;
    }
    else
    {
      ROS_ERROR("Failed to call service sp5_control");
      return false;
    }
  }
}

//------------------------------------------------------------------------------
// service caller to kxp_(virtual_)control
// bool tms_rp::TmsRpSubtask::kxp_control(bool type, int unit, int cmd, int arg_size, double* arg) {
//	if (!type) {
//		tms_msg_rc::rc_robot_control kxp_control_srv;
//		kxp_control_srv.request.unit = unit;
//		kxp_control_srv.request.cmd  = cmd;
//		kxp_control_srv.request.arg.resize(arg_size);
//		for (int i=0; i<kxp_control_srv.request.arg.size(); i++) {
//			kxp_control_srv.request.arg[i] = arg[i];
//		}

//		if (kxp_virtual_control_client.call(kxp_control_srv)) {
//			ROS_INFO("result: %d", kxp_control_srv.response.result);
//			return true;
//		} else {
//			ROS_ERROR("Failed to call service kxp_control");
//			return false;
//		}
//	}
//}

//------------------------------------------------------------------------------
// void tms_rp::TmsRpSubtask::sensingCallback(const tms_msg_ss::ods_person_dt::ConstPtr& msg) {
//	double dis = distance(msg->p1_x, msg->p1_y, msg->p2_x, msg->p2_y);
//	if (1.5 <= dis && dis <= 1.7) {
//		tms_msg_ts::ts_state_control s_srv;
//		s_srv.request.type = 1; // for subtask state update;
//		s_srv.request.state = 0;

//		ROS_INFO("Person Detection System returns True!!!");
//		kobuki_msgs::Sound sound_msg;
//		sound_msg.value = 4;
//		kobuki_sound.publish(sound_msg);

//		kobuki_msgs::MotorPower motor_msg;
//		motor_msg.state = 0;
//		kobuki_motorpower.publish(motor_msg);

//		s_srv.request.error_msg = "Kobuki found someone lying!!";
//		state_client.call(s_srv);
//	}
//}

//------------------------------------------------------------------------------
bool tms_rp::TmsRpSubtask::update_obj(int id, double x, double y, double z, double rr, double rp, double ry, int place, int sensor, int state, std::string note)
{
  tms_msg_db::Tmsdb assign_data;
  ros::Time now = ros::Time::now() + ros::Duration(9 * 60 * 60); // GMT +9
  assign_data.time = boost::posix_time::to_iso_extended_string(now.toBoost());
  assign_data.id = id;

  if (x != -1.0)
    assign_data.x = x;
  if (y != -1.0)
    assign_data.y = y;
  if (z != -1.0)
    assign_data.z = z;
  if (rr != -1.0)
    assign_data.rr = rr;
  if (rp != -1.0)
    assign_data.rp = rp;
  if (ry != -1.0)
    assign_data.ry = ry;
  if (place != -1)
    assign_data.place = place;
  if (sensor != -1)
    assign_data.sensor = sensor;
  if (state != -1)
    assign_data.state = state;
  if (note != "")
    assign_data.note = note;

  tms_msg_db::TmsdbStamped db_msg;
  db_msg.header.frame_id = "/world";
  db_msg.header.stamp = ros::Time::now() + ros::Duration(9 * 60 * 60); // GMT +9
  db_msg.tmsdb.push_back(assign_data);

  for (int i = 0; i < 3; i++)
    db_pub.publish(db_msg);
}

//------------------------------------------------------------------------------
// bool tms_rp::TmsRpSubtask::kxp_set_odom(void) {
//	std_srvs::Empty empty;
//	return kxp_setpose_client.call(empty);
//}

//------------------------------------------------------------------------------
// Subtask functions that is started in a thread
bool tms_rp::TmsRpSubtask::move(SubtaskData sd)
{
  ROS_INFO("type: %u, robotID: %d, arg: %f, arg_type %d", sd.type, sd.robot_id, sd.v_arg.at(0), sd.arg_type);
  tms_msg_db::TmsdbGetData srv;
  tms_msg_rp::rps_voronoi_path_planning rp_srv;
  tms_msg_ts::ts_state_control s_srv;
  s_srv.request.type = 1; // for subtask state update;
  s_srv.request.state = 0;

  rp_srv.request.robot_id = sd.robot_id;
  std::string robot_name("");

  for (int cnt = 0; cnt < 5; cnt++)
  {
    if (get_robot_pos(sd.type, sd.robot_id, robot_name, rp_srv))
      break;
  }

  srv.request.tmsdb.id = sd.arg_type;

  if (sd.arg_type == -1) // move (x,y,th)
  {
    rp_srv.request.goal_pos.x = sd.v_arg.at(1);
    rp_srv.request.goal_pos.y = sd.v_arg.at(2);
    rp_srv.request.goal_pos.z = 0.0;
    rp_srv.request.goal_pos.th = sd.v_arg.at(3);
    rp_srv.request.goal_pos.roll = 0.0;
    rp_srv.request.goal_pos.pitch = 0.0;
    rp_srv.request.goal_pos.yaw = sd.v_arg.at(3);
  }
  else if (sd.arg_type > 1000 && sd.arg_type < 2000) // person
  {
    // srv.request.tmsdb.sensor = 3001;
    srv.request.tmsdb.state = 1;
    ROS_INFO("person ID:%d", sd.arg_type);
    if (!get_data_client_.call(srv))
    {
      s_srv.request.error_msg = "Failed to get data";
      state_client.call(s_srv);
      return false;
    }
    double person_x = srv.response.tmsdb[0].x;
    double person_y = srv.response.tmsdb[0].y;
    double person_yaw = srv.response.tmsdb[0].ry; // + 1.570796;
    int place = srv.response.tmsdb[0].place;
    ROS_INFO("x=%f y=%f ry=%f place=%d", person_x, person_y, person_yaw, place);

    if (place == 6017)
    { // in the bed
      ROS_INFO("humen in the bed");
      srv.request.tmsdb.id = 6017 + sid_;

      if (!get_data_client_.call(srv))
      {
        s_srv.request.error_msg = "Failed to call service tms_db_get_current_furniture_data";
        state_client.call(s_srv);
        return false;
      }
      std::string etcdata = srv.response.tmsdb[0].etcdata;
      ROS_INFO("etc_data = %s", etcdata.c_str());
      std::vector<std::string> v_etcdata;
      v_etcdata.clear();
      boost::split(v_etcdata, etcdata, boost::is_any_of(";"));

      for (int i = 0; i < v_etcdata.size(); i++)
      {
        ROS_INFO("v_etcdata[%d]=%s", i, v_etcdata.at(i).c_str());
      }

      int i = 0;
      while (robot_name != v_etcdata.at(i))
      {
        i += 2;
        if (i >= v_etcdata.size())
        {
          s_srv.request.error_msg = "This robot cannot move to the slate point";
          state_client.call(s_srv);
          return false;
        }
      }

      std::string argdata = v_etcdata.at(i + 1);
      std::vector<std::string> v_argdata;
      v_argdata.clear();
      boost::split(v_argdata, argdata, boost::is_any_of(",")); // v_argdata[0, 1, 2] = [goal_x, goal_y, goal_th]

      if (v_argdata.size() != 3)
      {
        s_srv.request.error_msg = "There are incorrect data in DB! Check furniture's etcdata";
        state_client.call(s_srv);
        return false;
      }
      // string to double
      std::stringstream ss;
      double d_x, d_y, d_th;

      // goal_x
      ss << v_argdata.at(0);
      ss >> d_x; // string to double
      rp_srv.request.goal_pos.x = d_x;
      ROS_INFO("goal_x = [%f]  ", d_x);

      // goal_y
      ss.clear();
      ss.str("");
      ss << v_argdata.at(1);
      ss >> d_y;
      rp_srv.request.goal_pos.y = d_y;
      ROS_INFO("goal_y = [%f]  ", d_y);

      // goal_th
      ss.clear();
      ss.str("");
      ss << v_argdata.at(2);
      ss >> d_th;
      rp_srv.request.goal_pos.th = d_th;
      ROS_INFO("goal_th = [%f]  ", d_th);

      rp_srv.request.goal_pos.z = 0.0;
      rp_srv.request.goal_pos.roll = 0.0;
      rp_srv.request.goal_pos.pitch = 0.0;
      rp_srv.request.goal_pos.yaw = 0.0;
      // rp_srv.request.goal_pos.x = 10.3;
      // rp_srv.request.goal_pos.y = 3.7;
      // rp_srv.request.goal_pos.z = 0.0;
      // rp_srv.request.goal_pos.th = -1.57079633;
      // rp_srv.request.goal_pos.roll = 0.0;
      // rp_srv.request.goal_pos.pitch = 0.0;
      // rp_srv.request.goal_pos.yaw = 0.0;
    }
    else
    {
      rp_srv.request.goal_pos.x = person_x + 0.5 * cos(person_yaw);
      rp_srv.request.goal_pos.y = person_y + 0.5 * sin(person_yaw);

      if (person_yaw > 0)
        rp_srv.request.goal_pos.th = person_yaw - 3.141592;
      else
        rp_srv.request.goal_pos.th = person_yaw + 3.141592;

      rp_srv.request.goal_pos.z = 0.0;
      rp_srv.request.goal_pos.roll = 0.0;
      rp_srv.request.goal_pos.pitch = 0.0;
      rp_srv.request.goal_pos.yaw = 0.0;
    }
  }
  else if ((sd.arg_type > 2000 && sd.arg_type < 3000) || (sd.arg_type > 6000 && sd.arg_type < 7000)) // RobotID or FurnitureID
  {
    srv.request.tmsdb.id = sd.arg_type + sid_;

    if (!get_data_client_.call(srv))
    {
      s_srv.request.error_msg = "Failed to call service tms_db_get_current_furniture_data";
      state_client.call(s_srv);
      return false;
    }
    // analyze etcdata and get goal position
    std::string etcdata = srv.response.tmsdb[0].etcdata;
    std::vector<std::string> v_etcdata;
    v_etcdata.clear();
    boost::split(v_etcdata, etcdata, boost::is_any_of(";"));
    ROS_INFO("robot name = %s", robot_name.c_str());
    int i = 0;
    while (robot_name != v_etcdata.at(i))
    {
      i += 2;
      if (i >= v_etcdata.size())
      {
        s_srv.request.error_msg = "This robot cannot move to the slate point";
        state_client.call(s_srv);
        return false;
      }
    }
    std::string argdata = v_etcdata.at(i + 1);
    std::vector<std::string> v_argdata;
    v_argdata.clear();
    boost::split(v_argdata, argdata, boost::is_any_of(",")); // v_argdata[0, 1, 2] = [goal_x, goal_y, goal_th]

    if (v_argdata.size() != 3)
    {
      s_srv.request.error_msg = "There are incorrect data in DB. Check furniture's etcdata";
      state_client.call(s_srv);
      return false;
    }
    // string to double
    std::stringstream ss;
    double d_x, d_y, d_th;

    // goal_x
    ss << v_argdata.at(0);
    ss >> d_x;
    rp_srv.request.goal_pos.x = d_x;
    ROS_INFO("goal_x = [%f]  ", d_x);

    // goal_y
    ss.clear();
    ss.str("");
    ss << v_argdata.at(1);
    ss >> d_y;
    rp_srv.request.goal_pos.y = d_y;
    ROS_INFO("goal_y = [%f]  ", d_y);

    // goal_th
    ss.clear();
    ss.str("");
    ss << v_argdata.at(2);
    ss >> d_th;
    rp_srv.request.goal_pos.th = d_th;
    ROS_INFO("goal_th = [%f]\n", d_th);

    rp_srv.request.goal_pos.z = 0.0;
    rp_srv.request.goal_pos.roll = 0.0;
    rp_srv.request.goal_pos.pitch = 0.0;
    rp_srv.request.goal_pos.yaw = 0.0;
  }
  else if (sd.arg_type > 7000 && sd.arg_type < 8000) // ObjectID
  {
    // srv.request.tmsdb.sensor = 3018;
    srv.request.tmsdb.state = 1;
    ROS_INFO("Argument IDtype is Object%d!\n", sd.arg_type);
    if (!get_data_client_.call(srv))
    {
      s_srv.request.error_msg = "Failed to get data";
      state_client.call(s_srv);
      return false;
    }
    ROS_INFO("place is %d\n", srv.response.tmsdb[0].place);
    if (!((srv.response.tmsdb[0].place > 2000 && srv.response.tmsdb[0].place < 3000) || (srv.response.tmsdb[0].place > 6000 && srv.response.tmsdb[0].place < 7000)))
    {
      s_srv.request.error_msg = "Object is in unexpected place";
      state_client.call(s_srv);
      return false;
    }
    srv.request.tmsdb.id = srv.response.tmsdb[0].place + sid_;

    if (!get_data_client_.call(srv))
    {
      s_srv.request.error_msg = "Failed to call service tms_db_get_current_furniture_data";
      state_client.call(s_srv);
      return false;
    }
    std::string etcdata = srv.response.tmsdb[0].etcdata;
    ROS_INFO("etc_data = %s", etcdata.c_str());
    std::vector<std::string> v_etcdata;
    v_etcdata.clear();
    boost::split(v_etcdata, etcdata, boost::is_any_of(";"));

    for (int i = 0; i < v_etcdata.size(); i++)
    {
      ROS_INFO("v_etcdata[%d]=%s", i, v_etcdata.at(i).c_str());
    }

    int i = 0;
    while (robot_name != v_etcdata.at(i))
    {
      i += 2;
      if (i >= v_etcdata.size())
      {
        s_srv.request.error_msg = "This robot cannot move to the slate point";
        state_client.call(s_srv);
        return false;
      }
    }

    std::string argdata = v_etcdata.at(i + 1);
    std::vector<std::string> v_argdata;
    v_argdata.clear();
    boost::split(v_argdata, argdata, boost::is_any_of(",")); // v_argdata[0, 1, 2] = [goal_x, goal_y, goal_th]

    if (v_argdata.size() != 3)
    {
      s_srv.request.error_msg = "There are incorrect data in DB! Check furniture's etcdata";
      state_client.call(s_srv);
      return false;
    }
    // string to double
    std::stringstream ss;
    double d_x, d_y, d_th;

    // goal_x
    ss << v_argdata.at(0);
    ss >> d_x; // string to double
    rp_srv.request.goal_pos.x = d_x;
    ROS_INFO("goal_x = [%f]  ", d_x);

    // goal_y
    ss.clear();
    ss.str("");
    ss << v_argdata.at(1);
    ss >> d_y;
    rp_srv.request.goal_pos.y = d_y;
    ROS_INFO("goal_y = [%f]  ", d_y);

    // goal_th
    ss.clear();
    ss.str("");
    ss << v_argdata.at(2);
    ss >> d_th;
    rp_srv.request.goal_pos.th = d_th;
    ROS_INFO("goal_th = [%f]  ", d_th);

    rp_srv.request.goal_pos.z = 0.0;
    rp_srv.request.goal_pos.roll = 0.0;
    rp_srv.request.goal_pos.pitch = 0.0;
    rp_srv.request.goal_pos.yaw = 0.0;
  }
  else
  {
    s_srv.request.error_msg = "An Illegal arg_type number";
    state_client.call(s_srv);
    return false;
  }

  ROS_INFO("start[%f,%f,%f], goal[%f,%f,%f]\n", rp_srv.request.start_pos.x, rp_srv.request.start_pos.y, rp_srv.request.start_pos.th, rp_srv.request.goal_pos.x, rp_srv.request.goal_pos.y, rp_srv.request.goal_pos.th);

  // =====call service "voronoi_path_planning"=====
  int i = 1;
/*
  if (!voronoi_path_planning_client_.call(rp_srv))
  {
    s_srv.request.error_msg = "Failed to call service /rps_voronoi_path_planning";
    state_client.call(s_srv);
    return false;
  }
  if (rp_srv.response.VoronoiPath.empty())
  {
    s_srv.request.error_msg = "Planned path is empty";
    state_client.call(s_srv);
    return false;
  }
 */ 
  while (1)
  {
    ROS_INFO("result:%d, message:%s", rp_srv.response.success, rp_srv.response.message.c_str());
    // call virtual_controller

    switch (sd.robot_id)
    {
    case 2002: // smartpal5_1
    case 2003: // smartpal5_2
    {
      double arg[3];
      arg[0] = rp_srv.response.VoronoiPath[i].x;
      arg[1] = rp_srv.response.VoronoiPath[i].y;
      arg[2] = rp_srv.response.VoronoiPath[i].th;
      ROS_INFO("type:%d, arg0: %f,1: %f,2: %f", sd.type, arg[0], arg[1], arg[2]);
      // fix over 180 deg bug

      if (sd.type)
      {
        if (((rp_srv.response.VoronoiPath[i - 1].th > HALF_PI && rp_srv.response.VoronoiPath[i - 1].th <= PI) &&
             (rp_srv.response.VoronoiPath[i].th >= -PI && rp_srv.response.VoronoiPath[i].th < 0)) ||
            ((rp_srv.response.VoronoiPath[i].th > 0 && rp_srv.response.VoronoiPath[i].th <= PI) &&
             (rp_srv.response.VoronoiPath[i - 1].th >= -PI && rp_srv.response.VoronoiPath[i - 1].th < -HALF_PI)))
        {
          double g_ang = rp_srv.response.VoronoiPath[i].th - rp_srv.response.VoronoiPath[i - 1].th;

          if (g_ang > PI)
            g_ang = g_ang - 2 * PI;
          else if (g_ang < -PI)
            g_ang = g_ang + 2 * PI;

          double tmp_arg[3] = {0.0, 0.0, g_ang};

          ROS_INFO("goal_ang=%f", g_ang);

          if (!sp5_control(sd.type, UNIT_VEHICLE, CMD_MOVE_REL, 3, tmp_arg))
          {
            return false;
          }
        }
        else
        {
          if (!sp5_control(sd.type, UNIT_VEHICLE, CMD_MOVE_ABS, 3, arg))
          {
            send_rc_exception(1);
            return false;
          }
        }
        for (int i = 0; i < 5; i++)
        {
          if (sp5_control(sd.type, UNIT_ALL, SET_ODOM, 1, arg))
            break;
          else if (i == 4)
            send_rc_exception(0);
        }
      }
      else //if sd.type == false
      {
        if (!sp5_control(sd.type, UNIT_VEHICLE, CMD_MOVE_ABS, 3, arg))
        {
          send_rc_exception(1);
          return false;
        }
        usleep(300000); //sleep(0.3);
        usleep(300000);
      }

      i++;
      if (i == 2 && rp_srv.response.VoronoiPath.size() <= 2)
      {
        i++;
      }

      // Update Robot Path Planning
      if (i == 3)
      {
        for (int cnt = 0; cnt < 5; cnt++)
        {
          if (get_robot_pos(sd.type, sd.robot_id, robot_name, rp_srv))
          {
            break;
          }
        }

        // end determination
        double error_x, error_y, error_th, error_dis;

        error_x = fabs(rp_srv.request.start_pos.x - rp_srv.request.goal_pos.x);
        error_y = fabs(rp_srv.request.start_pos.y - rp_srv.request.goal_pos.y);
        error_th = fabs(rp_srv.request.start_pos.th - rp_srv.request.goal_pos.th);
        error_dis = distance(rp_srv.request.start_pos.x, rp_srv.request.start_pos.y, rp_srv.request.goal_pos.x, rp_srv.request.goal_pos.y);

        if (error_th > PI)
          error_th = error_th - 2 * PI;
        else if (error_th < -PI)
          error_th = error_th + 2 * PI;

        ROS_INFO("error_x:%f,error_y:%f,error_th:%f, error_dis:%f", error_x, error_y, error_th, error_dis);
        if (error_x <= 0.01 && error_y <= 0.01 && (error_th >= -0.05 && error_th <= 0.05))
        {
          ROS_INFO("finish");
          s_srv.request.state = 1;
          state_client.call(s_srv);
          return true;
        }

        rp_srv.response.VoronoiPath.clear();
        voronoi_path_planning_client_.call(rp_srv);

        if (rp_srv.response.VoronoiPath.empty())
        {
          s_srv.request.error_msg = "Planned path is empty";
          state_client.call(s_srv);
          return false;
        }

        i = 1;
      }
      break;
    }
    case 2007: // mimamorukun
    {
      tms_msg_rc::rc_robot_control mkun_srv;
      mkun_srv.request.unit = 1;
      mkun_srv.request.cmd = 0;
      mkun_srv.request.arg.resize(3);
      int size = rp_srv.response.VoronoiPath.size();

      while (1)
      {
        tms_msg_db::TmsdbGetData srv;
        srv.request.tmsdb.id = 2007;
        srv.request.tmsdb.sensor = 3001;
        if (!get_data_client_.call(srv))
        {
          s_srv.request.error_msg = "Failed to get data";
          state_client.call(s_srv);
          return false;
        }
        ROS_INFO("now x:%f y:%f th:%f", srv.response.tmsdb[0].x, srv.response.tmsdb[0].y, srv.response.tmsdb[0].ry);
        ROS_INFO("i:%d x:%f y:%f th:%f", i, rp_srv.response.VoronoiPath[i].x, rp_srv.response.VoronoiPath[i].y, rp_srv.response.VoronoiPath[i].th);
        double goal_dis = distance(srv.response.tmsdb[0].x, srv.response.tmsdb[0].y, rp_srv.request.goal_pos.x, rp_srv.request.goal_pos.y);
        double dis = distance(srv.response.tmsdb[0].x, srv.response.tmsdb[0].y, rp_srv.response.VoronoiPath[i].x, rp_srv.response.VoronoiPath[i].y);
        double goal_rad = DiffRadian(srv.response.tmsdb[0].ry, rp_srv.request.goal_pos.th);
        double rad = DiffRadian(srv.response.tmsdb[0].ry, rp_srv.response.VoronoiPath[i].th);
        ROS_INFO("goal_dis:%f dis:%f goal_rad:%f rad:%f", goal_dis, dis, goal_rad, rad);

        if (goal_dis <= 0.3)
        {
          while (1)
          {
            tms_msg_db::TmsdbGetData srv;
            srv.request.tmsdb.id = 2007;
            srv.request.tmsdb.sensor = 3001;
            if (!get_data_client_.call(srv))
            {
              s_srv.request.error_msg = "Failed to get data";
              state_client.call(s_srv);
              return false;
            }
            double goal_rad = DiffRadian(srv.response.tmsdb[0].ry, rp_srv.request.goal_pos.th);
            ROS_INFO("goal_rad:%f", goal_rad);
            if (goal_rad > -0.2 && goal_rad < 0.2)
            {
              ROS_INFO("arrive goal");
              mkun_srv.request.cmd = 10;
              if (mkun_control_client.call(mkun_srv))
                ROS_INFO("result: %d", mkun_srv.response.result);
              return true;
              s_srv.request.state = 1;
              state_client.call(s_srv);
            }
            mkun_srv.request.cmd = 1;
            mkun_srv.request.arg[0] = 0;
            mkun_srv.request.arg[1] = 0;
            mkun_srv.request.arg[2] = rp_srv.request.goal_pos.th;
            if (mkun_control_client.call(mkun_srv))
              ROS_INFO("result: %d", mkun_srv.response.result);
            else
            {
              ROS_ERROR("Failed to call service mkun_move");
              return false;
            }
          }
        }

        if ((rad < -1 || rad > 1) && dis > 0.5)
        {
          while (1)
          {
            tms_msg_db::TmsdbGetData srv;
            srv.request.tmsdb.id = 2007;
            srv.request.tmsdb.sensor = 3001;
            if (!get_data_client_.call(srv))
            {
              s_srv.request.error_msg = "Failed to get data";
              state_client.call(s_srv);
              return false;
            }
            rad = DiffRadian(srv.response.tmsdb[0].ry, rp_srv.response.VoronoiPath[i].th);
            ROS_INFO("rad:%f", rad);
            if (rad > -0.5 && rad < 0.5)
              break;

            mkun_srv.request.cmd = 1;
            mkun_srv.request.arg[0] = 0;
            mkun_srv.request.arg[1] = 0;
            mkun_srv.request.arg[2] = rp_srv.response.VoronoiPath[i].th;
            if (mkun_control_client.call(mkun_srv))
              ROS_INFO("result: %d", mkun_srv.response.result);
            else
            {
              ROS_ERROR("Failed to call service mkun_move");
              return false;
            }
          }
        }

        while (dis <= 0.35)
        {
          i += 1;
          if (i >= rp_srv.response.VoronoiPath.size())
          {
            i = rp_srv.response.VoronoiPath.size() - 1;
            break;
          }
          dis = distance(srv.response.tmsdb[0].x, srv.response.tmsdb[0].y, rp_srv.response.VoronoiPath[i].x, rp_srv.response.VoronoiPath[i].y);
          ROS_INFO("dis:%f", dis);
        }

        mkun_srv.request.cmd = 0;
        mkun_srv.request.arg[0] = rp_srv.response.VoronoiPath[i].x;
        mkun_srv.request.arg[1] = rp_srv.response.VoronoiPath[i].y;
        mkun_srv.request.arg[2] = rp_srv.response.VoronoiPath[i].th;
        ROS_INFO("path_size:%lu, i:%d", rp_srv.response.VoronoiPath.size(), i);
        ROS_INFO("[i=%d] goal x=%f, y=%f, yaw=%f", i, mkun_srv.request.arg[0], mkun_srv.request.arg[1], mkun_srv.request.arg[2]);
        if (mkun_control_client.call(mkun_srv))
          ROS_INFO("result: %d", mkun_srv.response.result);
        else
        {
          ROS_ERROR("Failed to call service mimamorukun_move");
          return false;
        }
      }
      break;
    }
    case 2012:
    {
      i++;
      geometry_msgs::PoseStamped double_goal;

      ros::Time now = ros::Time::now() + ros::Duration(9 * 60 * 60); // GMT +9
      double_goal.header.stamp = now;
      double_goal.header.frame_id = "map";
      double_goal.pose.position.x = rp_srv.request.goal_pos.x;
      double_goal.pose.position.y = rp_srv.request.goal_pos.y;
      double_goal.pose.position.z = 0;
      
      geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(rp_srv.request.goal_pos.th);

      double_goal.pose.orientation = quaternion;

      double_goal_pub.publish(double_goal);
      while (1)
      {
        tms_msg_db::TmsdbGetData srv;
        srv.request.tmsdb.id = 2012;
        srv.request.tmsdb.sensor = 3001;
        if (!get_data_client_.call(srv))
        {
          s_srv.request.error_msg = "Failed to get data";
          state_client.call(s_srv);
          return false;
        }
        ROS_INFO("now x:%f y:%f th:%f", srv.response.tmsdb[0].x, srv.response.tmsdb[0].y, srv.response.tmsdb[0].ry);
        double goal_dis = distance(srv.response.tmsdb[0].x, srv.response.tmsdb[0].y, rp_srv.request.goal_pos.x, rp_srv.request.goal_pos.y);
        double goal_rad = DiffRadian(srv.response.tmsdb[0].ry, rp_srv.request.goal_pos.th);
        ROS_INFO("goal_dis:%f goal_rad:%f", goal_dis, goal_rad);

        if (goal_rad > -0.2 && goal_rad < 0.2)
        {
            ROS_INFO("arrive goal");
            // double_srv.request.cmd = 10;
            // if (double_control_client.call(double_srv))
            // ROS_INFO("result: %d", double_srv.response.result);
            s_srv.request.state = 1;
            state_client.call(s_srv);
            return true;
        }
        // if (double_control_client.call(double_srv))
        //     ROS_INFO("result: %d", double_srv.response.result);
        // else
        // {
        //     ROS_ERROR("Failed to call service double_move");
        //     return false;
        // }
      }
      break;
    }
    case 2013: //turtlebot3
    {
      ROS_INFO("switch to turtlebot3 voronoi path planning");
      i++; //for identify others?
      tms_msg_rc::rc_robot_control turtlebot3_srv;
      turtlebot3_srv.request.unit = 1;
      turtlebot3_srv.request.cmd = 0;
      turtlebot3_srv.request.arg.resize(3);
      int size = rp_srv.response.VoronoiPath.size();
      
      for(int j=0;j<size;j++){
	std::cout<<rp_srv.response.VoronoiPath[i].x<<", "<<rp_srv.response.VoronoiPath[i].y<<", "<<rp_srv.response.VoronoiPath[i].z<< std::endl;
      }
      
      while (1)
      {
        tms_msg_db::TmsdbGetData srv;
        srv.request.tmsdb.id = 2013;
        srv.request.tmsdb.sensor = 3001;
        if (!get_data_client_.call(srv))
        {
          s_srv.request.error_msg = "Failed to get data";
          state_client.call(s_srv);
          return false;
        }
        ROS_INFO("now x:%f y:%f th:%f", srv.response.tmsdb[0].x, srv.response.tmsdb[0].y, srv.response.tmsdb[0].ry);
        ROS_INFO("i:%d x:%f y:%f th:%f", i, rp_srv.response.VoronoiPath[i].x, rp_srv.response.VoronoiPath[i].y, rp_srv.response.VoronoiPath[i].th);
        double goal_dis = distance(srv.response.tmsdb[0].x, srv.response.tmsdb[0].y, rp_srv.request.goal_pos.x, rp_srv.request.goal_pos.y);
        double dis = distance(srv.response.tmsdb[0].x, srv.response.tmsdb[0].y, rp_srv.response.VoronoiPath[i].x, rp_srv.response.VoronoiPath[i].y);
        double goal_rad = DiffRadian(srv.response.tmsdb[0].ry, rp_srv.request.goal_pos.th);
        double rad = DiffRadian(srv.response.tmsdb[0].ry, rp_srv.response.VoronoiPath[i].th);
        ROS_INFO("goal_dis:%f dis:%f goal_rad:%f rad:%f", goal_dis, dis, goal_rad, rad);

        if (goal_dis <= 0.2)
        {
          while (1)
          {
            tms_msg_db::TmsdbGetData srv;
            srv.request.tmsdb.id = 2013;
            srv.request.tmsdb.sensor = 3001;
            if (!get_data_client_.call(srv))
            {
              s_srv.request.error_msg = "Failed to get data";
              state_client.call(s_srv);
              return false;
            }
            double goal_rad = DiffRadian(srv.response.tmsdb[0].ry, rp_srv.request.goal_pos.th);
            ROS_INFO("goal_rad:%f", goal_rad);
            if (goal_rad > -0.2 && goal_rad < 0.2)
            {
              ROS_INFO("arrive goal");
              turtlebot3_srv.request.cmd = 10;
              if (turtlebot3_control_client.call(turtlebot3_srv))
                ROS_INFO("result: %d", turtlebot3_srv.response.result);
              s_srv.request.state = 1;
              state_client.call(s_srv);
              return true;
            }
            turtlebot3_srv.request.cmd = 1;
            turtlebot3_srv.request.arg[0] = 0;
            turtlebot3_srv.request.arg[1] = 0;
            turtlebot3_srv.request.arg[2] = rp_srv.request.goal_pos.th;
            if (turtlebot3_control_client.call(turtlebot3_srv))
              ROS_INFO("result: %d", turtlebot3_srv.response.result);
            else
            {
              ROS_ERROR("Failed to call service turtlebot3_move");
              return false;
            }
          }
        } //if goal_dis end

        if ((rad < -1 || rad > 1) && dis > 0.5)
        {
          while (1)
          {
            tms_msg_db::TmsdbGetData srv;
            srv.request.tmsdb.id = 2013;
            srv.request.tmsdb.sensor = 3001;
            if (!get_data_client_.call(srv))
            {
              s_srv.request.error_msg = "Failed to get data";
              state_client.call(s_srv);
              return false;
            }
            rad = DiffRadian(srv.response.tmsdb[0].ry, rp_srv.response.VoronoiPath[i].th);
            ROS_INFO("rad:%f", rad);
            if (rad > -0.5 && rad < 0.5)
              break;

            turtlebot3_srv.request.cmd = 1;
            turtlebot3_srv.request.arg[0] = 0;
            turtlebot3_srv.request.arg[1] = 0;
            turtlebot3_srv.request.arg[2] = rp_srv.response.VoronoiPath[i].th;
            if (turtlebot3_control_client.call(turtlebot3_srv))
              ROS_INFO("result: %d", turtlebot3_srv.response.result);
            else
            {
              ROS_ERROR("Failed to call service turtlebot3_move");
              return false;
            }
          }
        }

        while (dis <= 0.25)
        {
          i += 1;
          if (i >= rp_srv.response.VoronoiPath.size())
          {
            i = rp_srv.response.VoronoiPath.size() - 1;
            break;
          }
          dis = distance(srv.response.tmsdb[0].x, srv.response.tmsdb[0].y, rp_srv.response.VoronoiPath[i].x, rp_srv.response.VoronoiPath[i].y);
          ROS_INFO("dis:%f", dis);
        }

        turtlebot3_srv.request.cmd = 0;
        turtlebot3_srv.request.arg[0] = rp_srv.response.VoronoiPath[i].x;
        turtlebot3_srv.request.arg[1] = rp_srv.response.VoronoiPath[i].y;
        turtlebot3_srv.request.arg[2] = rp_srv.response.VoronoiPath[i].th;
        ROS_INFO("path size:%lu, i:%d", rp_srv.response.VoronoiPath.size(), i);
        ROS_INFO("[i=%d] goal x=%f, y=%f, yaw=%f", i, turtlebot3_srv.request.arg[0], turtlebot3_srv.request.arg[1], turtlebot3_srv.request.arg[2]);
        if (turtlebot3_control_client.call(turtlebot3_srv))
          ROS_INFO("result: %d", turtlebot3_srv.response.result);
        else
        {
          ROS_ERROR("Failed to call service turtlebot3_move");
          return false;
        }
      }
      break;
    }
    default:
    {
      s_srv.request.error_msg = "Unsupported robot in move function";
      state_client.call(s_srv);
      return false;
    }
  } //switch

  } //while
  //	apprise TS_control of succeeding subtask execution
  s_srv.request.state = 1;
  state_client.call(s_srv);
  return true;
}

bool tms_rp::TmsRpSubtask::grasp(SubtaskData sd)
{
  tms_msg_db::TmsdbGetData srv;
  tms_msg_ts::ts_state_control s_srv;
  tms_msg_db::Tmsdb assign_data;
  s_srv.request.type = 1; // for subtask state update;
  s_srv.request.state = 0;
  int furniture_id;

  nh1.setParam("planning_mode", 1); // stop ROS-TMS viewer

  if ((sd.robot_id == 2003) && sd.type == true)
  {
    if (!sp5_control(sd.type, UNIT_ARM_R, CMD_MOVE_ABS, 8, sp5arm_init_arg + 4))
    {
      send_rc_exception(2);
      return false;
    }

    if (!sp5_control(sd.type, UNIT_LUMBA, CMD_MOVE_REL, 4, sp5arm_init_arg))
    {
      send_rc_exception(3);
      return false;
    }

    if (!sp5_control(sd.type, UNIT_GRIPPER_R, CMD_MOVE_ABS, 3, sp5arm_init_arg + 12))
    {
      send_rc_exception(4);
      return false;
    }
  }

  //  std::vector<pathInfo> trajectory;
  int state;
  std::vector<double> begin;
  std::vector<double> end;

  // SET ROBOT
  switch (sd.robot_id)
  {
  case 2003:
    ROS_INFO("ID:%d is selected", sd.robot_id);
    break;
  default:
    ROS_ERROR("An illegal robot id");
    return false;
  }

  // SET OBJECT objectID : 7001~7999
  if (sd.arg_type > 7000 && sd.arg_type < 8000)
  {
    srv.request.tmsdb.id = sd.arg_type;
    srv.request.tmsdb.state = 1;

    if (!get_data_client_.call(srv))
    {
      s_srv.request.error_msg = "Cannot get object's position";
      state_client.call(s_srv);
      return false;
    }
    std::string obj_name = srv.response.tmsdb[0].name;
    furniture_id = srv.response.tmsdb[0].place;

    cout << obj_name << ", " << furniture_id << endl;

    if (furniture_id > 6000 && furniture_id < 7000)
    {
      srv.request.tmsdb.id = furniture_id;

      if (get_data_client_.call(srv))
      {
        std::string furniture_name = srv.response.tmsdb[0].name;
        assign_data = srv.response.tmsdb[0];
      }
    }
  }
  else
  {
    ROS_INFO("Robot cannot grasp this object! Wrong id:%d\n", sd.arg_type);
  }

  if (furniture_id == 2009 && sd.type == false)
  {
    nh1.setParam("/2009_is_real", false);

    tms_msg_db::TmsdbStamped db_msg;
    tms_msg_db::Tmsdb tmpData;
    ros::Time now = ros::Time::now() + ros::Duration(9 * 60 * 60); // GMT +9

    tmpData.time = boost::posix_time::to_iso_extended_string(now.toBoost());
    tmpData.id = 2009;
    tmpData.name = "refrigerator";
    tmpData.place = 5002;
    tmpData.sensor = 3005;
    tmpData.state = 1;
    tmpData.joint = "2.14";

    db_msg.tmsdb.push_back(tmpData);
    db_pub.publish(db_msg);

    sleep(1);

    double arg[3];
    arg[0] = 7.3;
    arg[1] = 5.1;
    arg[2] = 1.57079;
    if (!sp5_control(sd.type, UNIT_VEHICLE, CMD_MOVE_ABS, 3, arg))
    {
      send_rc_exception(1);
      return false;
    }
  }

  cout << "set begin and end posture." << endl;

  if (1)
  {
    switch (sd.robot_id)
    {
    case 2002: // for smartpal simulation
    {
    }
    case 2003:
    {
      tms_msg_rp::rp_pick srv;
      srv.request.robot_id = sd.robot_id;
      srv.request.object_id = sd.arg_type;
      double arg[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.175};

      if (!subtask_pick_client.call(srv))
      {
        s_srv.request.error_msg = "Unsupported robot in grasp function";
        state_client.call(s_srv);
        return false;
      }
      ROS_INFO("Successed to get grasp poses ID:%d", sd.arg_type);

      if (sd.type)
      {
        sleep(1);
        arg[0] = 0.0;
        if (!sp5_control(sd.type, UNIT_ALL, CMD_MOVE_TRAJECTORY, 1, arg))
        {
          send_rc_exception(7);
          return false;
        }
      }

      arg[0] = 0.0;
      arg[1] = -1.4;
      arg[2] = 0.0;
      arg[3] = 0.0;
      arg[4] = 0.0;
      arg[5] = 0.0;
      arg[6] = 0.0;
      arg[7] = 0.175;
      if (!sp5_control(sd.type, UNIT_ARM_L, CMD_MOVE_ABS, 8, arg))
      {
        send_rc_exception(2);
        return false;
      }

      arg[0] = 0.0;
      arg[1] = -0.17;
      arg[2] = 0.0;
      arg[3] = 0.0;
      arg[4] = 0.0;
      arg[5] = 0.0;
      arg[6] = 0.0;
      arg[7] = 0.175;
      if (!sp5_control(sd.type, UNIT_ARM_L, CMD_MOVE_ABS, 8, arg))
      {
        send_rc_exception(2);
        return false;
      }

      std::string note = assign_data.note;
      std::vector<std::string> v_note;
      v_note.clear();
      boost::split(v_note, note, boost::is_any_of(";"));

      ROS_INFO("note(before) = %s", note.c_str());

      for (int i = 0; i < v_note.size(); i += 2)
      {
        ROS_INFO("v_note.at(%d) = %s", i, v_note.at(i).c_str());
        std::stringstream ss;
        ss << v_note.at(i);
        int note_id;
        ss >> note_id;

        if (sd.arg_type == note_id)
        {
          v_note.at(i) = "0";
          break;
        }
      }

      std::stringstream assign_note;
      for (int i = 0; i < v_note.size(); i++)
      {
        assign_note << v_note.at(i);
        if (i != v_note.size() - 1)
        {
          assign_note << ";";
        }
      }

      ROS_INFO("note(after) = %s", note.c_str());

      ros::Time now = ros::Time::now() + ros::Duration(9 * 60 * 60);

      assign_data.time = boost::posix_time::to_iso_extended_string(now.toBoost());
      assign_data.note = assign_note.str();

      tms_msg_db::TmsdbStamped db_msg;
      db_msg.header.frame_id = "/world";
      db_msg.header.stamp = now;

      db_msg.tmsdb.push_back(assign_data);
      db_pub.publish(db_msg);

      break;
    }
    default:
      s_srv.request.error_msg = "Unsupported robot in grasp function";
      state_client.call(s_srv);
      return false;
    }
  }
  nh1.setParam("planning_mode", 0);

  if (furniture_id == 2009 && sd.type == false)
  {
    double arg[3];
    arg[0] = 7.3;
    arg[1] = 5.0;
    arg[2] = 1.57079;
    if (!sp5_control(sd.type, UNIT_VEHICLE, CMD_MOVE_ABS, 3, arg))
    {
      send_rc_exception(1);
      return false;
    }

    sleep(1);

    tms_msg_db::TmsdbStamped db_msg;
    tms_msg_db::Tmsdb tmpData;
    ros::Time now = ros::Time::now() + ros::Duration(9 * 60 * 60); // GMT +9

    tmpData.time = boost::posix_time::to_iso_extended_string(now.toBoost());
    tmpData.id = 2009;
    tmpData.name = "refrigerator";
    tmpData.place = 5002;
    tmpData.sensor = 3005;
    tmpData.state = 1;
    tmpData.joint = "0";

    db_msg.tmsdb.push_back(tmpData);
    db_pub.publish(db_msg);

    nh1.setParam("/2009_is_real", true);
  }

  //	apprise TS_control of succeeding subtask execution
  s_srv.request.state = 1;
  state_client.call(s_srv);
  return true;
}

bool tms_rp::TmsRpSubtask::release(SubtaskData sd)
{
  tms_msg_db::TmsdbGetData db_srv;
  tms_msg_ts::ts_state_control s_srv;
  s_srv.request.type = 1; // for subtask state update;
  s_srv.request.state = 0;

  int grasping_id = 0;

  // SET ROBOT
  switch (sd.robot_id)
  {
  case 2003:
    ROS_INFO("ID:%d is selected", sd.robot_id);
    break;
  default:
    ROS_ERROR("An illegal robot id");
    return false;
  }

  db_srv.request.tmsdb.id = sd.robot_id;

  db_srv.request.tmsdb.sensor = 3003;
  if (sd.type == false)
    db_srv.request.tmsdb.sensor = 3005;

  if (!get_data_client_.call(db_srv))
  {
    s_srv.request.error_msg = "failed get object id";
    state_client.call(s_srv);
    return false;
  }

  std::string note = db_srv.response.tmsdb[0].note;
  ROS_INFO("note = %s", note.c_str());
  std::vector<std::string> v_note;
  v_note.clear();
  boost::split(v_note, note, boost::is_any_of(";"));

  for (int i = 0; i < v_note.size(); i++)
  {
    std::vector<std::string> v_v_note;
    v_v_note.clear();
    boost::split(v_v_note, v_note.at(i), boost::is_any_of("="));
    if (v_v_note.at(0) == "grasping")
    {
      std::stringstream ss;
      ss << v_v_note.at(1);
      ss >> grasping_id;
      ROS_INFO("grasping_id:%d", grasping_id);
      break;
    }
  }

  if (grasping_id == 0)
  {
    s_srv.request.error_msg = "cannot found grasping_id";
    state_client.call(s_srv);
    return false;
  }

  if (sd.arg_type > 1000 && sd.arg_type < 2000) // person
  {
    switch (sd.robot_id)
    {
    case 2002: // for smartpal simulation
    {
    }
    case 2003:
    {
      if (1)
      {
        double arg[8];
        arg[0] = 0.0;
        arg[1] = -0.1;
        arg[2] = 0.0;
        arg[3] = 1.5707;
        arg[4] = 0.0;
        arg[5] = 0.0;
        arg[6] = 0.0;
        arg[7] = 0.175;
        if (!sp5_control(sd.type, UNIT_ARM_L, CMD_MOVE_ABS, 8, arg))
        {
          send_rc_exception(2);
          return false;
        }

        sleep(1);

        tms_msg_rp::rp_release srv;
        srv.request.object_id = grasping_id;
        if (!subtask_release_client.call(srv))
        {
          s_srv.request.error_msg = "failed release object";
          state_client.call(s_srv);
          return false;
        }
        ROS_INFO("Successed release object");

        sleep(1);

        arg[0] = -0.2;
        arg[1] = 0.175;
        arg[2] = 0.175;
        if (!sp5_control(sd.type, UNIT_GRIPPER_L, CMD_MOVE_ABS, 3, arg))
        {
          send_rc_exception(4);
          return false;
        }

        arg[0] = 0.0;
        arg[1] = -0.17;
        arg[2] = 0.0;
        arg[3] = 0.0;
        arg[4] = 0.0;
        arg[5] = 0.0;
        arg[6] = 0.0;
        arg[7] = 0.175;
        if (!sp5_control(sd.type, UNIT_ARM_L, CMD_MOVE_ABS, 8, arg))
        {
          send_rc_exception(2);
          return false;
        }
      }
      else
      {
      }
      break;
    }
    default:
      s_srv.request.error_msg = "Unsupported robot in release function";
      state_client.call(s_srv);
      return false;
    }
  }
  else if (sd.arg_type > 6000 && sd.arg_type < 7000)
  { // furniture
    switch (sd.robot_id)
    {
    case 2002:
    {
    }
    case 2003:
    {
      tms_msg_db::Tmsdb assign_data;
      tms_msg_rp::rp_place srv;
      srv.request.object_id = grasping_id;
      srv.request.roll = 0.0;
      srv.request.pitch = 0.0;

      db_srv.request.tmsdb.id = grasping_id;
      double offset_z = 0;
      if (!get_data_client_.call(db_srv))
      {
        s_srv.request.error_msg = "failed get data";
        state_client.call(s_srv);
        return false;
      }
      srv.request.yaw = db_srv.response.tmsdb[0].ry;
      offset_z = db_srv.response.tmsdb[0].offset_z;
      ROS_INFO("yaw = %f offset_z =%f", srv.request.yaw, offset_z);

      std::stringstream assign_note;
      db_srv.request.tmsdb.id = sd.arg_type;
      if (!get_data_client_.call(db_srv))
      {
        s_srv.request.error_msg = "failed get data";
        state_client.call(s_srv);
        return false;
      }
      std::string note = db_srv.response.tmsdb[0].note;
      assign_data = db_srv.response.tmsdb[0];
      ROS_INFO("note(before) = %s", note.c_str());
      std::vector<std::string> v_note;
      v_note.clear();
      boost::split(v_note, note, boost::is_any_of(";"));

      for (int i = 0; i < v_note.size(); i++)
      {
        ROS_INFO("v_note[%d]=%s", i, v_note.at(i).c_str());
      }

      int i = 0;
      while ("0" != v_note.at(i))
      {
        i += 2;
        if (i >= v_note.size())
        {
          s_srv.request.error_msg = "cannot found empty space";
          state_client.call(s_srv);
          return false;
        }
      }

      std::string notedata = v_note.at(i + 1);
      std::vector<std::string> v_notedata;
      v_notedata.clear();
      boost::split(v_notedata, notedata, boost::is_any_of(","));

      if (v_notedata.size() == 3)
      {
        std::stringstream ss;
        double d_x, d_y, d_z;

        ss << v_notedata.at(0);
        ss >> d_x;
        srv.request.x = d_x;
        ROS_INFO("place_x = [%f] ", d_x);

        ss.clear();
        ss.str("");
        ss << v_notedata.at(1);
        ss >> d_y;
        srv.request.y = d_y;
        ROS_INFO("place_y = [%f] ", d_y);

        ss.clear();
        ss.str("");
        ss << v_notedata.at(2);
        ss >> d_z;
        srv.request.z = d_z + offset_z;
        ROS_INFO("place_z = [%f] ", d_z + offset_z);
      }

      for (int j = 0; j < v_note.size(); j++)
      {
        if (j == i)
        {
          assign_note << grasping_id;
        }
        else
        {
          assign_note << v_note.at(j);
        }

        if (j != v_note.size() - 1)
        {
          assign_note << ";";
        }
      }

      if (!subtask_place_client.call(srv))
      {
        s_srv.request.error_msg = "failed place action";
        state_client.call(s_srv);
        return false;
      }
      ROS_INFO("Successed place action");

      ros::Time now = ros::Time::now() + ros::Duration(9 * 60 * 60);

      assign_data.time = boost::posix_time::to_iso_extended_string(now.toBoost());
      assign_data.note = assign_note.str();

      ROS_INFO("note(after) = %s", assign_note.str().c_str());

      tms_msg_db::TmsdbStamped db_msg;
      db_msg.header.frame_id = "/world";
      db_msg.header.stamp = now;

      db_msg.tmsdb.push_back(assign_data);
      db_pub.publish(db_msg);

      double arg[8];

      if (sd.type)
      {
        sleep(1);
        arg[0] = 0.0;
        if (!sp5_control(sd.type, UNIT_ALL, CMD_MOVE_TRAJECTORY, 1, arg))
        {
          send_rc_exception(7);
          return false;
        }
      }

      arg[0] = 0.0;
      arg[1] = -0.17;
      arg[2] = 0.0;
      arg[3] = 0.0;
      arg[4] = 0.0;
      arg[5] = 0.0;
      arg[6] = 0.0;
      arg[7] = 0.175;
      if (!sp5_control(sd.type, UNIT_ARM_L, CMD_MOVE_ABS, 8, arg))
      {
        send_rc_exception(2);
        return false;
      }

      if (sd.type)
      {
        sleep(1);
        double arg[1] = {0.0};
        if (!sp5_control(sd.type, UNIT_ALL, CMD_MOVE_TRAJECTORY, 1, arg))
        {
          send_rc_exception(7);
          return false;
        }
      }
    }
    }
  }
  else
  {
    ROS_ERROR("An illigal user id");
  }

  //	apprise TS_control of succeeding subtask execution
  s_srv.request.state = 1;
  state_client.call(s_srv);
  return true;
}

// bool tms_rp::TmsRpSubtask::open_ref(void) {
//	tms_msg_rs::rs_home_appliances ref_srv;
//	tms_msg_ts::ts_state_control s_srv;
//	s_srv.request.type = 1; // for subtask state update;
//	s_srv.request.state = 0;

//	ref_srv.request.id = 2009;
//	ref_srv.request.service = 1;
//	if (refrigerator_client.call(ref_srv)) {}
//	else {
//		s_srv.request.error_msg = "Failed to call service control_refrigerator";
//		state_client.call(s_srv);
//	    return false;
//	}
//	//	apprise TS_control of succeeding subtask execution
//	s_srv.request.state = 1;
//	state_client.call(s_srv);
//	return true;
//}

// bool tms_rp::TmsRpSubtask::close_ref(void) {
//	tms_msg_rs::rs_home_appliances ref_srv;
//	tms_msg_ts::ts_state_control s_srv;
//	s_srv.request.type = 1; // for subtask state update;
//	s_srv.request.state = 0;

//	ref_srv.request.id = 2009;
//	ref_srv.request.service = 0;
//	if (refrigerator_client.call(ref_srv)) {}
//	else {
//		s_srv.request.error_msg = "Failed to call service control_refrigerator";
//		state_client.call(s_srv);
//	    return false;
//	}
//	//	apprise TS_control of succeeding subtask execution
//	s_srv.request.state = 1;
//	state_client.call(s_srv);
//	return true;
//}

////------------------------------------------------------------------------------
// bool tms_rp::TmsRpSubtask::random_move(void) {
//	tms_msg_ts::ts_state_control s_srv;
//	s_srv.request.type = 1; // for subtask state update;
//	s_srv.request.state = 0;

//	int ret;
//	const char buf[] = "roslaunch kobuki_random_walker safe_random_walker_app.launch\n";
//	ROS_INFO("%s\n", buf);

//	ret = std::system(buf);
//	if(ret != 0){
//		s_srv.request.error_msg = "Excute command error";
//		state_client.call(s_srv);
//		return false;
//	  }
//	//	apprise TS_control of succeeding subtask execution
//	s_srv.request.state = 1;
//	state_client.call(s_srv);
//	return true;
//}

////------------------------------------------------------------------------------
// bool tms_rp::TmsRpSubtask::sensing(void) {
//	tms_msg_ts::ts_state_control s_srv;
//	s_srv.request.type = 1; // for subtask state update;
//	s_srv.request.state = 0;

//	int ret;
//	const char buf[] = "rosrun tms_ss_ods_person_detection ods_realtime_persondt\n";
//	ROS_INFO("%s\n", buf);

//	ret = std::system(buf);
//	if(ret != 0){
//		s_srv.request.error_msg = "Excute command error";
//		state_client.call(s_srv);
//		return false;
//	}
//	// ======= add end determination? =======
//	return true;
//	//	apprise TS_control of succeeding subtask execution
//	s_srv.request.state = 1;
//	state_client.call(s_srv);
//	return true;
//}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "tms_ts_subtask");
  tms_rp::TmsRpSubtask subtask;
  ros::spin();
  return 0;
}
