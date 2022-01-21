#ifndef TMS_TS_SUBTASK_H_
#define TMS_TS_SUBTASK_H_

#include <ros/ros.h>
#include <tms_msg_ts/ts_state_control.h>
#include <tms_msg_rp/rp_cmd.h>
#include <tms_msg_rp/rp_arrow.h>
#include <tms_msg_rp/rp_pick.h>
#include <tms_msg_rp/rp_place.h>
#include <tms_msg_rp/rp_release.h>
#include <tms_msg_rp/rps_voronoi_path_planning.h>
#include <tms_msg_rp/rps_goal_planning.h>
#include <tms_msg_rp/rps_joint_angle.h>
#include <tms_msg_rp/rps_cnoid_grasp_obj_planning.h>
#include <tms_msg_rc/tms_rc_pmove.h>
#include <tms_msg_rc/tms_rc_ppose.h>
//#include <tms_msg_rc/katana_pos_array.h>
//#include <tms_msg_rc/katana_pos.h>
#include <tms_msg_rc/rc_robot_control.h>
//#include <tms_msg_rs/rs_home_appliances.h>
//#include <tms_msg_ss/ods_person_dt.h>
#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_db/Tmsdb.h>
#include <tms_msg_db/TmsdbGetData.h>
//#include <kobuki_msgs/Sound.h>
//#include <kobuki_msgs/MotorPower.h>
//#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <math.h>

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
#define SET_ODOM 7
#define CMD_GETPOSE 8
#define CMD_SYNC_OBJ 8
#define CMD_MOVE_TRAJECTORY 8
#define CMD_CALC_BACKGROUND 9
#define CMD_MOVE_ABS 15
#define CMD_MOVE_REL 16

#define PI 3.14159265       // 180
#define HALF_PI 1.57079633  // 90

namespace tms_rp
{
class TmsRpSubtask
{
public:
  TmsRpSubtask();
  static TmsRpSubtask* instance();
  virtual ~TmsRpSubtask();

  double RadianNormalize(double rad);
  double DiffRadian(double rad1,double rad2);
  double distance(double x1, double y1, double x2, double y2);
  std::string DoubleToString(double number);
  void send_rc_exception(int error_type);
  bool get_robot_pos(bool type, int robot_id, std::string& robot_name, tms_msg_rp::rps_voronoi_path_planning& rp_srv);
  bool subtask(tms_msg_rp::rp_cmd::Request& req, tms_msg_rp::rp_cmd::Response& res);
  bool sp5_control(bool type, int unit, int cmd, int arg_size, double* arg);
  //	bool kxp_control(bool type, int unit, int cmd, int arg_size, double* arg);
  //	void sensingCallback(const tms_msg_ss::ods_person_dt::ConstPtr& msg);

private:
  ros::NodeHandle nh1;

  uint32_t sid_;
  struct SubtaskData
  {
    bool type;
    int robot_id;
    int arg_type;
    std::vector< double > v_arg;
  };

  bool update_obj(int id, double x, double y, double z, double rr, double rp, double ry, int place, int sensor,
                  int state, std::string note);
  //	bool kxp_set_odom(void);

  // for thread
  bool move(SubtaskData sd);     // 9001
  bool grasp(SubtaskData sd);    // 9002
  bool release(SubtaskData sd);  // 9003
                                 //	bool open_ref(void);  // 9004
                                 //	bool close_ref(void); // 9005
                                 //	bool random_move(void); // 9006
                                 //	bool sensing(void); // 9007

  ros::ServiceServer rp_subtask_server;
  ros::ServiceClient get_data_client_;
  ros::ServiceClient sp5_control_client_;
  ros::ServiceClient sp5_virtual_control_client;
  ros::ServiceClient subtask_pick_client;
  ros::ServiceClient subtask_place_client;
  ros::ServiceClient subtask_release_client;

  ros::ServiceClient rp_cmd_client;
  //	ros::ServiceClient kxp_virtual_control_client;
  //	ros::ServiceClient kxp_mbase_client;
  //	ros::ServiceClient v_kxp_mbase_client;
  //	ros::ServiceClient kxp_setpose_client;
  //	ros::ServiceClient katana_client;
  //	ros::ServiceClient v_katana_client;
  //	ros::ServiceClient kobuki_virtual_control_client;
  //	ros::ServiceClient kobuki_actual_control_client;
  //	ros::ServiceClient mkun_virtual_control_client;
  ros::ServiceClient mkun_control_client;
  ros::ServiceClient double_control_client;
  //egashira
  ros::ServiceClient turtlebot3_control_client;
  //
  ros::ServiceClient voronoi_path_planning_client_;
  ros::ServiceClient give_obj_client;
  //	ros::ServiceClient refrigerator_client;
  ros::ServiceClient state_client;

  ros::Publisher db_pub;
  ros::Publisher double_goal_pub;
  //	ros::Publisher kobuki_sound;
  //	ros::Publisher kobuki_motorpower;
  //	ros::Subscriber sensing_sub;

  //	std::ostream& os_;
  //	grasp::TmsRpController& trc_;
};
}

#endif  // TMS_TS_SUBTASK_H_
