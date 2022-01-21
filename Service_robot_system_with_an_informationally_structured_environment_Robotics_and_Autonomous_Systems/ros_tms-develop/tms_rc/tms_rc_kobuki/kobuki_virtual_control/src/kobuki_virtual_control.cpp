#include <ros/ros.h>
#include <unistd.h>

#include <tms_msg_rc/rc_robot_control.h>
#include <tms_msg_db/TmsdbStamped.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#define SUCCESS 1
#define FAILURE -1
#define SRV_UNIT_ERR -112
#define SRV_CMD_ERR -113

ros::Publisher pose_publisher;
// ロボット初期値----------------------------------------------------------------
double g_x = 2000.0;
double g_y = 500.0;
double g_t = -90.0;  // -90
double g_r_state = 1;

//------------------------------------------------------------------------------
int8_t VehicleMoveLinearAbs(double x_mm, double y_mm, double theta_deg)
{
  bool ret = true;

  g_x = x_mm;
  g_y = y_mm;
  g_t = theta_deg - 90;  // 座標変換
  if (g_t > 180)
    g_t = g_t - 360;
  else if (g_t < -180)
    g_t = g_t + 360;

  printf("vehicleMoveLinearAbs(%0.1fmm, %0.1fmm, %0.1fdeg) result:", g_x, g_y, g_t);
  ret ? printf("Success\n") : printf("Failure\n");

  if (ret)
    return SUCCESS;
  else
    return FAILURE;
}

//------------------------------------------------------------------------------
bool robotControl(tms_msg_rc::rc_robot_control::Request &req, tms_msg_rc::rc_robot_control::Response &res)
{
  switch (req.unit)
  {
    //----------------------------------------------------------------------------
    case 1:  // Vehivle
      switch (req.cmd)
      {
        case 15:
          res.result = VehicleMoveLinearAbs(req.arg[0], req.arg[1], req.arg[2]);
          break;
        default:
          res.result = SRV_CMD_ERR;
          break;
      }
      break;

    //--------------------------------------------------------------------------
    default:
      res.result = SRV_UNIT_ERR;
      break;
  }
  return true;
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //--------------------------------------------------------------------------
  ros::init(argc, argv, "kobuki_virtual_control");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("kobuki_virtual_control", robotControl);

  pose_publisher = nh.advertise< tms_msg_db::TmsdbStamped >("tms_db_data", 10);

  // kobuki initialize
  printf("Virtual KOBUKI initialization has been completed.\n\n");
  ros::Rate loop_rate(10);  // 10Hz frequency (0.1 sec)

  //--------------------------------------------------------------------------
  while (ros::ok())
  {
    ros::Time now = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9
    double secs = now.toSec();
    // ROS_INFO("r1|Time:%f,x:%f,y:%f,ry:%f", secs,g_x,g_y,g_t);

    tms_msg_db::TmsdbStamped db_msg;
    tms_msg_db::Tmsdb current_pos_data;

    db_msg.header.frame_id = "/world";
    db_msg.header.stamp = now;

    current_pos_data.time = boost::posix_time::to_iso_extended_string(now.toBoost());
    current_pos_data.id = 2005;
    current_pos_data.x = g_x;
    current_pos_data.y = g_y;
    current_pos_data.z = 0;
    current_pos_data.rr = 90.0;
    current_pos_data.rp = 0.0;
    current_pos_data.ry = g_t;
    current_pos_data.offset_z = 60;
    current_pos_data.place = 5001;
    current_pos_data.sensor = 3005;
    current_pos_data.state = g_r_state;

    db_msg.tmsdb.push_back(current_pos_data);
    pose_publisher.publish(db_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return (0);
}
//------------------------------------------------------------------------------
