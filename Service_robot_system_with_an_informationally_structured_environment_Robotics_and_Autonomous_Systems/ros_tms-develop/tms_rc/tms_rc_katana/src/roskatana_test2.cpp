// include for ROS
#include "ros/ros.h"
#include <unistd.h>
#include <boost/date_time/local_time/local_time.hpp>

#include <tms_msg_rc/katana_pos.h>
#include <tms_msg_rc/katana_pos_array.h>
#include <tms_msg_rc/katana_pos_single.h>
#include <tms_msg_rc/gripper_action.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ros_katana_test2");
  ros::NodeHandle n;
  ROS_INFO("ros_katana_test2 : init");

  ros::ServiceClient client0 = n.serviceClient< tms_msg_rc::gripper_action >("katana_gripper_action");
  ros::ServiceClient client1 = n.serviceClient< tms_msg_rc::katana_pos_array >("katana_move_angle_array");
  ros::ServiceClient client2 = n.serviceClient< tms_msg_rc::katana_pos_array >("katana_move_pose_array");
  ros::ServiceClient client3 = n.serviceClient< tms_msg_rc::katana_pos_array >("katana_move_enc");
  ros::ServiceClient client4 = n.serviceClient< tms_msg_rc::katana_pos_array >("katana_move_motor_angle");
  ros::ServiceClient client5 = n.serviceClient< tms_msg_rc::katana_pos_single >("katana_move_motor_angle");

  tms_msg_rc::gripper_action srv0;
  tms_msg_rc::katana_pos_array srv1;
  tms_msg_rc::katana_pos_single srv2;

  tms_msg_rc::katana_pos srv_temp;

  //	srv_temp.pose.push_back(180.0); srv_temp.pose.push_back(45.0);
  //	srv_temp.pose.push_back(90.0); srv_temp.pose.push_back(200.0);
  //	srv_temp.pose.push_back(180.0); srv_temp.pose.push_back(-30.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();
  //
  //	srv_temp.pose.push_back(150.0); srv_temp.pose.push_back(35.0);
  //	srv_temp.pose.push_back(60.0); srv_temp.pose.push_back(90.0);
  //	srv_temp.pose.push_back(90.0); srv_temp.pose.push_back(-30.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();
  //
  //	srv_temp.pose.push_back(90.0); srv_temp.pose.push_back(80.0);
  //	srv_temp.pose.push_back(160.0); srv_temp.pose.push_back(225.0);
  //	srv_temp.pose.push_back(45.0); srv_temp.pose.push_back(-30.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();

  //	srv0.request.gripper = 2;
  //	if(client0.call(srv0)){
  //
  //	}
  //	else{
  //		ROS_ERROR("error");
  //	}

  // for place(2), get breath
  srv2.request.pose.push_back(5);
  srv2.request.pose.push_back(26.0);
  if (client5.call(srv2))
  {
  }
  else
  {
    ROS_ERROR("error");
  }

  srv_temp.pose.push_back(145.0);
  srv_temp.pose.push_back(90.0);
  srv_temp.pose.push_back(180.0);
  srv_temp.pose.push_back(180.0);
  srv_temp.pose.push_back(180);
  srv_temp.pose.push_back(-90.0);

  srv1.request.pose_array.push_back(srv_temp);
  srv_temp.pose.clear();

  srv_temp.pose.push_back(145.0);
  srv_temp.pose.push_back(45.0);
  srv_temp.pose.push_back(60.0);
  srv_temp.pose.push_back(150.0);
  srv_temp.pose.push_back(180);
  srv_temp.pose.push_back(-45.0);

  srv1.request.pose_array.push_back(srv_temp);
  srv_temp.pose.clear();

  srv_temp.pose.push_back(145.0);
  srv_temp.pose.push_back(25.0);
  srv_temp.pose.push_back(80.0);
  srv_temp.pose.push_back(150.0);
  srv_temp.pose.push_back(180);
  srv_temp.pose.push_back(-45.0);

  srv1.request.pose_array.push_back(srv_temp);
  srv_temp.pose.clear();

  // for place (1)
  //	srv_temp.pose.push_back(160.0); srv_temp.pose.push_back(90.0);
  //	srv_temp.pose.push_back(180.0); srv_temp.pose.push_back(180.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-90.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();
  //
  //	srv_temp.pose.push_back(160.0); srv_temp.pose.push_back(45.0);
  //	srv_temp.pose.push_back(60.0); srv_temp.pose.push_back(150.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-45.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();
  //
  //	srv_temp.pose.push_back(160.0); srv_temp.pose.push_back(10.0);
  //	srv_temp.pose.push_back(120.0); srv_temp.pose.push_back(150.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-45.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();
  //
  //	srv_temp.pose.push_back(160.0); srv_temp.pose.push_back(45.0);
  //	srv_temp.pose.push_back(60.0); srv_temp.pose.push_back(150.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-45.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();
  //
  //	srv_temp.pose.push_back(160.0); srv_temp.pose.push_back(10.0);
  //	srv_temp.pose.push_back(120.0); srv_temp.pose.push_back(150.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-45.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();
  //
  //	srv_temp.pose.push_back(160.0); srv_temp.pose.push_back(45.0);
  //	srv_temp.pose.push_back(60.0); srv_temp.pose.push_back(150.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-45.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();

  // for place (2)
  //	srv_temp.pose.push_back(150.0); srv_temp.pose.push_back(90.0);
  //	srv_temp.pose.push_back(180.0); srv_temp.pose.push_back(180.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-90.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();
  //
  //	srv_temp.pose.push_back(150.0); srv_temp.pose.push_back(45.0);
  //	srv_temp.pose.push_back(60.0); srv_temp.pose.push_back(150.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-45.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();
  //
  //	srv_temp.pose.push_back(150.0); srv_temp.pose.push_back(25.0);
  //	srv_temp.pose.push_back(90.0); srv_temp.pose.push_back(150.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-45.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();
  //
  //	srv_temp.pose.push_back(150.0); srv_temp.pose.push_back(45.0);
  //	srv_temp.pose.push_back(60.0); srv_temp.pose.push_back(150.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-45.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();
  //
  //	srv_temp.pose.push_back(150.0); srv_temp.pose.push_back(25.0);
  //	srv_temp.pose.push_back(90.0); srv_temp.pose.push_back(150.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-45.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();
  //
  //	srv_temp.pose.push_back(150.0); srv_temp.pose.push_back(45.0);
  //	srv_temp.pose.push_back(60.0); srv_temp.pose.push_back(150.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-45.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();

  // for place (3)
  //	srv_temp.pose.push_back(200.0); srv_temp.pose.push_back(90.0);
  //	srv_temp.pose.push_back(180.0); srv_temp.pose.push_back(180.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-90.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();
  //
  //	srv_temp.pose.push_back(200.0); srv_temp.pose.push_back(45.0);
  //	srv_temp.pose.push_back(60.0); srv_temp.pose.push_back(150.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-45.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();
  //
  //	srv_temp.pose.push_back(200.0); srv_temp.pose.push_back(25.0);
  //	srv_temp.pose.push_back(90.0); srv_temp.pose.push_back(150.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-45.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();
  //
  //	srv_temp.pose.push_back(200.0); srv_temp.pose.push_back(45.0);
  //	srv_temp.pose.push_back(60.0); srv_temp.pose.push_back(150.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-45.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();
  //
  //	srv_temp.pose.push_back(200.0); srv_temp.pose.push_back(25.0);
  //	srv_temp.pose.push_back(90.0); srv_temp.pose.push_back(150.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-45.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();
  //
  //	srv_temp.pose.push_back(200.0); srv_temp.pose.push_back(45.0);
  //	srv_temp.pose.push_back(60.0); srv_temp.pose.push_back(150.0);
  //	srv_temp.pose.push_back(180); srv_temp.pose.push_back(-45.0);
  //
  //	srv1.request.pose_array.push_back(srv_temp);
  //	srv_temp.pose.clear();

  if (client1.call(srv1))
  {
  }
  else
  {
    ROS_ERROR("error");
  }

  return 0;
}
