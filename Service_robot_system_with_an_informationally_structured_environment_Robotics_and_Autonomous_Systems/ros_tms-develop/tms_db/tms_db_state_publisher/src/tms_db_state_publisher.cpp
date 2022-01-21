
//------------------------------------------------------------------------------
// @file   : tms_db_state_publisher.cpp
// @brief  : subscribe the current information of object and publish the state of object
// @author : Yoonseok Pyo
// @version: Ver0.0.4 (since 2015.07.22)
// @date   : 2015.08.26
//------------------------------------------------------------------------------
// include for ROS
#include <ros/ros.h>
#include <unistd.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_db/Tmsdb.h>
#include <tms_msg_db/TmsdbGetData.h>
#include <sensor_msgs/JointState.h>
#include <tms_msg_ss/SkeletonArray.h>
#include <visualization_msgs/Marker.h>

// include for std
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <map>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "picojson.h"

// #define rad2deg(x)	((x)*(180.0)/M_PI)
// #define deg2rad(x)	((x)*M_PI/180.0)

//------------------------------------------------------------------------------
using std::string;
using std::vector;
using namespace std;
using namespace boost;

const int inf = -100000;
#define PI 3.14159265


//------------------------------------------------------------------------------
class DbStatePublisher
{
  //------------------------------------------------------------------------------
private:
  // ROS NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // ROS Parameters
  bool is_debug;
  // ROS Topic Subscriber
  ros::Subscriber data_sub;
  // ROS Topic Publisher
  ros::Publisher state_pub;
  ros::Publisher skeleton_pub;
  ros::Publisher skeleton_joint_pub;
  ros::Publisher marker_pub;
  ros::Publisher heartrate_pub;

  ros::ServiceClient get_data_client_;
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::TransformListener listener;
  int nfbed_state=0;

  //------------------------------------------------------------------------------
public:
  DbStatePublisher() : nh_priv("~"), is_debug(false)
  {
    // Init parameter
    nh_priv.param("is_debug", is_debug, is_debug);
    // Init target name
    ROS_ASSERT(initDbStatePublisher());
    data_sub = nh.subscribe("/tms_db_publisher", 1, &DbStatePublisher::dbTFCallback, this);
    state_pub = nh.advertise< sensor_msgs::JointState >("/joint_states", 10);
    get_data_client_ = nh.serviceClient< tms_msg_db::TmsdbGetData >("/tms_db_reader");
    skeleton_pub = nh.advertise< tms_msg_ss::SkeletonArray >("integrated_skeleton_stream", 1);
    skeleton_joint_pub = nh.advertise< sensor_msgs::JointState >("/sim/joint_states",10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    heartrate_pub = nh.advertise<visualization_msgs::Marker>("heartrate",10);
  }

  //----------------------------------------------------------------------------
  ~DbStatePublisher()
  {
    ROS_ASSERT(shutdownDbStatePublisher());
  }

  //------------------------------------------------------------------------------
private:
  bool initDbStatePublisher()
  {
    ROS_INFO("tms_db_state_publisher : Init OK!\n");
    return true;
  }

  //----------------------------------------------------------------------------
  bool shutdownDbStatePublisher()
  {
    return true;
  }

  //----------------------------------------------------------------------------
  // void dbTFCallback(const ros::MessageEvent<tms_msg_db::TmsdbStamped const>& event)
  void dbTFCallback(const tms_msg_db::TmsdbStamped::ConstPtr &msg)
  {
    uint32_t id, oID, sensor, state, place;
    double posX, posY, posZ, rotR, rotP, rotY;
    sensor_msgs::JointState state_data;
    std::string joint,note;

    if (msg->tmsdb.size() == 0)
      return;

    bool type2003;
    nh.param< bool >("/2003_is_real", type2003, true);
    bool type2009;
    nh.param< bool >("/2009_is_real", type2009, true);

    for (uint32_t i = 0; i < msg->tmsdb.size(); i++)
    {
      id = msg->tmsdb[i].id;
      sensor = msg->tmsdb[i].sensor;
      state = msg->tmsdb[i].state;
      place = msg->tmsdb[i].place;
      joint = msg->tmsdb[i].joint;
      note = msg->tmsdb[i].note;

      if (id == 1100)  // pozyx
      {
        if (state == 1)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          posZ = msg->tmsdb[i].z;
          rotR = msg->tmsdb[i].rr;
          rotP = msg->tmsdb[i].rp;
          rotY = msg->tmsdb[i].ry;

          if (posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            transform.setOrigin(tf::Vector3(posX, posY, posZ));
            transform.setRotation(tf::Quaternion(rotP*cos(rotY)+rotR*sin(rotY), rotR*cos(rotY)-rotP*sin(rotY), rotY-1.5708));
            // transform.setRotation(tf::Quaternion(whsPitch,whsRoll,-1.5708));

            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world_link", "Body"));
            sensor_msgs::JointState skeleton_joint;
            skeleton_joint.header.stamp = ros::Time::now();
            skeleton_joint.name.push_back("R_ARM_JOINT1");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("R_ARM_JOINT2");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("R_ARM_JOINT3");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("R_ARM_JOINT4");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("R_ARM_JOINT5");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("R_ARM_JOINT6");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("R_ARM_JOINT7");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("L_ARM_JOINT1");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("L_ARM_JOINT2");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("L_ARM_JOINT3");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("L_ARM_JOINT4");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("L_ARM_JOINT5");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("L_ARM_JOINT6");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("L_ARM_JOINT7");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("NECK_JOINT0");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("NECK_JOINT1");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("NECK_JOINT2");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("R_LEG_JOINT1");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("R_LEG_JOINT2");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("R_LEG_JOINT3");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("R_LEG_JOINT4");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("R_LEG_JOINT5");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("R_LEG_JOINT6");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("L_LEG_JOINT1");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("L_LEG_JOINT2");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("L_LEG_JOINT3");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("L_LEG_JOINT4");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("L_LEG_JOINT5");
            skeleton_joint.position.push_back(0);
            skeleton_joint.name.push_back("L_LEG_JOINT6");
            skeleton_joint.position.push_back(0);
            skeleton_joint_pub.publish(skeleton_joint);
          }
        }
      }

      if (id == 2003 && ((type2003 == true && sensor == 3001) || (type2003 == false && sensor == 3005)))  // smartpal5-2
      {
        if (state == 1)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotY = msg->tmsdb[i].ry;
          double g_jR[7] = {0.0, -0.08, 0.0, 0.0, 0.0, 0.0, 0.0};
          double g_jL[7] = {0.0, -0.08, 0.0, 0.0, 0.0, 0.0, 0.0};
          double g_gripper_right = 0.3;
          double g_gripper_left = 0.3;

          if (joint != "")
          {
            std::vector< std::string > v_joint;
            v_joint.clear();
            boost::split(v_joint, joint, boost::is_any_of(";"));
            std::stringstream ss;
            for (int i = 0; i < 7; i++)
            {
              ss.clear();
              ss << v_joint.at(2 + i);
              ss >> g_jR[i];
            }
            ss.clear();
            ss << v_joint.at(9);
            ss >> g_gripper_right;
            for (int i = 0; i < 7; i++)
            {
              ss.clear();
              ss << v_joint.at(10 + i);
              ss >> g_jL[i];
            }
            ss.clear();
            ss << v_joint.at(17);
            ss >> g_gripper_left;
          }

          state_data.header.stamp = ros::Time::now();
          state_data.name.push_back("smartpal5_x_joint");
          state_data.position.push_back(posX);
          state_data.name.push_back("smartpal5_y_joint");
          state_data.position.push_back(posY);
          state_data.name.push_back("smartpal5_yaw_joint");
          state_data.position.push_back(rotY);
          state_data.name.push_back("base_footprint_joint");
          state_data.position.push_back(0);
          state_data.name.push_back("lumbar_lower_joint");
          state_data.position.push_back(0);
          state_data.name.push_back("lumbar_upper_joint");
          state_data.position.push_back(0);
          // state_data.name.push_back("head_camera_joint");
          // state_data.position.push_back(0);
          state_data.name.push_back("l_arm_j1_joint");
          state_data.position.push_back(g_jL[0]);
          state_data.name.push_back("l_arm_j2_joint");
          state_data.position.push_back(g_jL[1]);  //(0.08);
          state_data.name.push_back("l_arm_j3_joint");
          state_data.position.push_back(g_jL[2]);
          state_data.name.push_back("l_arm_j4_joint");
          state_data.position.push_back(g_jL[3]);
          state_data.name.push_back("l_arm_j5_joint");
          state_data.position.push_back(g_jL[4]);
          state_data.name.push_back("l_arm_j6_joint");
          state_data.position.push_back(g_jL[5]);
          state_data.name.push_back("l_arm_j7_joint");
          state_data.position.push_back(g_jL[6]);
          state_data.name.push_back("l_gripper_thumb_joint");
          state_data.position.push_back(g_gripper_left);
          // state_data.name.push_back("l_gripper_joint");
          // state_data.position.push_back(0);
          // state_data.name.push_back("l_end_effector_joint");
          // state_data.position.push_back(0);
          state_data.name.push_back("r_arm_j1_joint");
          state_data.position.push_back(g_jR[0]);
          state_data.name.push_back("r_arm_j2_joint");
          state_data.position.push_back(g_jR[1]);
          state_data.name.push_back("r_arm_j3_joint");
          state_data.position.push_back(g_jR[2]);
          state_data.name.push_back("r_arm_j4_joint");
          state_data.position.push_back(g_jR[3]);
          state_data.name.push_back("r_arm_j5_joint");
          state_data.position.push_back(g_jR[4]);
          state_data.name.push_back("r_arm_j6_joint");
          state_data.position.push_back(g_jR[5]);
          state_data.name.push_back("r_arm_j7_joint");
          state_data.position.push_back(g_jR[6]);
          state_data.name.push_back("r_gripper_thumb_joint");
          state_data.position.push_back(g_gripper_right);
          // state_data.name.push_back("r_gripper_joint");
          // state_data.position.push_back(0);
          // state_data.name.push_back("r_end_effector_joint");
          // state_data.position.push_back(0);
        }
      }

      if (id == 2007)  // wheelchair
      {
        if (state != 0)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotY = msg->tmsdb[i].ry;

          if (posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("wheelchair_x_joint");
            state_data.name.push_back("wheelchair_y_joint");
            state_data.name.push_back("wheelchair_yaw_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(rotY);
          }
        }
      }

      if (id == 2009 &&
          ((type2009 == true && sensor == 3001) || (type2009 == false && sensor == 3005)))  // refrigerator
      {
        if (joint != "")
        {
          std::stringstream ss;
          ss << joint;
          ss >> rotY;
          ss.clear();

          state_data.header.stamp = ros::Time::now();
          state_data.name.push_back("refrigerator_door_joint");
          state_data.position.push_back(rotY);
        }
      }

      if (id == 3020) //nursingcare_bed
      {
        picojson::value v;
        std::string err;
        const char* json = note.c_str();
        picojson::parse(v, json, json + strlen(json), &err);
        if(err.empty())
        {
          picojson::object& o = v.get<picojson::object>();
          std::string slstate = o["sleepstate"].get<std::string>().c_str();

          if(strcmp(slstate.c_str(),"state_off")==0) nfbed_state=0;
          else if(strcmp(slstate.c_str(),"state_up")==0) nfbed_state=1;
          else if(strcmp(slstate.c_str(),"state_wake")==0) nfbed_state=2;
          else nfbed_state=3;
        }
      }

      if (id == 3021 && state == 1) //whs1
      {
        tms_msg_db::TmsdbGetData srv;
        srv.request.tmsdb.id = 1100;
        get_data_client_.call(srv);
        // if(srv.response.tmsdb[0].x > BED_X1 && srv.response.tmsdb[0].x < BED_X2 && srv.response.tmsdb[0].y > BED_Y1 && srv.response.tmsdb[0].y < BED_Y2)
        // {
        //   posX = 11.15; //in the bed
        //   posY = 2.57;
        //   posZ = 0.5;
        //   rotY = 0;
        // }else
        // {
        //   rotY = srv.response.tmsdb[0].ry - PI;
        //   posX = srv.response.tmsdb[0].x - 0.05*cos(rotY) + 0.15*sin(rotY);
        //   posY = srv.response.tmsdb[0].y - 0.15*cos(rotY) - 0.05*sin(rotY);
        //   posZ = 1.7;
        // }

        tf::StampedTransform transform;
        double tfx,tfy,tfz,yaw;
        try{
          listener.lookupTransform("/world_link","/Head",ros::Time(0),transform);
          tfx = transform.getOrigin().getX();
          tfy = transform.getOrigin().getY();
          tfz = transform.getOrigin().getZ();
          yaw = tf::getYaw(transform.getRotation());
        }catch(tf::TransformException &ex){
          ROS_ERROR("%s",ex.what());
          continue;
        }

        rotY = yaw-PI*0.5;
        posX = tfx + 0.15*sin(rotY);// - 0.05*cos(rotY) + 0.15*sin(rotY);
        posY = tfy - 0.15*cos(rotY);// - 0.15*cos(rotY) - 0.05*sin(rotY);
        posZ = tfz + 0.2;

        // rotY = srv.response.tmsdb[0].ry - PI;
        // posX = srv.response.tmsdb[0].x - 0.05*cos(rotY) + 0.15*sin(rotY);
        // posY = srv.response.tmsdb[0].y - 0.15*cos(rotY) - 0.05*sin(rotY);
        // double height = 0.3+1.4*cos(srv.response.tmsdb[0].rp)*cos(srv.response.tmsdb[0].rr);
        // if(srv.response.tmsdb[0].x > BED_X1 && srv.response.tmsdb[0].x < BED_X2 && srv.response.tmsdb[0].y > BED_Y1 && srv.response.tmsdb[0].y < BED_Y2)
        // {
        //   height+=0.3;
        // }
        // posZ = height;

        visualization_msgs::Marker points;
        visualization_msgs::Marker heartrate;
        points.header.frame_id = "/world_link";
        points.header.stamp = ros::Time::now();
        points.type = visualization_msgs::Marker::LINE_STRIP;
        points.action = visualization_msgs::Marker::ADD;
        points.scale.x = 0.008;
        points.color.r = 0;
        points.color.g = 1;
        points.color.b = 0;
        points.color.a = 1;
        heartrate.header.frame_id = "/world_link";
        heartrate.header.stamp = ros::Time::now();
        heartrate.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        heartrate.action = visualization_msgs::Marker::ADD;
        heartrate.scale.x = 0.005;
        heartrate.color.r = 1;
        heartrate.color.g = 0;
        heartrate.color.b = 0;
        heartrate.color.a = 1;
        heartrate.text = "test";
        heartrate.pose.position.x = posX - 0.4*sin(rotY);
        heartrate.pose.position.y = posY + 0.4*cos(rotY);
        heartrate.pose.position.z = posZ + 0.1;
        heartrate.scale.z = 0.15;
        picojson::value v;
        std::string err;
        const char* json = note.c_str();
        picojson::parse(v,json,json+strlen(json),&err);
        if(err.empty())
        {
          picojson::object& o = v.get<picojson::object>();
          int temp = o["temp"].get<double>();
          int rate = o["rate"].get<double>();
          std::stringstream ss;
          ss << rate;
          heartrate.text = ss.str();
          picojson::array& array = o["wave"].get<picojson::array>();
          std::vector<int> wavelist;
          for (picojson::array::iterator it2 = array.begin(); it2 != array.end(); it2++) {
            wavelist.push_back(it2->get<double>());
          }
          for(int i=0;i<wavelist.size()-1;i++)
          {
            geometry_msgs::Point p;
            p.x = posX - (wavelist.size()-i)*0.003*sin(rotY);
            p.y = posY + (wavelist.size()-i)*0.003*cos(rotY);
            p.z = posZ + wavelist[i]*0.0003;
            points.points.push_back(p);
          }
        }
        marker_pub.publish(points);
        heartrate_pub.publish(heartrate);
      }

      if (id == 6004)  // chair
      {
        if (state != 0)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotY = msg->tmsdb[i].ry;

          if (posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("chair_x_joint");
            state_data.name.push_back("chair_y_joint");
            state_data.name.push_back("chair_yaw_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(rotY);
          }
        }
      }

      if (id == 6007)  // meeting_chair1
      {
        if (state != 0)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotY = msg->tmsdb[i].ry;

          if (posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("meeting_chair1_x_joint");
            state_data.name.push_back("meeting_chair1_y_joint");
            state_data.name.push_back("meeting_chair1_yaw_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(rotY);
          }
        }
      }

      if (id == 6008)  // meeting_chair2
      {
        if (state != 0)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotY = msg->tmsdb[i].ry;

          if (posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("meeting_chair2_x_joint");
            state_data.name.push_back("meeting_chair2_y_joint");
            state_data.name.push_back("meeting_chair2_yaw_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(rotY);
          }
        }
      }

      if (id == 6009)  // meeting_chair3
      {
        if (state != 0)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotY = msg->tmsdb[i].ry;

          if (posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("meeting_chair3_x_joint");
            state_data.name.push_back("meeting_chair3_y_joint");
            state_data.name.push_back("meeting_chair3_yaw_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(rotY);
          }
        }
      }

      if (id == 6010)  // meeting_chair4
      {
        if (state != 0)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotY = msg->tmsdb[i].ry;

          if (posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("meeting_chair4_x_joint");
            state_data.name.push_back("meeting_chair4_y_joint");
            state_data.name.push_back("meeting_chair4_yaw_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(rotY);
          }
        }
      }

      if (id == 6018)  // wagon
      {
        if (state == 1)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotY = msg->tmsdb[i].ry;

          if (posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("wagon_x_joint");
            state_data.name.push_back("wagon_y_joint");
            state_data.name.push_back("wagon_yaw_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(rotY);
          }
        }
      }

      if (id == 7001)  // chipstar_red
      {
        if (state != 0)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          posZ = msg->tmsdb[i].z;
          rotR = msg->tmsdb[i].rr;
          rotP = msg->tmsdb[i].rp;
          rotY = msg->tmsdb[i].ry;

          if (posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("chipstar_red_x_joint");
            state_data.name.push_back("chipstar_red_y_joint");
            state_data.name.push_back("chipstar_red_z_joint");
            state_data.name.push_back("chipstar_red_yaw_joint");
            state_data.name.push_back("chipstar_red_pitch_joint");
            state_data.name.push_back("chipstar_red_roll_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(posZ);
            state_data.position.push_back(rotY);
            state_data.position.push_back(rotP);
            state_data.position.push_back(rotR);
          }
        }
        else if(state == 0)
        {
          posX = inf;
          posY = inf;
          posZ = inf;
          rotR = 0;
          rotP = 0;
          rotY = 0;
          state_data.header.stamp = ros::Time::now();
          state_data.name.push_back("chipstar_red_x_joint");
          state_data.name.push_back("chipstar_red_y_joint");
          state_data.name.push_back("chipstar_red_z_joint");
          state_data.name.push_back("chipstar_red_yaw_joint");
          state_data.name.push_back("chipstar_red_pitch_joint");
          state_data.name.push_back("chipstar_red_roll_joint");
          state_data.position.push_back(posX);
          state_data.position.push_back(posY);
          state_data.position.push_back(posZ);
          state_data.position.push_back(rotY);
          state_data.position.push_back(rotP);
          state_data.position.push_back(rotR);
        }
      }
      if (id == 7004)  // greentea_bottle
      {
        if (state != 0)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          posZ = msg->tmsdb[i].z;
          rotR = msg->tmsdb[i].rr;
          rotP = msg->tmsdb[i].rp;
          rotY = msg->tmsdb[i].ry;

          if (posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("greentea_bottle_x_joint");
            state_data.name.push_back("greentea_bottle_y_joint");
            state_data.name.push_back("greentea_bottle_z_joint");
            state_data.name.push_back("greentea_bottle_yaw_joint");
            state_data.name.push_back("greentea_bottle_pitch_joint");
            state_data.name.push_back("greentea_bottle_roll_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(posZ);
            state_data.position.push_back(rotY);
            state_data.position.push_back(rotP);
            state_data.position.push_back(rotR);
          }
        }
        else if(state == 0)
        {
          posX = inf;
          posY = inf;
          posZ = inf;
          rotR = 0;
          rotP = 0;
          rotY = 0;
          state_data.header.stamp = ros::Time::now();
          state_data.name.push_back("greentea_bottle_x_joint");
          state_data.name.push_back("greentea_bottle_y_joint");
          state_data.name.push_back("greentea_bottle_z_joint");
          state_data.name.push_back("greentea_bottle_yaw_joint");
          state_data.name.push_back("greentea_bottle_pitch_joint");
          state_data.name.push_back("greentea_bottle_roll_joint");
          state_data.position.push_back(posX);
          state_data.position.push_back(posY);
          state_data.position.push_back(posZ);
          state_data.position.push_back(rotY);
          state_data.position.push_back(rotP);
          state_data.position.push_back(rotR);
        }
      }
      if (id == 7006)  // cancoffee
      {
        if (state != 0)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          posZ = msg->tmsdb[i].z;
          rotR = msg->tmsdb[i].rr;
          rotP = msg->tmsdb[i].rp;
          rotY = msg->tmsdb[i].ry;

          if (posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("cancoffee_x_joint");
            state_data.name.push_back("cancoffee_y_joint");
            state_data.name.push_back("cancoffee_z_joint");
            state_data.name.push_back("cancoffee_yaw_joint");
            state_data.name.push_back("cancoffee_pitch_joint");
            state_data.name.push_back("cancoffee_roll_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(posZ);
            state_data.position.push_back(rotY);
            state_data.position.push_back(rotP);
            state_data.position.push_back(rotR);
          }
        }
        else if(state == 0)
        {
          posX = inf;
          posY = inf;
          posZ = inf;
          rotR = 0;
          rotP = 0;
          rotY = 0;
          state_data.header.stamp = ros::Time::now();
          state_data.name.push_back("cancoffee_x_joint");
          state_data.name.push_back("cancoffee_y_joint");
          state_data.name.push_back("cancoffee_z_joint");
          state_data.name.push_back("cancoffee_yaw_joint");
          state_data.name.push_back("cancoffee_pitch_joint");
          state_data.name.push_back("cancoffee_roll_joint");
          state_data.position.push_back(posX);
          state_data.position.push_back(posY);
          state_data.position.push_back(posZ);
          state_data.position.push_back(rotY);
          state_data.position.push_back(rotP);
          state_data.position.push_back(rotR);
        }
      }
      if (id == 7009)  // soysauce_bottle_black
      {
        if (state != 0)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          posZ = msg->tmsdb[i].z;
          rotR = msg->tmsdb[i].rr;
          rotP = msg->tmsdb[i].rp;
          rotY = msg->tmsdb[i].ry;

          if (posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("soysauce_bottle_black_x_joint");
            state_data.name.push_back("soysauce_bottle_black_y_joint");
            state_data.name.push_back("soysauce_bottle_black_z_joint");
            state_data.name.push_back("soysauce_bottle_black_yaw_joint");
            state_data.name.push_back("soysauce_bottle_black_pitch_joint");
            state_data.name.push_back("soysauce_bottle_black_roll_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(posZ);
            state_data.position.push_back(rotY);
            state_data.position.push_back(rotP);
            state_data.position.push_back(rotR);
          }
        }
        else if(state == 0)
        {
          posX = inf;
          posY = inf;
          posZ = inf;
          rotR = 0;
          rotP = 0;
          rotY = 0;
          state_data.header.stamp = ros::Time::now();
          state_data.name.push_back("soysauce_bottle_black_x_joint");
          state_data.name.push_back("soysauce_bottle_black_y_joint");
          state_data.name.push_back("soysauce_bottle_black_z_joint");
          state_data.name.push_back("soysauce_bottle_black_yaw_joint");
          state_data.name.push_back("soysauce_bottle_black_pitch_joint");
          state_data.name.push_back("soysauce_bottle_black_roll_joint");
          state_data.position.push_back(posX);
          state_data.position.push_back(posY);
          state_data.position.push_back(posZ);
          state_data.position.push_back(rotY);
          state_data.position.push_back(rotP);
          state_data.position.push_back(rotR);
        }
      }
    }
    state_pub.publish(state_data);
  }
};

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "tms_db_state_publisher");
  DbStatePublisher dsp;

  ros::spin();
  return 0;
}

//------------------------------------------------------------------------------
// EOF
