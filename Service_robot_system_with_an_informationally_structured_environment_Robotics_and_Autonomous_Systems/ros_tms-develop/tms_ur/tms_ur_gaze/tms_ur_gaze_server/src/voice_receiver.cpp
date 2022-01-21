/*
 * voice_receiver.cpp
 *
 *  Created on: Jan 29, 2015
 *      Author: kazuto
 */

#include "voice_receiver.h"
#define JAPANESE 0

//----------------------------------------------------------------------------------
// constructor
//----------------------------------------------------------------------------------
VoiceReceiver::VoiceReceiver(ros::NodeHandle *nh) : nh_(nh)
{
  std::vector< std::string >::iterator itr;
  value_.push_back("chipstar");
  value_.push_back("tea");
  value_.push_back("soysauce");
  value_.push_back("bottle");
  value_.push_back("robot");
  // chipstar-------------------------------------------------------------------
  itr = value_.begin();
  hash_.insert(std::make_pair("chips", &(*itr)));
#if JAPANESE
  hash_.insert(std::make_pair("お菓子", &(*itr)));
  hash_.insert(std::make_pair("チップスター", &(*itr)));
  hash_.insert(std::make_pair("ポテトチップス", &(*itr)));
  hash_.insert(std::make_pair("チップス", &(*itr)));
#endif
  // tea------------------------------------------------------------------------
  itr++;
  hash_.insert(std::make_pair("tea", &(*itr)));
#if JAPANESE
  hash_.insert(std::make_pair("ペットボトル", &(*itr)));
  hash_.insert(std::make_pair("お茶", &(*itr)));
#endif
  // soysauce-------------------------------------------------------------------
  itr++;
  hash_.insert(std::make_pair("soysauce", &(*itr)));
  hash_.insert(std::make_pair("sauce", &(*itr)));
#if JAPANESE
  hash_.insert(std::make_pair("醤油", &(*itr)));
  hash_.insert(std::make_pair("しょうゆ", &(*itr)));
#endif
  // bottle---------------------------------------------------------------------
  itr++;
  hash_.insert(std::make_pair("water", &(*itr)));
  hash_.insert(std::make_pair("bottle", &(*itr)));
#if JAPANESE
  hash_.insert(std::make_pair("ボトル", &(*itr)));
#endif
  // robot---------------------------------------------------------------------
  itr++;
  hash_.insert(std::make_pair("robot", &(*itr)));
  hash_.insert(std::make_pair("robots", &(*itr)));
#if JAPANESE
  hash_.insert(std::make_pair("ロボット", &(*itr)));
#endif

  server_ = nh_->advertiseService("voice_command", &VoiceReceiver::Callback, this);
  client_ = nh_->serviceClient< tms_ur_gaze_server::object_list >("object_sorting");
  arrow_client_ = nh_->serviceClient< tms_msg_rp::rp_arrow >("rp_arrow");

  ROS_INFO("voice receiver constructed");
}

//----------------------------------------------------------------------------------
// destructor
//----------------------------------------------------------------------------------
VoiceReceiver::~VoiceReceiver()
{
  ROS_INFO("voice receiver destructed");
}

//----------------------------------------------------------------------------------
// list candidate
//----------------------------------------------------------------------------------
void VoiceReceiver::ListCandidates(std::string tag, tms_ur_gaze_server::object_list *srv)
{
  if (tag == "chipstar")
  {
    srv->request.id_in.push_back(7001);  // chipstar_red
    srv->request.id_in.push_back(7002);  // chipstar_orange
    srv->request.id_in.push_back(7003);  // chipstar_green
  }
  else if (tag == "tea")
  {
    srv->request.id_in.push_back(7004);  // greentea_bottle
    srv->request.id_in.push_back(7005);  // soukentea_bottle
  }
  else if (tag == "soysauce")
  {
    srv->request.id_in.push_back(7009);  // soysauce_bottle_black
    srv->request.id_in.push_back(7010);  // soysauce_bottle_blue
    srv->request.id_in.push_back(7011);  // soysauce_bottle_white
  }
  else if (tag == "bottle")
  {
    srv->request.id_in.push_back(7004);  // greentea_bottle
    srv->request.id_in.push_back(7005);  // soukentea_bottle
    srv->request.id_in.push_back(7009);  // soysauce_bottle_black
    srv->request.id_in.push_back(7010);  // soysauce_bottle_blue
    srv->request.id_in.push_back(7011);  // soysauce_bottle_white
  }
  else if (tag == "robot")
  {
    srv->request.id_in.push_back(2001);  // smartpal5
    srv->request.id_in.push_back(2002);  // smartpal4
  }
}

//----------------------------------------------------------------------------------
// callback function
//----------------------------------------------------------------------------------
bool VoiceReceiver::Callback(tms_ur_gaze_server::recognized_text::Request &req,
                             tms_ur_gaze_server::recognized_text::Response &res)
{
  tms_ur_gaze_server::object_list srv;
  tms_msg_rp::rp_arrow arrow_srv;

  std::cout << "Detected sentence: " << req.request << std::endl;

  // Seek the key in the hash table
  if (hash_.find(req.request) != hash_.end())
  {
    ROS_INFO_STREAM("Matched: " << *(hash_.at(req.request)));
    ListCandidates(*(hash_.at(req.request)), &(srv));
  }
  else
  {
    ROS_ERROR("Nothing has matched\n");
    res.response = "Nothing has matched";
    return true;
  }

  // object_selector
  if (client_.call(srv))
  {
    ROS_INFO_STREAM("Result: " << srv.response.id_out << " " << srv.response.name.c_str());

#if 0
    // Point to the object on the choreonoid
    arrow_srv.request.x = srv.response.x * 1000;
    arrow_srv.request.y = srv.response.y * 1000;
    arrow_srv.request.z = srv.response.z * 1000;
    arrow_srv.request.id = 20001;
    arrow_srv.request.mode = 1;

    // rp_arrow
    if (arrow_client_.call(arrow_srv)) {
      arrow_srv.request.mode = 0;
      if (arrow_client_.call(arrow_srv)) {
        res.response = "Succeeded to call service rp_arrow";
        return true;
      }
    } else {
      ROS_ERROR("Failed to call service rp_arrow\n");
      res.response = "Failed to call service rp_arrow";
      return true;
    }
#endif
  }
  else
  {
    ROS_ERROR("Failed to call service object_selector\n");
    res.response = "Failed to call service object_selector";
  }

  return true;
}
