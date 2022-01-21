/*
 * evaluator.cpp
 *
 *  Created on: Jan 29, 2015
 *      Author: kazuto
 */

#include "evaluator.h"
#define STATE 0

//----------------------------------------------------------------------------------
// absolute value
//----------------------------------------------------------------------------------
void Vec3::CalcAbs()
{
  length_ = sqrt(x_ * x_ + y_ * y_ + z_ * z_);
}

//----------------------------------------------------------------------------------
// argument
//----------------------------------------------------------------------------------
void Vec3::CalcArg()
{
  // glasses coodinates(fixed): (1,0,0)
  if (length_ == 0)
  {
    argument_ = 0.0;
  }
  else
  {
    argument_ = acos(x_ / length_);
  }
}

//----------------------------------------------------------------------------------
// constructor
//----------------------------------------------------------------------------------
Object::Object(ros::NodeHandle* nh) : nh_(nh), sort_key_(0)
{
  ROS_INFO("object constructed");
}

//----------------------------------------------------------------------------------
// get geometric info
//----------------------------------------------------------------------------------
bool Object::GetGeometry(int id)
{
  ros::ServiceClient db_client = nh_->serviceClient< tms_msg_db::TmsdbGetData >("/tms_db_reader");

  tms_msg_db::TmsdbGetData getData;
  getData.request.tmsdb.id = id;

  if (db_client.call(getData))
  {
    ROS_INFO("Get info of object ID: %d", getData.request.tmsdb.id);
  }
  else
  {
    ROS_ERROR("Failed to call service (ID: %d)", getData.request.tmsdb.id);
    return false;
  }

  if (getData.response.tmsdb.empty() == true)
  {
    ROS_ERROR("Nothing on floor ID: %d", getData.request.tmsdb.id);
    return false;
  }

#if STATE
  if (getData.response.tmsdb[0].state == 1)
  {
#endif
    id_ = id;
    name_ = getData.response.tmsdb[0].name;
    world_.x_ = getData.response.tmsdb[0].x / 1000;
    world_.y_ = getData.response.tmsdb[0].y / 1000;
    world_.z_ = getData.response.tmsdb[0].z / 1000;
    offset_.x_ = getData.response.tmsdb[0].offset_x / 1000;
    offset_.y_ = getData.response.tmsdb[0].offset_y / 1000;
    offset_.z_ = getData.response.tmsdb[0].offset_z / 1000;
#if STATE
  }
  else
  {
    ROS_ERROR("Unavailable ID: %d", getData.request.tmsdb.id);
    return false;
  }
#endif

  return true;
}

//----------------------------------------------------------------------------------
// constructor
//----------------------------------------------------------------------------------
Evaluator::Evaluator(ros::NodeHandle* nh)
  : nh_(nh), cos_yaw_(1), sin_yaw_(0), cos_pitch_(1), sin_pitch_(0), object_tmp_(nh)
{
  sever_ = nh_->advertiseService("object_sorting", &Evaluator::CallBack, this);
  ROS_INFO("evaluator constructed");
}

//----------------------------------------------------------------------------------
// destructor
//----------------------------------------------------------------------------------
Evaluator::~Evaluator()
{
  ROS_INFO("evaluator destructed");
}

//----------------------------------------------------------------------------------
// for indirect reference
//----------------------------------------------------------------------------------
void Evaluator::StoreAddress(std::vector< Object* >* object, std::vector< Object >* object_data)
{
  std::vector< Object >::iterator it, begin, end;
  begin = object_data->begin();
  end = object_data->end();
  for (it = begin; it != end; ++it)
  {
    object->push_back(&(*it));
  }
}

//----------------------------------------------------------------------------------
// callback function
//----------------------------------------------------------------------------------
bool Evaluator::CallBack(tms_ur_gaze_server::object_list::Request& req, tms_ur_gaze_server::object_list::Response& res)
{
  // get the geometry of glasses
  ros::ServiceClient db_client = nh_->serviceClient< tms_msg_db::TmsdbGetData >("/tms_db_reader");

  tms_msg_db::TmsdbGetData getData;
  getData.request.tmsdb.id = 1001;

  if (db_client.call(getData))
  {
    ROS_INFO_STREAM("Get info of object: " << getData.request.tmsdb.id);
  }
  else
  {
    ROS_ERROR_STREAM("Failed to call service: " << getData.request.tmsdb.id);
    return false;
  }

  if (getData.response.tmsdb.empty() == true)
  {
    ROS_ERROR_STREAM("Nothing on floor: " << getData.request.tmsdb.id);
    return false;
  }

#if STATE
  if (getData.response.tmsdb[0].state == 1)
  {
#endif
    trans_.x_ = getData.response.tmsdb[1].x / 1000;
    trans_.y_ = getData.response.tmsdb[1].y / 1000;
    trans_.z_ = getData.response.tmsdb[1].z / 1000;
    pitch_ = getData.response.tmsdb[1].rp * M_PI / 180;
    yaw_ = getData.response.tmsdb[1].ry * M_PI / 180;

    ROS_INFO("Glasses");
    ROS_INFO_STREAM("x: " << trans_.x_);
    ROS_INFO_STREAM("y: " << trans_.y_);
    ROS_INFO_STREAM("z: " << trans_.z_);
    ROS_INFO_STREAM("pitch: " << getData.response.tmsdb[1].rp);
    ROS_INFO_STREAM("yaw: " << getData.response.tmsdb[1].ry);

#if STATE
  }
  else
  {
    ROS_ERROR_STREAM("Unavailable: " << getData.request.tmsdb.id);
    return false;
  }
#endif

  cos_pitch_ = cos(pitch_);
  sin_pitch_ = sin(pitch_);
  cos_yaw_ = cos(yaw_);
  sin_yaw_ = sin(yaw_);

  int size = req.id_in.size();

  for (int j = 0; j < size; j++)
  {
    if (object_tmp_.GetGeometry(req.id_in[j]) != false)
    {
      // translation
      Vec3 diff;
      diff.x_ = object_tmp_.world_.x_ - trans_.x_;
      diff.y_ = object_tmp_.world_.y_ - trans_.y_;
      diff.z_ = object_tmp_.world_.z_ - trans_.z_;

      // rotation
      object_tmp_.glasses_.x_ =
          cos_yaw_ * cos_pitch_ * diff.x_ + cos_pitch_ * sin_yaw_ * diff.y_ + sin_pitch_ * diff.z_;
      object_tmp_.glasses_.y_ = -1 * sin_yaw_ * diff.x_ + cos_yaw_ * diff.y_;
      object_tmp_.glasses_.z_ =
          -1 * sin_pitch_ * cos_yaw_ * diff.x_ - sin_pitch_ * sin_yaw_ * diff.y_ + cos_pitch_ * diff.z_;

      // calculate the absolute value & the argument
      object_tmp_.glasses_.CalcAbs();
      object_tmp_.glasses_.CalcArg();

      // calculate the sort key
      object_tmp_.sort_key_ = object_tmp_.glasses_.length_ * (object_tmp_.glasses_.argument_ + 0.01);

      ROS_INFO_STREAM("global_x: " << object_tmp_.world_.x_ << " >> local_x: " << object_tmp_.glasses_.x_);
      ROS_INFO_STREAM("global_y: " << object_tmp_.world_.y_ << " >> local_y: " << object_tmp_.glasses_.y_);
      ROS_INFO_STREAM("global_z: " << object_tmp_.world_.z_ << " >> local_z: " << object_tmp_.glasses_.z_);
      ROS_INFO_STREAM("Abs: " << object_tmp_.glasses_.length_);
      ROS_INFO_STREAM("Arg: " << object_tmp_.glasses_.argument_ * 180 / M_PI);
      ROS_INFO_STREAM("Key: " << object_tmp_.sort_key_);

      objects_.push_back(object_tmp_);
    }
  }

  std::vector< Object* >::iterator it, begin, end;
  StoreAddress(&objects_ptr_, &objects_);

  begin = objects_ptr_.begin();
  end = objects_ptr_.end();

  Evaluator::compare_struct compare(this);
  std::sort(begin, end, compare);  // Sort by key

  int j = 1;
  for (it = begin; it != end; ++it)
  {
    std::cout << j++ << " ";
    std::cout << (*it)->name_ << std::endl;
  }
  std::cout << std::endl;
  it = begin;

  // Store the result to the response
  res.id_out = (*it)->id_;
  res.name = (*it)->name_;
  res.x = (*it)->glasses_.x_ + (*it)->offset_.x_;
  res.y = (*it)->glasses_.y_ + (*it)->offset_.y_;
  res.z = (*it)->glasses_.z_ + 0.15;

  objects_.clear();
  objects_ptr_.clear();

  return true;
}
