#ifndef _EVALUATOR_H
#define _EVALUATOR_H

#include "tms_msg_db/TmsdbGetData.h"
#include "tms_ur_gaze_server/object_list.h"

#include <ros/ros.h>

#define _USE_MATH_DEFINES
#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <algorithm>
#include <cstdlib>

//----------------------------------------------------------------------------------
// 3-dimensional vector
//----------------------------------------------------------------------------------
class Vec3
{
public:
  double x_;
  double y_;
  double z_;
  double length_;
  double argument_;
  void CalcAbs();
  void CalcArg();
};

//----------------------------------------------------------------------------------
// object
//----------------------------------------------------------------------------------
class Object
{
private:
  ros::NodeHandle* nh_;

public:
  std::string name_;
  int id_;
  Vec3 world_;
  Vec3 offset_;
  Vec3 glasses_;
  double sort_key_;

  Object(ros::NodeHandle* nh);
  bool GetGeometry(int id);
};

//----------------------------------------------------------------------------------
// sort-key evaluator
//----------------------------------------------------------------------------------
class Evaluator
{
  // non-static compare method
  struct compare_struct
  {
    Evaluator* ptr_;
    compare_struct(Evaluator* ptr) : ptr_(ptr)
    {
    }
    bool operator()(Object* object1, Object* object2)
    {
      return object1->sort_key_ < object2->sort_key_;
    }
  };

private:
  ros::NodeHandle* nh_;
  ros::ServiceServer sever_;
  std::vector< Object > objects_;
  std::vector< Object* > objects_ptr_;
  Object object_tmp_;

  double yaw_;
  double pitch_;

  // pitching
  double cos_pitch_;
  double sin_pitch_;

  // yawing
  double cos_yaw_;
  double sin_yaw_;

  // translation
  Vec3 trans_;

public:
  Evaluator(ros::NodeHandle* nh);
  ~Evaluator();
  void StoreAddress(std::vector< Object* >* object, std::vector< Object >* object_data);
  bool CallBack(tms_ur_gaze_server::object_list::Request& req, tms_ur_gaze_server::object_list::Response& res);
};

#endif
