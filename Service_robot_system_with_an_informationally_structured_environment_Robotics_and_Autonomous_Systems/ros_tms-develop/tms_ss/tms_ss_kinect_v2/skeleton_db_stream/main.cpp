#include <iostream>
#include <sstream>
#include <iomanip>

#include <ros/ros.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <tf/LinearMath/Matrix3x3.h>

#include <tms_msg_ss/SkeletonArray.h>
#include <tms_msg_db/TmsdbStamped.h>

#include "for_model01.h"

#define SKELETON_ID 1006

//------------------------------------------------------------------------------
class SkeletonDBStream
{
public:
  SkeletonDBStream();
  ~SkeletonDBStream();
  void callback(const tms_msg_ss::SkeletonArray::ConstPtr& msg);

private:
  // NodeHandle
  ros::NodeHandle nh_;
  // Publisher
  ros::Publisher db_pub_;
  // Subscriber
  ros::Subscriber skeleton_sub_;

  double update_time_;
  std::string frame_id_;

  std::string makeJSONofJointAngles(std::map< std::string, double >& data);
};

//------------------------------------------------------------------------------
SkeletonDBStream::SkeletonDBStream() : frame_id_("/world"), update_time_(0.01)  // sec
{
  db_pub_ = nh_.advertise< tms_msg_db::TmsdbStamped >("tms_db_data", 1);
  skeleton_sub_ = nh_.subscribe("integrated_skeleton_stream", 1, &SkeletonDBStream::callback, this);

  ROS_ASSERT("Start sending skeleton data to DB.");
  return;
}

//------------------------------------------------------------------------------
SkeletonDBStream::~SkeletonDBStream()
{
  ROS_ASSERT("Stop sending skeleton data to DB.");
  return;
}

//------------------------------------------------------------------------------
void SkeletonDBStream::callback(const tms_msg_ss::SkeletonArray::ConstPtr& msg)
{
  // Assumption: All of skeletons are detected at the same time.
  ros::Time now = msg->data[0].header.stamp;

  tms_msg_db::TmsdbStamped db_msg;

  db_msg.header.frame_id = frame_id_;
  db_msg.header.stamp = now;
  for (int i = 0; i < msg->data.size(); i++)
  {
    const tms_msg_ss::Skeleton& skeleton = msg->data[i];

    tms_msg_db::Tmsdb data;
    // Data name
    std::stringstream data_name;
    data_name << "Human_" << std::setw(3) << std::setfill('0') << i + 1;
    // Calculation of position and posture
    Eigen::Vector3d pos;
    Eigen::Quaterniond quat;
    std::map< std::string, double > joint_states;
    calcForModel01< double >(skeleton, pos, quat, joint_states);
    double rr, rp, ry;
    tf::Matrix3x3(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w())).getRPY(rr, rp, ry);

    // set data for DB
    // ref: ~/catkin_ws/src/ros_tms/tms_msg/tms_msg_db
    data.time = boost::posix_time::to_iso_extended_string(now.toBoost());
    // data.type
    data.id = SKELETON_ID + i;
    data.name = data_name.str();
    data.x = pos[0];
    data.y = pos[1];
    data.z = pos[2];
    data.rr = rr;
    data.rp = rp;
    data.ry = ry;
    // data.offset_x
    // data.offset_y
    // data.offset_z
    // data.joint
    // data.weight
    // data.rfid
    data.etcdata = makeJSONofJointAngles(joint_states);
    // data.place
    // data.extfile
    // data.sensor
    // data.probability
    data.state = 1;
    // data.state = (msg->data[i].user_id < 0 ? 0 : 2);
    // data.task
    // data.note
    // data.tag

    db_msg.tmsdb.push_back(data);
    ROS_INFO("Sending data to DB %d", data.id);
  }

  db_pub_.publish(db_msg);
  return;
}

//------------------------------------------------------------------------------
std::string SkeletonDBStream::makeJSONofJointAngles(std::map< std::string, double >& data)
{
  std::stringstream json;
  json << "{";
  for (int i = 0; i < kJointDoF; i++)
  {
    const std::string joint_name(kJointName[i]);
    json << "\"" << joint_name << "\":" << data[joint_name] << (i == kJointDoF - 1 ? "" : ",");
  }
  json << "}";
  return json.str();
}

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "skeleton_db_stream");
  SkeletonDBStream sds;
  ros::spin();
  return 0;
}
