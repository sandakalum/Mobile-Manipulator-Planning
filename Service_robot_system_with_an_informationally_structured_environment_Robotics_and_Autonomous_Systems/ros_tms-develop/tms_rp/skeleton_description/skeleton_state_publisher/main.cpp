#include <sstream>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <robot_state_publisher/robot_state_publisher.h>

#include <tms_msg_ss/SkeletonArray.h>
#include <tms_msg_db/TmsdbStamped.h>

#include "picojson.h"
#include "../calc_joint_angles/for_model01.h"

#define PARENT_LINK "world_link"
#define CHILD_LINK "Body"

const ros::Duration GMT(9 * 60 * 60);  // GMT: Tokyo +9 [sec]

//------------------------------------------------------------------------------
class SkeletonsStatePublisher
{
public:
  tf::TransformBroadcaster broadcaster_;

  SkeletonsStatePublisher(ros::NodeHandle& nh, const std::vector< KDL::Tree >& kdl_forest, const bool& usingDB = false);
  ~SkeletonsStatePublisher();

  void run();
  void send(ros::Time time);

private:
  ros::NodeHandle nh_;
  ros::Subscriber data_sub_;

  std::vector< robot_state_publisher::RobotStatePublisher > state_pubs_;  // Publish tf tree
  std::vector< tf::StampedTransform > transforms_;                        // Position & Rotation
  std::vector< std::map< std::string, double > > joint_states;            // Posture

  bool usingDB_;

  // Switch on whether use database.
  void callback(const tms_msg_ss::SkeletonArray::ConstPtr& msg);
  void callback2(const tms_msg_db::TmsdbStamped::ConstPtr& msg);
};

//------------------------------------------------------------------------------
SkeletonsStatePublisher::SkeletonsStatePublisher(ros::NodeHandle& nh, const std::vector< KDL::Tree >& kdl_forest,
                                                 const bool& usingDB)
  : nh_(nh), usingDB_(usingDB)
{
  if (usingDB_)
  {
    data_sub_ = nh_.subscribe("/tms_db_publisher", 1, &SkeletonsStatePublisher::callback2, this);
  }
  else
  {
    data_sub_ = nh_.subscribe("integrated_skeleton_stream", 1, &SkeletonsStatePublisher::callback, this);
  }

  ros::Time now = ros::Time::now() + GMT;

  for (int i = 0; i < kdl_forest.size(); i++)
  {
    state_pubs_.push_back(robot_state_publisher::RobotStatePublisher(kdl_forest[i]));

    std::stringstream tf_prefix;
    tf_prefix << "skeleton" << i + 1;
    transforms_.push_back(
        tf::StampedTransform(tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.0)), now,
                             PARENT_LINK, tf::resolve(tf_prefix.str(), CHILD_LINK)));
    std::map< std::string, double > joint_state;
    joint_states.push_back(joint_state);
  }

  return;
}

//------------------------------------------------------------------------------
SkeletonsStatePublisher::~SkeletonsStatePublisher()
{
  return;
}

//------------------------------------------------------------------------------
void SkeletonsStatePublisher::callback(const tms_msg_ss::SkeletonArray::ConstPtr& msg)
{
  for (int i = 0; i < msg->data.size(); i++)
  {
    Eigen::Vector3d pos;
    Eigen::Quaterniond rot;
    if (msg->data[i].user_id < 0)  // ID < 0 means invalid data.
    {
      // Set data to disappear from environment
      for (int j = 0; j < kJointDoF; j++)
      {
        joint_states[i][kJointName[j]] = 0.0;
      }
      transforms_[i].setData(tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.0)));
    }
    else
    {
      // Set data for drawing into environment
      calcForModel01< double >(msg->data[i], pos, rot, joint_states[i]);
      transforms_[i].setData(
          tf::Transform(tf::Quaternion(rot.x(), rot.y(), rot.z(), rot.w()), tf::Vector3(pos[0], pos[1], pos[2])));
    }
  }

  this->send(ros::Time::now() + GMT);

  return;
}

//------------------------------------------------------------------------------
void SkeletonsStatePublisher::callback2(const tms_msg_db::TmsdbStamped::ConstPtr& msg)
{
  for (int32_t i = 0; i < msg->tmsdb.size(); i++)
  {
    const int32_t human_id_start = 1006;
    if (msg->tmsdb[i].id >= human_id_start && msg->tmsdb[i].id < human_id_start + state_pubs_.size())
    {
      const tms_msg_db::Tmsdb& skeleton_db_data = msg->tmsdb[i];
      const int ref = msg->tmsdb[i].id - human_id_start;

      Eigen::Vector3d pos;
      Eigen::Quaterniond rot;

      // JSON parsing
      picojson::value val;
      picojson::parse(val, skeleton_db_data.etcdata);
      picojson::object& json_obj = val.get< picojson::object >();
      ROS_INFO("------\nstorage_prace: %d\nid: %d\nposition: %f, %f, %f\nrpy: %f, %f, %f\n\
          ---------------------------------\n",
               i, skeleton_db_data.id, skeleton_db_data.x, skeleton_db_data.y, skeleton_db_data.z, skeleton_db_data.rr,
               skeleton_db_data.rp, skeleton_db_data.ry);
      for (int j = 0; j < kJointDoF; j++)
      {
        joint_states[ref][kJointName[j]] = json_obj[kJointName[j]].get< double >();
      }

      tf::Quaternion q;
      q.setRPY(skeleton_db_data.rr, skeleton_db_data.rp, skeleton_db_data.ry);
      transforms_[ref].setData(
          tf::Transform(q, tf::Vector3(skeleton_db_data.x, skeleton_db_data.y, skeleton_db_data.z)));
    }
  }

  this->send(ros::Time::now() + GMT);

  return;
}

//------------------------------------------------------------------------------
void SkeletonsStatePublisher::run()
{
  ros::spin();
  return;
}

//------------------------------------------------------------------------------
void SkeletonsStatePublisher::send(ros::Time time)
{
  for (int i = 0; i < transforms_.size(); i++)
  {
    transforms_[i].stamp_ = time;
    std::stringstream tf_prefix;
    tf_prefix << "skeleton" << i + 1;
    broadcaster_.sendTransform(transforms_[i]);
    state_pubs_[i].publishTransforms(joint_states[i], time, tf_prefix.str());
  }
  return;
}

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "skeleton_state_publisher");

  // Get input from ros params
  ros::NodeHandle nh;
  int num_of_skeletons = 0;
  if (nh.getParam("num_of_skeletons", num_of_skeletons))
  {
    ROS_INFO("SkeletonsStatePublisher: Get the number of skeletons -> %d", num_of_skeletons);
  }
  bool usingDB = false;
  if (nh.getParam("using_db", usingDB))
  {
    if (usingDB)
    {
      ROS_INFO("SkeletonsStatePublisher: Get data from DB.");
    }
  }

  // Load models
  std::vector< KDL::Tree > kdl_forest;
  const std::string base_description_name("skeleton_description");

  kdl_forest.resize((num_of_skeletons == 0 ? 1 : num_of_skeletons));
  for (int i = 0; i < num_of_skeletons; i++)
  {
    urdf::Model model;
    std::stringstream description_name_stream;
    description_name_stream << base_description_name << i + 1;
    model.initParam(description_name_stream.str());
    if (!kdl_parser::treeFromUrdfModel(model, kdl_forest[i]))
    {
      ROS_ERROR("SkeletonsStatePublisher: Failed to extract kdl tree from xml robot description.\
          (model %d)",
                i + 1);
      return -2;
    }
  }

  // Run
  SkeletonsStatePublisher obj(nh, kdl_forest, usingDB);

  obj.run();

  return 0;
}
