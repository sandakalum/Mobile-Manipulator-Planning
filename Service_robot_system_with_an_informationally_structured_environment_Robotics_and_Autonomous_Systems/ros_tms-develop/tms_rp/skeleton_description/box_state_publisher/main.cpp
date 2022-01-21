#include <sstream>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <robot_state_publisher/robot_state_publisher.h>

#include <tms_msg_ss/vicon_data.h>
#include <tms_msg_ss/SkeletonArray.h>

//------------------------------------------------------------------------------
class BoxStatePublisher : public robot_state_publisher::RobotStatePublisher
{
public:
  tf::TransformBroadcaster broadcaster;

  BoxStatePublisher(const KDL::Tree& tree, std::string tf_prefix = "");
  ~BoxStatePublisher();

  void run();
  void send(ros::Time time);

private:
  ros::NodeHandle nh;
  ros::Subscriber data_sub;
  ros::Publisher state_pub;
  sensor_msgs::JointState state_data;
  tf::StampedTransform transform_;

  std::string tf_prefix_;
  bool bFind;

  geometry_msgs::Point pos;
  geometry_msgs::Quaternion rot;

  void callback(const tms_msg_ss::vicon_data::ConstPtr& msg);
};

//------------------------------------------------------------------------------
BoxStatePublisher::BoxStatePublisher(const KDL::Tree& tree, std::string tf_prefix)
  : robot_state_publisher::RobotStatePublisher(tree), bFind(false), tf_prefix_(tf_prefix)
{
  data_sub = nh.subscribe("vicon_stream/output", 10, &BoxStatePublisher::callback, this);
  transform_ = tf::StampedTransform(tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.0)),
                                    ros::Time::now(), "world_link", tf::resolve(tf_prefix_, "Box"));
  return;
}

//------------------------------------------------------------------------------
BoxStatePublisher::~BoxStatePublisher()
{
  return;
}

//------------------------------------------------------------------------------
void BoxStatePublisher::callback(const tms_msg_ss::vicon_data::ConstPtr& msg)
{
  if (msg->subjectName == tf::resolve(tf_prefix_, "checker_box"))
  {
    pos = msg->translation;
    rot = msg->rotation;
  }

  return;
}

//------------------------------------------------------------------------------
void BoxStatePublisher::run()
{
  ros::Duration sleeper(0.1);
  while (nh.ok())
  {
    ros::Time time = ros::Time::now() + sleeper;

    ros::spinOnce();

    ROS_INFO("Got box state from vicon_stream");
    // [mm] -> [m]
    tf::Quaternion q(rot.x, rot.y, rot.z, rot.w);
    transform_.setData(tf::Transform(q, tf::Vector3(pos.x / 1000.0, pos.y / 1000.0, pos.z / 1000.0)));

    this->send(time);
    sleeper.sleep();
  }
  return;
}

//------------------------------------------------------------------------------
void BoxStatePublisher::send(ros::Time time)
{
  std::map< std::string, double > joint_states;
  transform_.stamp_ = time;
  broadcaster.sendTransform(transform_);
  this->publishTransforms(joint_states, time, tf_prefix_);
  return;
}

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  std::string tf_prefix("");
  // if (argc > 1)
  //{
  //  ROS_INFO("Set tf_prefix: %s.\n", argv[1]);
  //  tf_prefix.assign(argv[1]);
  //}

  ros::init(argc, argv, "box_state_publisher");

  urdf::Model model;
  std::stringstream description_name_stream;
  // if (!tf_prefix.empty())
  //{
  //  description_name_stream << tf_prefix << "_";
  //}
  description_name_stream << "box_description";

  model.initParam(description_name_stream.str());

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return -1;
  }

  BoxStatePublisher obj(tree);

  obj.run();

  return 0;
}
