#include <sstream>

#include <Eigen/Eigen>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <robot_state_publisher/robot_state_publisher.h>

#include <tms_msg_ss/SkeletonStreamWrapper.h>

#define PARENT_LINK "world_link"
#define CHILD_LINK "base_link"

const ros::Duration GMT(9 * 60 * 60);  // GMT: Tokyo +9 [sec]

//------------------------------------------------------------------------------
class KinectStatePublisher
{
public:
  tf::TransformBroadcaster broadcaster_;

  KinectStatePublisher(ros::NodeHandle& nh, const std::vector< KDL::Tree >& kdl_forest,
                       const std::vector< int >& cameraID_array);
  ~KinectStatePublisher();

  void run();
  void send(ros::Time time);

private:
  ros::NodeHandle nh_;

  const std::vector< int > cameraID_array_;

  std::vector< ros::Subscriber > data_subs_;
  std::vector< robot_state_publisher::RobotStatePublisher > state_pubs_;
  std::vector< tf::StampedTransform > transforms_;

  std::vector< Eigen::Vector3d > pos_;
  std::vector< Eigen::Quaterniond > rot_;

  void callback(const tms_msg_ss::SkeletonStreamWrapper::ConstPtr& msg);
};

//------------------------------------------------------------------------------
KinectStatePublisher::KinectStatePublisher(ros::NodeHandle& nh, const std::vector< KDL::Tree >& kdl_forest,
                                           const std::vector< int >& cameraID_array)
  : nh_(nh), cameraID_array_(cameraID_array)
{
  ros::Time now = ros::Time::now() + GMT;
  for (int i = 0; i < cameraID_array_.size(); i++)
  {
    std::stringstream subscribe_topic;
    subscribe_topic << "skeleton_stream_wrapper" << cameraID_array_[i];
    data_subs_.push_back(nh_.subscribe(subscribe_topic.str(), 1, &KinectStatePublisher::callback, this));

    state_pubs_.push_back(robot_state_publisher::RobotStatePublisher(kdl_forest[i]));

    std::stringstream tf_prefix;
    tf_prefix << "kinect" << cameraID_array_[i];
    transforms_.push_back(
        tf::StampedTransform(tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.0)), now,
                             PARENT_LINK, tf::resolve(tf_prefix.str(), CHILD_LINK)));
  }

  pos_.resize(cameraID_array_.size());
  rot_.resize(cameraID_array_.size());

  return;
}

//------------------------------------------------------------------------------
KinectStatePublisher::~KinectStatePublisher()
{
  return;
}

//------------------------------------------------------------------------------
void KinectStatePublisher::callback(const tms_msg_ss::SkeletonStreamWrapper::ConstPtr& msg)
{
  int camera_num = (int)msg->camera_number - 1;

  ROS_INFO("kinect%d: Received posture.", camera_num);

  tms_msg_ss::CameraPosture camera_posture = msg->camera_posture;
  rot_[camera_num] = Eigen::Quaterniond(camera_posture.rotation.w, camera_posture.rotation.x, camera_posture.rotation.y,
                                        camera_posture.rotation.z);
  pos_[camera_num] =
      Eigen::Vector3d(camera_posture.translation.x, camera_posture.translation.y, camera_posture.translation.z);

  return;
}

//------------------------------------------------------------------------------
void KinectStatePublisher::run()
{
  ros::Duration sleeper(10.0);
  while (nh_.ok())
  {
    ros::spinOnce();

    ros::Time time = ros::Time::now() + GMT + sleeper;

    for (int i = 0; i < cameraID_array_.size(); i++)
    {
      tf::Quaternion q(rot_[i].x(), rot_[i].y(), rot_[i].z(), rot_[i].w());
      transforms_[i].setData(tf::Transform(q, tf::Vector3(pos_[i][0], pos_[i][1], pos_[i][2])));
    }

    this->send(time);

    sleeper.sleep();
  }
  return;
}

//------------------------------------------------------------------------------
void KinectStatePublisher::send(ros::Time time)
{
  std::map< std::string, double > joint_states;
  for (int i = 0; i < cameraID_array_.size(); i++)
  {
    transforms_[i].stamp_ = time;
    std::stringstream tf_prefix;
    tf_prefix << "kinect" << cameraID_array_[i];
    broadcaster_.sendTransform(transforms_[i]);
    state_pubs_[i].publishTransforms(joint_states, time, tf_prefix.str());
  }
  return;
}

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_state_publisher");

  ros::NodeHandle nh;

  std::vector< int > cameraID_array;
  std::string using_numbers_str;
  if (!nh.getParam("using_numbers", using_numbers_str))
  {
    ROS_INFO("KinectStatePublisher: Need some camera id as arguments. Exiting...");
    return -1;
  }
  std::stringstream ss(using_numbers_str);
  std::string buffer;
  while (std::getline(ss, buffer, ' '))
  {
    cameraID_array.push_back(atoi(buffer.c_str()));
  }

  std::vector< KDL::Tree > kdl_forest;
  std::vector< urdf::Model > models;

  kdl_forest.resize(cameraID_array.size());
  for (int i = 0; i < cameraID_array.size(); i++)
  {
    urdf::Model model;
    std::stringstream description_name_stream;
    description_name_stream << "kinect_description";
    description_name_stream << cameraID_array[i];
    model.initParam(description_name_stream.str());
    models.push_back(model);
    if (!kdl_parser::treeFromUrdfModel(models[i], kdl_forest[i]))
    {
      ROS_ERROR("Failed to extract kdl tree from xml robot description");
      return -1;
    }
  }

  KinectStatePublisher obj(nh, kdl_forest, cameraID_array);

  obj.run();

  return 0;
}
