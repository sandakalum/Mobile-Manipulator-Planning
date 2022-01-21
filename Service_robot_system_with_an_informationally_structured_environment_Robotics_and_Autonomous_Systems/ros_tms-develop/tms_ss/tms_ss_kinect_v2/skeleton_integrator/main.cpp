#include <iostream>
#include <vector>
#include <queue>

#include <string>
#include <sstream>
#include <iomanip>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <tms_msg_ss/SkeletonArray.h>
#include <tms_msg_ss/SkeletonStreamWrapper.h>

#include <for_model01.h>

#define MAX_USERS 12

#define IDENTIFY_RANGE 0.5     // [m]
#define FRONT_ANGLE_RANGE 130  // [deg]

//-----------------------------------------------------------------------------
template < class T >
std::string to_str(const T& t)
{
  std::stringstream ss;
  ss << t;
  return ss.str();
}

//-----------------------------------------------------------------------------
inline tms_msg_ss::Skeleton initialize_skeleton()
{
  tms_msg_ss::Skeleton ret;
  ret.user_id = -1;
  ret.position.resize(JOINT_NUM);
  ret.orientation.resize(JOINT_NUM);
  ret.confidence.resize(JOINT_NUM);
  for (int i = 0; i < JOINT_NUM; i++)
  {
    ret.position[i].x = 0.0;
    ret.position[i].y = 0.0;
    ret.position[i].z = 0.0;
    ret.orientation[i].w = 1.0;
    ret.orientation[i].x = 0.0;
    ret.orientation[i].y = 0.0;
    ret.orientation[i].z = 0.0;
    ret.confidence[i] = 0;
  }
  return ret;
}

//-----------------------------------------------------------------------------
class SkeletonIntegrator
{
public:
  SkeletonIntegrator(const std::vector< int >& camera, ros::NodeHandle& nh);
  ~SkeletonIntegrator();

  void callback(const tms_msg_ss::SkeletonStreamWrapper::ConstPtr& msg);
  void run();

private:
  ros::NodeHandle& nh_;
  std::vector< int > array;  // using camera's id array
  tms_msg_ss::SkeletonArray out;
  int tracked_skeleton_num_;
  int tracking_validity[MAX_USERS];

  std::vector< std::map< int, bool > > bFaceStateTable;       // [user_index, [camera_id, is_front?]]
  std::vector< std::pair< int, float > > storage_evaluation;  // [user_index, [camera_id, value]]

  void listenSkeletonStream();

  // For Debug
  void showStatus();
};

//-----------------------------------------------------------------------------
SkeletonIntegrator::SkeletonIntegrator(const std::vector< int >& camera, ros::NodeHandle& nh)
  : array(camera), tracked_skeleton_num_(0), nh_(nh)
{
  out.data.resize(MAX_USERS);
  bFaceStateTable.resize(MAX_USERS);
  for (int i = 0; i < MAX_USERS; i++)
  {
    out.data[i] = initialize_skeleton();
    tracking_validity[i] = 0;
    for (std::vector< int >::iterator it = array.begin(); it != array.end(); it++)
    {
      bFaceStateTable[i][*it] = false;
    }
    storage_evaluation.push_back(std::pair< int, float >(-1, 1));
  }
  return;
}

//-----------------------------------------------------------------------------
SkeletonIntegrator::~SkeletonIntegrator()
{
  return;
}

//-----------------------------------------------------------------------------
void SkeletonIntegrator::callback(const tms_msg_ss::SkeletonStreamWrapper::ConstPtr& msg)
{
  tms_msg_ss::Skeleton skeleton = msg->skeleton;
  tms_msg_ss::CameraPosture camera_posture = msg->camera_posture;

  // Transform to world coordinate
  Eigen::Vector3f translation(camera_posture.translation.x, camera_posture.translation.y, camera_posture.translation.z);
  Eigen::Quaternionf rotation(camera_posture.rotation.w, camera_posture.rotation.x, camera_posture.rotation.y,
                              camera_posture.rotation.z);

  Eigen::Matrix3f rot_mat = rotation.matrix();
  ROS_INFO("Camera %d status:\n  \
      Position in world:\n    \
      (%3.4f, %3.4f, %3.4f)\n  \
      Rotation in world:\n    \
      x-axis (%3.4f, %3.4f, %3.4f)\n    \
      y-axis (%3.4f, %3.4f, %3.4f)\n    \
      z-axis (%3.4f, %3.4f, %3.4f)\n\
      \n",
           msg->camera_number, translation[0], translation[1], translation[2], rot_mat(0, 0), rot_mat(1, 0),
           rot_mat(2, 0), rot_mat(0, 1), rot_mat(1, 1), rot_mat(2, 1), rot_mat(0, 2), rot_mat(1, 2), rot_mat(2, 2));

  tms_msg_ss::Skeleton integrated_skeleton;
  integrated_skeleton.user_id = 0;
  integrated_skeleton.position.resize(JOINT_NUM);
  integrated_skeleton.orientation.resize(JOINT_NUM);
  integrated_skeleton.confidence.resize(JOINT_NUM);
  for (int i = 0; i < JOINT_NUM; i++)
  {
    Eigen::Vector3f pos(skeleton.position[i].x, skeleton.position[i].y, skeleton.position[i].z);
    Eigen::Quaternionf ori(skeleton.orientation[i].w, skeleton.orientation[i].x, skeleton.orientation[i].y,
                           skeleton.orientation[i].z);
    pos = rotation.matrix() * pos + translation;
    ori = rotation * ori;
    integrated_skeleton.position[i].x = pos[0];
    integrated_skeleton.position[i].y = pos[1];
    integrated_skeleton.position[i].z = pos[2];
    integrated_skeleton.orientation[i].w = ori.w();
    integrated_skeleton.orientation[i].x = ori.x();
    integrated_skeleton.orientation[i].y = ori.y();
    integrated_skeleton.orientation[i].z = ori.z();
    integrated_skeleton.confidence[i] = skeleton.confidence[i];
  }

  // identify received skeleton
  int i;
  int index, user_id;
  for (i = 0; i < out.data.size(); i++)
  {
    Eigen::Vector3f v1(integrated_skeleton.position[SpineMid].x, integrated_skeleton.position[SpineMid].y,
                       integrated_skeleton.position[SpineMid].z);
    Eigen::Vector3f v2(out.data[i].position[SpineMid].x, out.data[i].position[SpineMid].y,
                       out.data[i].position[SpineMid].z);
    if ((v1 - v2).norm() <= IDENTIFY_RANGE)
    {
      index = i;
      user_id = out.data[i].user_id;
      break;
    }
  }
  if (i == MAX_USERS)  // If not detected yet, treat as new skeleton
  {
    user_id = tracked_skeleton_num_;
    for (i = 0; i < MAX_USERS; i++)
    {
      if (out.data[i].user_id < 0)  // user_id < 0 means empty
      {
        index = i;
        break;
      }
    }
    if (i == MAX_USERS)
    {
      ROS_INFO("SkeletonIntegrator: Data buffer is full. Discarding data.");
      return;
    }
    tracked_skeleton_num_++;
  }

  // Update skeleton state
  bool& table_ref = bFaceStateTable[index][msg->camera_number];
  integrated_skeleton.user_id = user_id;

  // Calculate skeleton direction (front is y)
  Eigen::Vector3f tmp_x(skeleton.position[HipRight].x - skeleton.position[HipLeft].x,
                        skeleton.position[HipRight].y - skeleton.position[HipLeft].y,
                        skeleton.position[HipRight].z - skeleton.position[HipLeft].z);
  Eigen::Vector3f tmp_z(skeleton.position[SpineMid].x - skeleton.position[SpineBase].x,
                        skeleton.position[SpineMid].y - skeleton.position[SpineBase].y,
                        skeleton.position[SpineMid].z - skeleton.position[SpineBase].z);
  Eigen::Vector3f skeleton_direction_cam;
  Eigen::Vector3f camera_direction_cam;
  skeleton_direction_cam = (tmp_z.cross(tmp_x)).normalized();
  camera_direction_cam = Eigen::Vector3f::UnitZ();
  float dot_skeleton_camera = skeleton_direction_cam.dot(camera_direction_cam);

  if (msg->face_state == 2)
  {
    // Log face state and storage as valid skeleton
    table_ref = true;
  }
  else
  {
    if (table_ref)
    {
      Eigen::Quaternionf skeleton_rotation(
          integrated_skeleton.orientation[SpineBase].w, integrated_skeleton.orientation[SpineBase].x,
          integrated_skeleton.orientation[SpineBase].y, integrated_skeleton.orientation[SpineBase].z);
      // Check that skeleton is facing the camera
      if (dot_skeleton_camera <= cos(FRONT_ANGLE_RANGE * M_PI / 180))
      {
        // storage as valid skeleton
      }
      else
      {
        table_ref = false;
        // Allow only move
        const tms_msg_ss::Skeleton& last_state = out.data[index];
        Eigen::Vector3d translation(integrated_skeleton.position[SpineMid].x - last_state.position[SpineMid].x,
                                    integrated_skeleton.position[SpineMid].y - last_state.position[SpineMid].y,
                                    integrated_skeleton.position[SpineMid].z - last_state.position[SpineMid].z);
        for (int i = 0; i < integrated_skeleton.position.size(); i++)
        {
          integrated_skeleton.position[i].x = last_state.position[i].x + translation[0];
          integrated_skeleton.position[i].y = last_state.position[i].y + translation[1];
          integrated_skeleton.position[i].z = last_state.position[i].z + translation[2];
        }
      }
    }
    else
    {
      // Allow only move
      const tms_msg_ss::Skeleton& last_state = out.data[index];
      Eigen::Vector3d translation(integrated_skeleton.position[SpineMid].x - last_state.position[SpineMid].x,
                                  integrated_skeleton.position[SpineMid].y - last_state.position[SpineMid].y,
                                  integrated_skeleton.position[SpineMid].z - last_state.position[SpineMid].z);
      for (int i = 0; i < integrated_skeleton.position.size(); i++)
      {
        integrated_skeleton.position[i].x = last_state.position[i].x + translation[0];
        integrated_skeleton.position[i].y = last_state.position[i].y + translation[1];
        integrated_skeleton.position[i].z = last_state.position[i].z + translation[2];
      }
    }
  }

  // Choose better data
  if ((msg->camera_number == storage_evaluation[index].first ||
       dot_skeleton_camera <= storage_evaluation[index].second) &&
      table_ref)
  {
    out.data[index] = integrated_skeleton;
    tracking_validity[index] = 15;  // reflesh validity
    // Update evaluation
    storage_evaluation[index].first = msg->camera_number;
    storage_evaluation[index].second = dot_skeleton_camera;
  }

  return;
}

//-----------------------------------------------------------------------------
void SkeletonIntegrator::run()
{
  ros::Rate loop_late(10);

  static boost::thread thread_listener(boost::bind(&SkeletonIntegrator::listenSkeletonStream, this));

  ros::Publisher pub = nh_.advertise< tms_msg_ss::SkeletonArray >("integrated_skeleton_stream", 1);

  while (ros::ok())
  {
    pub.publish(out);
    for (int i = 0; i < MAX_USERS; i++)
    {
      // decrease validity of skeleton whenever publish
      if (tracking_validity[i] > 0)
      {
        tracking_validity[i]--;
      }

      // eliminate invalid skeletons
      if (tracking_validity[i] == 0)
      {
        out.data[i] = initialize_skeleton();
        for (std::vector< int >::iterator camera_id = array.begin(); camera_id != array.end(); camera_id++)
        {
          bFaceStateTable[i][*camera_id] = false;
        }
      }
    }

    showStatus();
    ros::spinOnce();
    loop_late.sleep();
  }

  return;
}

//-----------------------------------------------------------------------------
void SkeletonIntegrator::listenSkeletonStream()
{
  ros::Rate loop_late(10);
  std::vector< ros::Subscriber > sub(array.size());

  std::cout << "Subscribe ===" << std::endl;
  for (int i = 0; i < array.size(); i++)
  {
    std::string topic_name("skeleton_stream_wrapper");
    sub[i] = nh_.subscribe(topic_name.append(to_str< int >(array[i])), 1, &SkeletonIntegrator::callback, this);
  }

  ros::spin();

  return;
}

//-----------------------------------------------------------------------------
void SkeletonIntegrator::showStatus()
{
  std::stringstream ss;
  ss.fill(' ');

  ss << "User ID" << std::endl;
  for (int i = 0; i < MAX_USERS; i++)
  {
    ss << "|" << std::setw(5) << i;
  }
  ss << "|" << std::endl;
  for (int i = 0; i < MAX_USERS; i++)
  {
    ss << "|" << std::setw(5) << out.data[i].user_id;
  }
  ss << "|" << std::endl
     << std::endl;
  ;

  ss << "Validity table" << std::endl;
  for (int i = 0; i < MAX_USERS; i++)
  {
    ss << "|" << std::setw(2) << i;
  }
  ss << "|" << std::endl;
  for (int i = 0; i < MAX_USERS; i++)
  {
    ss << "|" << std::setw(2) << tracking_validity[i];
  }
  ss << "|" << std::endl
     << std::endl;

  ss << "Face state table" << std::endl;
  for (std::vector< int >::iterator it = array.begin(); it != array.end(); it++)
  {
    if (*it == 0)
    {
      continue;
    }
    ss << "|" << std::setw(2) << *it << "|";
    for (int i = 0; i < MAX_USERS; i++)
    {
      if (bFaceStateTable[i][*it])
      {
        ss << "T|";
      }
      else
      {
        ss << "F|";
      }
    }
    ss << std::endl;
  }

  ROS_INFO("%s", ss.str().c_str());
  return;
}

//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{
  std::vector< int > cameraID_array;

  ros::init(argc, argv, "skeleton_integrator");

  ros::NodeHandle nh;

  std::string using_numbers_str;
  if (!nh.getParam("using_numbers", using_numbers_str))
  {
    ROS_INFO("SkeletonIntegrator: Need some camera id as arguments. Exiting...");
    return -1;
  }

  std::stringstream ss(using_numbers_str);
  std::string buffer;
  while (std::getline(ss, buffer, ' '))
  {
    cameraID_array.push_back(atoi(buffer.c_str()));
  }

  SkeletonIntegrator integrator(cameraID_array, nh);

  integrator.run();

  return 0;
}
