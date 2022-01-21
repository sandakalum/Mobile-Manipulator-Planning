#include <iostream>
#include <string>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tms_msg_ss/SkeletonArray.h>
#include <tms_msg_ss/CameraPosture.h>
#include <tms_msg_ss/SkeletonStreamWrapper.h>

static const float marker_color[][3] = {  // BGR
    {1.0f, 0, 0},
    {0, 1.0f, 0},
    {0, 0, 1.0f},
    {1.0f, 1.0f, 0},
    {1.0f, 0, 1.0f},
    {0, 1.0f, 1.0f}};

class SkeletonViewer
{
public:
  SkeletonViewer();
  ~SkeletonViewer();
  void callback_skeleton(const tms_msg_ss::SkeletonArray::ConstPtr &msg);
  void run();

private:
  ros::NodeHandle _nh;
  ros::Publisher *_pmarker_array_pub;
  bool gotCameraPosture;
  Eigen::Vector3f translation;
  Eigen::Quaternionf rotation;

  inline void toWorldPoint(Eigen::Vector3f &vec);
};

SkeletonViewer::SkeletonViewer() : gotCameraPosture(false)
{
}

SkeletonViewer::~SkeletonViewer()
{
}

inline void SkeletonViewer::toWorldPoint(Eigen::Vector3f &vec)
{
  vec = this->rotation.matrix() * vec + this->translation;
  return;
}

template < class T >
std::string to_str(const T &t)
{
  std::stringstream ss;
  ss << t;
  return ss.str();
}

void SkeletonViewer::callback_skeleton(const tms_msg_ss::SkeletonArray::ConstPtr &msg)
{
  visualization_msgs::MarkerArray marker_array;
  for (int j = 0; j < msg->data.size(); j++)
  {
    tms_msg_ss::Skeleton skeleton = msg->data[j];
    ROS_INFO("Received skeleton %d", skeleton.user_id);

    uint32_t shape = visualization_msgs::Marker::SPHERE;
    for (int i = 0; i < 25; i++)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/world_link";
      marker.header.stamp = ros::Time::now();
      std::string name("skeleton");
      marker.ns = name.append(to_str< int >(j + 1));
      marker.id = i;
      marker.type = shape;
      marker.action = visualization_msgs::Marker::ADD;
      Eigen::Vector3f pos(skeleton.position[i].x, skeleton.position[i].y, skeleton.position[i].z);
      if (gotCameraPosture)
      {
        this->toWorldPoint(pos);
      }
      marker.pose.position.x = pos.x();
      marker.pose.position.y = pos.y();
      marker.pose.position.z = pos.z();
      marker.pose.orientation.x = skeleton.orientation[i].x;
      marker.pose.orientation.y = skeleton.orientation[i].y;
      marker.pose.orientation.z = skeleton.orientation[i].z;
      marker.pose.orientation.w = skeleton.orientation[i].w;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.r = marker_color[j / 6][2];
      marker.color.g = marker_color[j / 6][1];
      marker.color.b = marker_color[j / 6][0];
      marker.color.a = 1.0f * (float)skeleton.confidence[i] / 2.0f;
      marker.lifetime = ros::Duration();
      marker_array.markers.push_back(marker);
    }
  }
  _pmarker_array_pub->publish(marker_array);

  return;
}

void SkeletonViewer::run()
{
  ros::Subscriber sub_skeleton =
      this->_nh.subscribe("integrated_skeleton_stream", 1, &SkeletonViewer::callback_skeleton, this);
  ros::Publisher marker_pub = _nh.advertise< visualization_msgs::MarkerArray >("skeleton_visualization", 1);
  _pmarker_array_pub = &marker_pub;
  ros::spin();
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "skeleton_viewer");

  SkeletonViewer viewer;

  viewer.run();

  return 0;
}
