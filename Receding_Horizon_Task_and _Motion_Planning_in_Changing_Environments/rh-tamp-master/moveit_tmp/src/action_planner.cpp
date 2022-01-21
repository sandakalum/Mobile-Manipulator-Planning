#include <moveit_tmp/action_planner.h>

namespace moveit_tmp
{
ActionPlanner::ActionPlanner(const std::string& name) : name_(name) {}

void ActionPlanner::setIKFrame(const Eigen::Isometry3d& pose,
                               const std::string& link)
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = link;
  tf::poseEigenToMsg(pose, pose_msg.pose);
  setIKFrame(pose_msg);
}

} // namespace moveit_tmp
