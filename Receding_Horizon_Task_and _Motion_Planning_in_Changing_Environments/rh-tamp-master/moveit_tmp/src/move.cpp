#include <moveit_tmp/move.h>

namespace moveit_tmp
{
Move::Move(const PlannerPtr& planner, double timeout)
    : planner_(planner), timeout_(timeout)
{
}

void Move::setIKFrame(const Eigen::Isometry3d& pose, const std::string& link)
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = link;
  tf::poseEigenToMsg(pose, pose_msg.pose);
  setIKFrame(pose_msg);
}

bool Move::compute(const planning_scene::PlanningSceneConstPtr& start,
                   const std::vector<double>& joint_values,
                   planning_scene::PlanningScenePtr& goal,
                   robot_trajectory::RobotTrajectoryPtr& trajectory)
{
  // planning_scene::PlanningScenePtr scene = start->diff();

  goal = start->diff();

  const auto& robot_model = goal->getRobotModel();
  assert(robot_model);

  const moveit::core::JointModelGroup* jmg =
      robot_model->getJointModelGroup(group_);

  if (!jmg)
  {
    ROS_WARN_STREAM_NAMED("Move", "Invalid joint model group: " << group_);
    return -1;
  }

  robot_state::RobotState& state = goal->getCurrentStateNonConst();

  state.setJointGroupPositions(jmg, joint_values);

  if (planner_->plan(start, goal, jmg, timeout_,
                     trajectory)) // if(robot_trajectory)
  {
    goal->setCurrentState(trajectory->getLastWayPoint());
    return true;
  }
  return false;
}

bool Move::compute(const planning_scene::PlanningSceneConstPtr& start,
                   const std::string& named_joint_pose,
                   planning_scene::PlanningScenePtr& goal,
                   robot_trajectory::RobotTrajectoryPtr& trajectory)
{
  // planning_scene::PlanningScenePtr scene = start->diff();

  goal = start->diff();

  const auto& robot_model = goal->getRobotModel();
  assert(robot_model);

  const moveit::core::JointModelGroup* jmg =
      robot_model->getJointModelGroup(group_);

  if (!jmg)
  {
    ROS_WARN_STREAM_NAMED("Move", "Invalid joint model group: " << group_);
    return -1;
  }

  robot_state::RobotState& state = goal->getCurrentStateNonConst();

  if (state.setToDefaultValues(jmg, named_joint_pose))
  {
    state.update();
  }
  else
  {
    ROS_WARN_STREAM_NAMED("Move", "Unknown joint pose: " << named_joint_pose);
    return false;
  }

  if (planner_->plan(start, goal, jmg, timeout_,
                     trajectory)) // if(robot_trajectory)
  {
    goal->setCurrentState(trajectory->getLastWayPoint());
    return true;
  }
  return false;
}
} // namespace moveit_tmp
