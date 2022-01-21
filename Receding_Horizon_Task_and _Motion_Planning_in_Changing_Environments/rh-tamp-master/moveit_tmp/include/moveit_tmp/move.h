#ifndef MOVE_H
#define MOVE_H

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_tmp/planner.h>

namespace moveit_tmp
{
class Move
{
public:
  Move(const PlannerPtr& planner, double timeout = 10.0);

  bool compute(const planning_scene::PlanningSceneConstPtr& start,
               const std::vector<double>& joint_values,
               planning_scene::PlanningScenePtr& goal,
               robot_trajectory::RobotTrajectoryPtr& trajectory);

  bool compute(const planning_scene::PlanningSceneConstPtr& start,
               const std::string& named_joint_pose,
               planning_scene::PlanningScenePtr& goal,
               robot_trajectory::RobotTrajectoryPtr& trajectory);

  void setIKFrame(const geometry_msgs::PoseStamped& pose) { ik_frame_ = pose; }
  void setIKFrame(const Eigen::Isometry3d& pose, const std::string& link);

  void setGroup(const std::string& group) { group_ = group; }

  void setTimeout(double timeout) { timeout_ = timeout; }

private:
  PlannerPtr planner_;
  std::string group_;
  double timeout_;
  geometry_msgs::PoseStamped ik_frame_;
};
} // namespace moveit_tmp

#endif // MOVE_H
