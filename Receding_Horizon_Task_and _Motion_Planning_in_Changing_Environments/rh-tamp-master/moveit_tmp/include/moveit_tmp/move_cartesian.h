#ifndef MOVE_CARTESIAN_H
#define MOVE_CARTESIAN_H

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <geometry_msgs/Vector3Stamped.h>

#include <eigen_conversions/eigen_msg.h>

namespace moveit_tmp
{
class MoveCartesian
{
public:
  MoveCartesian(const moveit::core::RobotModelConstPtr& model);

  void setStepSize(double step_size) { step_size_ = step_size; }
  void setJumpThreshold(double jump_threshold)
  {
    jump_threshold_ = jump_threshold;
  }
  void setMinFraction(double min_fraction) { min_fraction_ = min_fraction; }

  void setMaxVelocityScaling(double factor)
  {
    max_velocity_scaling_factor_ = factor;
  }
  void setMaxAccelerationScaling(double factor)
  {
    max_acceleration_scaling_factor_ = factor;
  }

  void setGroup(const std::string& group) { group_ = group; }

  bool compute(const planning_scene::PlanningSceneConstPtr& start,
               const geometry_msgs::Vector3Stamped& direction,
               const double min_distance, const double max_distance,
               planning_scene::PlanningScenePtr& goal,
               robot_trajectory::RobotTrajectoryPtr& trajectory);

private:
  bool plan(const planning_scene::PlanningSceneConstPtr& from,
            const planning_scene::PlanningSceneConstPtr& to,
            const moveit::core::JointModelGroup* jmg, /*double timeout,*/
            robot_trajectory::RobotTrajectoryPtr& result);

  bool plan(const planning_scene::PlanningSceneConstPtr& from,
            const moveit::core::LinkModel& link,
            const Eigen::Isometry3d& target,
            const moveit::core::JointModelGroup* jmg, /*double timeout,*/
            robot_trajectory::RobotTrajectoryPtr& result);

  double jump_threshold_;
  double min_fraction_;
  double max_acceleration_scaling_factor_;
  double max_velocity_scaling_factor_;
  double step_size_;

  std::string group_;
  double timeout_;
  geometry_msgs::PoseStamped ik_frame_;
};
} // namespace moveit_tmp

#endif // MOVE_CARTESIAN_H
