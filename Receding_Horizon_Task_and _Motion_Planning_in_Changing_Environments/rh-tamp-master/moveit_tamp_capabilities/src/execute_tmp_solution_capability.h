#ifndef EXECUTE_TMP_SOLUTION_H
#define EXECUTE_TMP_SOLUTION_H

#include <actionlib/server/simple_action_server.h>
#include <moveit/move_group/move_group_capability.h>

#include <moveit_tmp_msgs/ExecuteTMPSolutionAction.h>

#include <memory>

namespace move_group
{

class ExecuteTMPSolutionCapability : public MoveGroupCapability
{
public:
  ExecuteTMPSolutionCapability();

  virtual void initialize();

private:
  bool constructMotionPlan(
      const std::vector<moveit_tmp_msgs::SubTrajectory>& solution,
      plan_execution::ExecutableMotionPlan& plan);

  void
  goalCallback(const moveit_tmp_msgs::ExecuteTMPSolutionGoalConstPtr& goal);
  void preemptCallback();

  std::unique_ptr<
      actionlib::SimpleActionServer<moveit_tmp_msgs::ExecuteTMPSolutionAction>>
      as_;
};
} // namespace move_group

#endif // EXECUTE_TMP_SOLUTION_H
