#ifndef CURRENT_STATE_H
#define CURRENT_STATE_H

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/GetPlanningScene.h>

namespace moveit_tmp
{
class CurrentState
{
public:
  CurrentState(const moveit::core::RobotModelConstPtr& robot_model);

  bool compute(planning_scene::PlanningScenePtr& scene);

  void setTimeout(double timeout) { timeout_ = timeout; };

private:
  double timeout_;
  moveit::core::RobotModelConstPtr robot_model_;
  // planning_scene::PlanningScenePtr scene_;
};
} // namespace moveit_tmp

#endif // CURRENT_STATE_H
