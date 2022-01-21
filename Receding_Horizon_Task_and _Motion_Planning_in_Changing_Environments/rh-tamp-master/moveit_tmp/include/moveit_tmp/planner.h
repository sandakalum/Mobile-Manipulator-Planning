#ifndef PLANNER_H
#define PLANNER_H

#include <memory>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit_tmp
{
class Planner
{
public:
  Planner(const moveit::core::RobotModelConstPtr& model);

  bool plan(const planning_scene::PlanningSceneConstPtr& from,
            const planning_scene::PlanningSceneConstPtr& to,
            const moveit::core::JointModelGroup* jmg, double timeout,
            robot_trajectory::RobotTrajectoryPtr& result);

private:
  static planning_pipeline::PlanningPipelinePtr createPlanner(
      const moveit::core::RobotModelConstPtr& model,
      const std::string& ns = "move_group",
      const std::string& planning_plugin_param_name = "planning_plugin",
      const std::string& adapter_plugins_param_name = "request_adapters");

  planning_pipeline::PlanningPipelinePtr planner_;
};

typedef std::shared_ptr<Planner> PlannerPtr;
typedef std::shared_ptr<const Planner> PlannerConstPtr;
} // namespace moveit_tmp

#endif // PLANNER_H
