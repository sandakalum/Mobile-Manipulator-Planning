#include <moveit_tmp/planner.h>

namespace moveit_tmp
{
Planner::Planner(const moveit::core::RobotModelConstPtr& model)
{
  planner_ = createPlanner(model);
}

bool Planner::plan(const planning_scene::PlanningSceneConstPtr& from,
                   const planning_scene::PlanningSceneConstPtr& to,
                   const moveit::core::JointModelGroup* jmg, double timeout,
                   robot_trajectory::RobotTrajectoryPtr& result)
{

  moveit_msgs::MotionPlanRequest req;
  req.group_name = jmg->getName();
  req.allowed_planning_time = timeout;
  req.start_state.is_diff = true;
  req.num_planning_attempts = 10;

  req.goal_constraints.resize(1);
  req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(
      to->getCurrentState(), jmg, 1e-5);

  planning_interface::MotionPlanResponse res;

  bool success = planner_->generatePlan(from, req, res);

  result = res.trajectory_;

  return success;
}

planning_pipeline::PlanningPipelinePtr
Planner::createPlanner(const moveit::core::RobotModelConstPtr& model,
                       const std::string& ns,
                       const std::string& planning_plugin_param_name,
                       const std::string& adapter_plugins_param_name)
{

  return std::make_shared<planning_pipeline::PlanningPipeline>(
      model, ros::NodeHandle(ns), planning_plugin_param_name,
      adapter_plugins_param_name);
}
} // namespace moveit_tmp
