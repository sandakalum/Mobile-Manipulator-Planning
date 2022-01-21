#include "execute_tmp_solution_capability.h"

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#if MOVEIT_MASTER
#include <moveit/utils/message_checks.h>
#endif

namespace
{
// TODO: move to moveit::core::RobotModel
const moveit::core::JointModelGroup*
findJointModelGroup(const moveit::core::RobotModel& model,
                    const std::vector<std::string>& joints)
{
  std::set<std::string> joint_set(joints.begin(), joints.end());

  const std::vector<const moveit::core::JointModelGroup*>& jmgs =
      model.getJointModelGroups();

  for (const moveit::core::JointModelGroup* jmg : jmgs)
  {
    const std::vector<std::string>& jmg_joints = jmg->getJointModelNames();
    std::set<std::string> jmg_joint_set(jmg_joints.begin(), jmg_joints.end());

    // return group if sets agree on all active joints
    if (std::includes(jmg_joint_set.begin(), jmg_joint_set.end(),
                      joint_set.begin(), joint_set.end()))
    {
      std::set<std::string> difference;
      std::set_difference(jmg_joint_set.begin(), jmg_joint_set.end(),
                          joint_set.begin(), joint_set.end(),
                          std::inserter(difference, difference.begin()));
      unsigned int acceptable = 0;
      for (const std::string& diff_joint : difference)
      {
        const moveit::core::JointModel* diff_jm =
            model.getJointModel(diff_joint);
        if (diff_jm->isPassive() || diff_jm->getMimic() ||
            diff_jm->getType() == moveit::core::JointModel::FIXED)
          ++acceptable;
      }
      if (difference.size() == acceptable)
        return jmg;
    }
  }

  return nullptr;
}
} // namespace

namespace move_group
{

ExecuteTMPSolutionCapability::ExecuteTMPSolutionCapability()
    : MoveGroupCapability("ExecuteTMPSolution")
{
}

void ExecuteTMPSolutionCapability::initialize()
{
  // configure the action server
  as_.reset(new actionlib::SimpleActionServer<
            moveit_tmp_msgs::ExecuteTMPSolutionAction>(
      root_node_handle_, "execute_tmp_solution",
      std::bind(&ExecuteTMPSolutionCapability::goalCallback, this,
                std::placeholders::_1),
      false));
  as_->registerPreemptCallback(
      std::bind(&ExecuteTMPSolutionCapability::preemptCallback, this));
  as_->start();
}

void ExecuteTMPSolutionCapability::goalCallback(
    const moveit_tmp_msgs::ExecuteTMPSolutionGoalConstPtr& goal)
{
  moveit_tmp_msgs::ExecuteTMPSolutionResult result;

  if (!context_->plan_execution_)
  {
    const std::string response =
        "Cannot execute solution. ~allow_trajectory_execution was set to false";
    result.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
    as_->setAborted(result, response);
    return;
  }

  plan_execution::ExecutableMotionPlan plan;
  if (!constructMotionPlan(goal->solution, plan))
    result.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
  else
  {
    ROS_INFO_NAMED("ExecuteTaskSolution", "Executing TaskSolution");
    result.error_code = context_->plan_execution_->executeAndMonitor(plan);
  }

  const std::string response =
      context_->plan_execution_->getErrorCodeString(result.error_code);

  if (result.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    as_->setSucceeded(result, response);
  else if (result.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
    as_->setPreempted(result, response);
  else
    as_->setAborted(result, response);
}

void ExecuteTMPSolutionCapability::preemptCallback()
{
  if (context_->plan_execution_)
    context_->plan_execution_->stop();
}

bool ExecuteTMPSolutionCapability::constructMotionPlan(
    const std::vector<moveit_tmp_msgs::SubTrajectory>& solution,
    plan_execution::ExecutableMotionPlan& plan)
{
  robot_model::RobotModelConstPtr model =
      context_->planning_scene_monitor_->getRobotModel();

  robot_state::RobotState state(model);
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(
        context_->planning_scene_monitor_);
    state = scene->getCurrentState();
  }

  plan.plan_components_.reserve(solution.size());
  for (size_t i = 0; i < solution.size(); ++i)
  {
    const moveit_tmp_msgs::SubTrajectory& sub_traj = solution[i];

    plan.plan_components_.emplace_back();
    plan_execution::ExecutableTrajectory& exec_traj =
        plan.plan_components_.back();

    // define individual variable for use in closure below
    const std::string description =
        std::to_string(i + 1) + "/" + std::to_string(solution.size());
    exec_traj.description_ = description;

    const moveit::core::JointModelGroup* group = nullptr;
    {
      std::vector<std::string> joint_names(
          sub_traj.trajectory.joint_trajectory.joint_names);
      joint_names.insert(
          joint_names.end(),
          sub_traj.trajectory.multi_dof_joint_trajectory.joint_names.begin(),
          sub_traj.trajectory.multi_dof_joint_trajectory.joint_names.end());
      if (joint_names.size())
      {
        group = findJointModelGroup(*model, joint_names);
        if (!group)
        {
          ROS_ERROR_STREAM_NAMED(
              "ExecuteTMPSolution",
              "Could not find JointModelGroup that actuates {"
                  << boost::algorithm::join(joint_names, ", ") << "}");
          return false;
        }
        ROS_DEBUG_NAMED("ExecuteTMPSolution",
                        "Using JointModelGroup '%s' for execution",
                        group->getName().c_str());
      }
    }
    exec_traj.trajectory_ =
        std::make_shared<robot_trajectory::RobotTrajectory>(model, group);
    exec_traj.trajectory_->setRobotTrajectoryMsg(state, sub_traj.trajectory);


    /* TODO add action feedback and markers */
    exec_traj.effect_on_success_ =
        [this, sub_traj,
         description](const plan_execution::ExecutableMotionPlan*) {
#if MOVEIT_MASTER
          if (!moveit::core::isEmpty(sub_traj.scene_diff))
          {
#else
          if (!planning_scene::PlanningScene::isEmpty(sub_traj.scene_diff))
          {
#endif
            ROS_DEBUG_STREAM_NAMED("ExecuteTMPSolution",
                                   "apply effect of " << description);
            return context_->planning_scene_monitor_->newPlanningSceneMessage(
                sub_traj.scene_diff);
          }
          return true;
        };

#if MOVEIT_MASTER
    if (!moveit::core::isEmpty(sub_traj.scene_diff.robot_state) &&
#else
    if (!planning_scene::PlanningScene::isEmpty(
            sub_traj.scene_diff.robot_state) &&
#endif
        !moveit::core::robotStateMsgToRobotState(
            sub_traj.scene_diff.robot_state, state, true))
    {
      ROS_ERROR_STREAM_NAMED(
          "ExecuteTMPSolution",
          "invalid intermediate robot state in scene diff of SubTrajectory "
              << description);
      return false;
    }
  }

  return true;
}

} // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::ExecuteTMPSolutionCapability,
                            move_group::MoveGroupCapability)
