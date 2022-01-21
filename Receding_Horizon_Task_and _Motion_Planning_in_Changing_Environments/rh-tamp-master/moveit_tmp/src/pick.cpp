

#include <moveit_tmp/pick.h>

namespace moveit_tmp
{

Pick::Pick(const std::string& name, const PlannerPtr& planner)
    : ActionPlanner(name), planner_(planner), move_(planner_, 10.0)
{
}

bool Pick::reason(const planning_scene::PlanningSceneConstPtr& planning_scene,
                  const std::string& target_name,
                  const std::vector<Eigen::Isometry3d>& target_poses,
                  planning_scene::PlanningScenePtr& goal,
                  std::vector<std::string>& collision_objects)
{
  collision_objects.clear();

  geometry_msgs::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = target_name;

  planning_scene::PlanningScenePtr tmp;
  std::vector<double> tmp_joint_val;

  ik_.setGroup(group_);
  ik_.setEndEffector(eef_);
  ik_.setIKFrame(ik_frame_);

  for (const Eigen::Isometry3d& target_pose : target_poses)
  {
    tf::poseEigenToMsg(target_pose, target_pose_msg.pose);

    ik_.setTargetPose(target_pose_msg);

    std::vector<std::string> collisions;

    if (ik_.compute(planning_scene, goal, collisions))
    {
      return true;
    }
    else
    {
      if (!collisions.empty() && (collision_objects.empty() ||
                                  collisions.size() < collision_objects.size()))
      {
        collision_objects = collisions;

        goal->getCurrentState().copyJointGroupPositions(group_, tmp_joint_val);
      }
    }
  }

  goal = planning_scene->diff();
  robot_state::RobotState& robot_state = goal->getCurrentStateNonConst();
  robot_state.setJointGroupPositions(group_, tmp_joint_val);
  robot_state.update();

  return false;
}

bool Pick::plan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                const std::string target_name,
                const std::vector<double>& joint_values,
                ActionPipeline& pipeline)
{

  // Config move object
  move_.setIKFrame(ik_frame_);

  Step step_1;

  move_.setGroup(group_);
  if (!move_.compute(planning_scene, joint_values, step_1.first, step_1.second))
  {
    ROS_WARN("Motion plan not found");
    return false;
  }
  pipeline.push_back(step_1);

  // Fisso oggetto
  Step step_2;
  // step_2.second = robot_trajectory::RobotTrajectoryPtr();
  mps_.attachObject(target_name, ik_frame_.header.frame_id);
  mps_.apply(step_1.first, step_2.first);
  pipeline.push_back(step_2);

  // Consento collisione con tavolo
  /*
  Step step_3;
  step_3.second = robot_trajectory::RobotTrajectoryPtr();
  mps.allowCollisions(object, "table");
  mps.apply(step_2.first, step_3.first);
  pipeline.push_back(step_3);
  */

  return true;
}
} // namespace moveit_tmp
