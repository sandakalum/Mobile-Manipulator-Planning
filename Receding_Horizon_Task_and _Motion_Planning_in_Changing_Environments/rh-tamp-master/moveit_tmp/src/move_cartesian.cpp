#include <moveit_tmp/move_cartesian.h>
namespace moveit_tmp
{
MoveCartesian::MoveCartesian(const moveit::core::RobotModelConstPtr& model)
    : max_acceleration_scaling_factor_(1.0), max_velocity_scaling_factor_(1.0),
      jump_threshold_(1.5), min_fraction_(1.0), step_size_(0.01)
{
}

bool MoveCartesian::compute(const planning_scene::PlanningSceneConstPtr& start,
                            const geometry_msgs::Vector3Stamped& direction,
                            const double min_distance,
                            const double max_distance,
                            planning_scene::PlanningScenePtr& goal,
                            robot_trajectory::RobotTrajectoryPtr& trajectory)
{
  goal = start->diff();
  const robot_model::RobotModelConstPtr& robot_model = goal->getRobotModel();
  assert(robot_model);

  const moveit::core::JointModelGroup* jmg =
      robot_model->getJointModelGroup(group_);
  if (!jmg)
  {
    ROS_WARN_STREAM_NAMED("MoveRelative",
                          "Invalid joint model group: " << group_);
    return false;
  }

  bool success = false;

  geometry_msgs::PoseStamped ik_pose_msg;
  const moveit::core::LinkModel* link;
  if (ik_frame_.header.frame_id == "")
  { // frame undefined
    //  determine IK link from group
    if (!(link = jmg->getOnlyOneEndEffectorTip()))
    {
      ROS_WARN_STREAM_NAMED("MoveRelative", "Failed to derive IK target link");
      return false;
    }
    ik_pose_msg.header.frame_id = link->getName();
    ik_pose_msg.pose.orientation.w = 1.0;
  }
  else
  {
    ik_pose_msg = ik_frame_;
    if (!(link = robot_model->getLinkModel(ik_pose_msg.header.frame_id)))
    {
      ROS_WARN_STREAM_NAMED("MoveRelative",
                            "Unknown link: " << ik_pose_msg.header.frame_id);
      return false;
    }
  }

  bool use_rotation_distance = false; // measure achieved distance as rotation?
  Eigen::Vector3d linear;             // linear translation
  Eigen::Vector3d angular;            // angular rotation
  double linear_norm = 0.0, angular_norm = 0.0;

  Eigen::Isometry3d target_eigen;
  Eigen::Isometry3d link_pose = goal->getCurrentState().getGlobalLinkTransform(
      link); // take a copy here, pose will change on success

  const geometry_msgs::Vector3Stamped& target = direction;
  const Eigen::Isometry3d& frame_pose =
      goal->getFrameTransform(target.header.frame_id);
  tf::vectorMsgToEigen(target.vector, linear);

  // use max distance?
  if (max_distance > 0.0)
  {
    linear.normalize();
    linear *= max_distance;
  }
  linear_norm = linear.norm();

  // compute absolute transform for link
  linear = frame_pose.linear() * linear;
  target_eigen = link_pose;
  target_eigen.translation() += linear;

  // transform target pose such that ik frame will reach there if link does
  Eigen::Isometry3d ik_pose;
  tf::poseMsgToEigen(ik_pose_msg.pose, ik_pose);
  target_eigen = target_eigen * ik_pose.inverse();

  success = plan(start, *link, target_eigen, jmg, trajectory);

  // min_distance reached?
  if (min_distance > 0.0)
  {
    double distance = 0.0;
    if (trajectory && trajectory->getWayPointCount() > 0)
    {
      robot_state::RobotStatePtr& reached_state =
          trajectory->getLastWayPointPtr();
      reached_state->updateLinkTransforms();
      const Eigen::Isometry3d& reached_pose =
          reached_state->getGlobalLinkTransform(link);
      if (use_rotation_distance)
      {
        Eigen::AngleAxisd rotation(reached_pose.linear() *
                                   link_pose.linear().transpose());
        distance = rotation.angle();
      }
      else
        distance =
            (reached_pose.translation() - link_pose.translation()).norm();
    }
    success = distance >= min_distance;
    if (!success)
    {
      ROS_WARN_STREAM("min_distance not reached (" << distance << " < "
                                                   << min_distance << ")");
    }
  }
  else if (min_distance == 0.0)
  { // if min_distance is zero, we succeed in any case
    success = true;
  }

  // store result
  if (trajectory)
  {
    goal->setCurrentState(trajectory->getLastWayPoint());

    /*To-Do reenable
   solution.setTrajectory(robot_trajectory);

    // set cost
    double cost = 0;
    for (const double& distance : robot_trajectory->getWayPointDurations()) {
      cost += distance;
    }
    solution.setCost(cost);


    if (!success)
      solution.markAsFailure();
    return true;
    */

    if (!success)
      return false;
    return true;
  }

  return false;
}

bool MoveCartesian::plan(
    const planning_scene::PlanningSceneConstPtr& from,
    const planning_scene::PlanningSceneConstPtr& to,
    const moveit::core::JointModelGroup* jmg,
    /*double timeout,*/ robot_trajectory::RobotTrajectoryPtr& result)
{
  const moveit::core::LinkModel* link = jmg->getOnlyOneEndEffectorTip();
  if (!link)
  {
    ROS_WARN_STREAM("no unique tip for joint model group: " << jmg->getName());
    return false;
  }

  // reach pose of forward kinematics
  return plan(from, *link, to->getCurrentState().getGlobalLinkTransform(link),
              jmg, result);
}

bool MoveCartesian::plan(const planning_scene::PlanningSceneConstPtr& from,
                         const moveit::core::LinkModel& link,
                         const Eigen::Isometry3d& target,
                         const moveit::core::JointModelGroup* jmg,
                         /*double timeout,*/
                         robot_trajectory::RobotTrajectoryPtr& result)
{

  planning_scene::PlanningScenePtr sandbox_scene = from->diff();

  kinematic_constraints::KinematicConstraintSet kcs(
      sandbox_scene->getRobotModel());
  // kcs.add(path_constraints, sandbox_scene->getTransforms());

  auto isValid = [&sandbox_scene,
                  &kcs](moveit::core::RobotState* state,
                        const moveit::core::JointModelGroup* jmg,
                        const double* joint_positions) {
    state->setJointGroupPositions(jmg, joint_positions);
    state->update();
    return !sandbox_scene->isStateColliding(
               const_cast<const robot_state::RobotState&>(*state),
               jmg->getName()) &&
           kcs.decide(*state).satisfied;
  };

  std::vector<moveit::core::RobotStatePtr> trajectory;

  double achieved_fraction =
      moveit::core::CartesianInterpolator::computeCartesianPath(
          &(sandbox_scene->getCurrentStateNonConst()), jmg, trajectory, &link,
          target, true, moveit::core::MaxEEFStep(step_size_),
          moveit::core::JumpThreshold(jump_threshold_), isValid);

  if (!trajectory.empty())
  {
    result.reset(new robot_trajectory::RobotTrajectory(
        sandbox_scene->getRobotModel(), jmg));
    for (const auto& waypoint : trajectory)
      result->addSuffixWayPoint(waypoint, 0.0);

    trajectory_processing::IterativeParabolicTimeParameterization timing;
    timing.computeTimeStamps(*result, max_velocity_scaling_factor_,
                             max_acceleration_scaling_factor_);
  }

  return achieved_fraction >= min_fraction_;
}
} // namespace moveit_tmp
