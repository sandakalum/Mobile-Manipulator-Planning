#include <moveit_tmp/compute_ik.h>

namespace moveit_tmp
{
ComputeIK::ComputeIK() {}

bool ComputeIK::convertDoublesToEigen(std::vector<double> values,
                                      Eigen::Isometry3d& transform)
{
  if (values.size() == 6)
  {
    // This version is correct RPY
    Eigen::AngleAxisd roll_angle(values[3], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(values[4], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(values[5], Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> quaternion = roll_angle * pitch_angle * yaw_angle;

    transform =
        Eigen::Translation3d(values[0], values[1], values[2]) * quaternion;

    return true;
  }
  else if (values.size() == 7)
  {
    // Quaternion
    transform = Eigen::Translation3d(values[0], values[1], values[2]) *
                Eigen::Quaterniond(values[3], values[4], values[5], values[6]);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Invalid number of doubles provided for transform, size="
                     << values.size());
    return false;
  }
}

bool ComputeIK::isTargetPoseColliding(
    const planning_scene::PlanningScenePtr& scene, Eigen::Isometry3d pose,
    const robot_model::LinkModel* link,
    collision_detection::CollisionResult* collision_result)
{
  robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();

  // consider all rigidly connected parent links as well
  const robot_model::LinkModel* parent =
      robot_model::RobotModel::getRigidlyConnectedParentLinkModel(link);

  if (parent != link) // transform pose into pose suitable to place parent
    pose = pose * robot_state.getGlobalLinkTransform(link).inverse() *
           robot_state.getGlobalLinkTransform(parent);

  // place link at given pose
  robot_state.updateStateWithLinkAt(parent, pose);

  robot_state.updateCollisionBodyTransforms();

  // disable collision checking for parent links (except links fixed to root)
  auto& acm = scene->getAllowedCollisionMatrixNonConst();
  std::vector<const std::string*> pending_links; // parent link names that might
                                                 // be rigidly connected to root

  while (parent)
  {
    pending_links.push_back(&parent->getName());
    link = parent;
    const robot_model::JointModel* joint = link->getParentJointModel();
    parent = joint->getParentLinkModel();

    if (joint->getType() != robot_model::JointModel::FIXED)
    {
      for (const std::string* name : pending_links)
        acm.setDefaultEntry(*name, true);
      pending_links.clear();
    }
  }

  // check collision with the world using the padded version
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult result;
  req.contacts = (collision_result != nullptr);
  req.max_contacts = 100;
  req.max_contacts_per_pair = 1;
  collision_detection::CollisionResult& res =
      collision_result ? *collision_result : result;
  scene->checkCollision(req, res, robot_state, acm);
  return res.collision;
}

bool ComputeIK::isTargetPoseColliding(
    const planning_scene::PlanningSceneConstPtr& scene,
    collision_detection::CollisionResult* collision_result)
{
  robot_state::RobotState robot_state = scene->getCurrentState();

  // check collision with the world using the padded version
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult result;
  req.contacts = (collision_result != nullptr);
  req.max_contacts = 100; // added
  collision_detection::CollisionResult& res =
      collision_result ? *collision_result : result;
  scene->checkCollision(req, res, robot_state);
  return res.collision;
}

std::string ComputeIK::listCollisionPairs(
    const collision_detection::CollisionResult::ContactMap& contacts,
    const std::string& separator)
{
  std::string result;
  for (const auto& contact : contacts)
  {
    if (!result.empty())
      result.append(separator);
    result.append(contact.first.first)
        .append(" - ")
        .append(contact.first.second);
  }
  return result;
}

void ComputeIK::listCollisionObjects(
    const collision_detection::CollisionResult::ContactMap& contacts,
    std::vector<std::string>& objects)
{

  std::string object;

  for (const auto& contact : contacts)
  {
    const collision_detection::Contact c = contact.second.front();

    if ((c.body_type_1 == collision_detection::BodyTypes::WORLD_OBJECT) ==
        (c.body_type_2 == collision_detection::BodyTypes::WORLD_OBJECT))
    {
      ROS_WARN_STREAM("Not a collision between object and robot ("
                      << c.body_name_1 << " " << c.body_name_2 << ")");
      // std::cout << c.body_type_1 << " " << c.body_type_2 << std::endl;
      continue;
    }

    if (c.body_type_1 == collision_detection::BodyTypes::WORLD_OBJECT)
    {
      object = c.body_name_1;
    }
    else if (c.body_type_2 == collision_detection::BodyTypes::WORLD_OBJECT)
    {
      object = c.body_name_2;
    }

    if (std::find(objects.begin(), objects.end(), object) == objects.end())
    {
      objects.push_back(object);
    }
  }
}

void ComputeIK::setIKFrame(const Eigen::Isometry3d& pose,
                           const std::string& link)
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = link;
  tf::poseEigenToMsg(pose, pose_msg.pose);
  setIKFrame(pose_msg);
}

void ComputeIK::setTargetPose(const Eigen::Isometry3d& pose,
                              const std::string& frame)
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = frame;
  tf::poseEigenToMsg(pose, pose_msg.pose);
  setTargetPose(pose_msg);
}

bool ComputeIK::compute(const planning_scene::PlanningSceneConstPtr& from,
                        planning_scene::PlanningScenePtr& to,
                        std::vector<std::string>& collision_objects)
{

  planning_scene::PlanningScenePtr sandbox_scene = from->diff();

  // sandbox_scene = scene_->diff();

  const auto& robot_model = sandbox_scene->getRobotModel();

  // IK
  robot_state::RobotState& sandbox_state =
      sandbox_scene->getCurrentStateNonConst();

  const robot_state::JointModelGroup* joint_model_group =
      robot_model->getJointModelGroup(group_);

  // TODO: rimuovere duplicato?
  geometry_msgs::PoseStamped target_pose_msg = target_pose_;
  if (target_pose_msg.header.frame_id
          .empty()) // if not provided, assume planning frame
    target_pose_msg.header.frame_id = sandbox_scene->getPlanningFrame();

  Eigen::Isometry3d target_pose;
  tf::poseMsgToEigen(target_pose_msg.pose, target_pose);
  if (target_pose_msg.header.frame_id != sandbox_scene->getPlanningFrame())
  {
    if (!sandbox_scene->knowsFrameTransform(target_pose_msg.header.frame_id))
    {
      ROS_WARN_STREAM_NAMED("ComputeIK",
                            "Unknown reference frame for target pose: "
                                << target_pose_msg.header.frame_id);
      return false;
    }
    // transform target_pose w.r.t. planning frame
    target_pose =
        from->getFrameTransform(target_pose_msg.header.frame_id) * target_pose;
  }

  // determine IK link from ik_frame
  const robot_model::LinkModel* link = nullptr;

  geometry_msgs::PoseStamped ik_pose_msg = ik_frame_;
  Eigen::Isometry3d ik_pose;
  tf::poseMsgToEigen(ik_pose_msg.pose, ik_pose);

  if (robot_model->hasLinkModel(ik_pose_msg.header.frame_id))
  {
    link = robot_model->getLinkModel(ik_pose_msg.header.frame_id);
  }
  else
  {
    ROS_WARN("No link found (TODO Sistemare messaggio)");
  }

  target_pose = target_pose * ik_pose.inverse();

  bool found_ik =
      sandbox_state.setFromIK(joint_model_group, target_pose, timeout_);

  const std::vector<std::string>& joint_names =
      joint_model_group->getVariableNames();
  std::vector<double> joint_values;

  if (found_ik)
  {
    sandbox_state.copyJointGroupPositions(joint_model_group, joint_values);

    // for (std::size_t i = 0; i < joint_names.size(); ++i)
    //{
    //  ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    //}

    to = from->diff();
    // set scene's robot state
    robot_state::RobotState& robot_state = to->getCurrentStateNonConst();
    robot_state.setJointGroupPositions(joint_model_group, joint_values);
    robot_state.update();

    collision_detection::CollisionResult collisions;
    // if (isTargetPoseColliding(sandbox_scene, target_pose, link, &collisions))
    if (isTargetPoseColliding(sandbox_scene, &collisions))
    {
      std::cout << listCollisionPairs(collisions.contacts, ", ") << std::endl;
      listCollisionObjects(collisions.contacts, collision_objects);
      return false;
    }

    return true;
  }
  return false;
}
} // namespace moveit_tmp
