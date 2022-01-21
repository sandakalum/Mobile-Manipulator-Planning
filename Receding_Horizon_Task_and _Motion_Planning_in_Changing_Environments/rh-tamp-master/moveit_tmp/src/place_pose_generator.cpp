#include <moveit_tmp/place_pose_generator.h>

namespace moveit_tmp
{
PlacePoseGenerator::PlacePoseGenerator() : max_attempts_(50), rng_(time(0)) {}

bool PlacePoseGenerator::compute(
    const planning_scene::PlanningSceneConstPtr& scene,
    const std::string object_id, const std::string frame_id,
    const Eigen::Isometry3d& p_min, const Eigen::Isometry3d& p_max,
    geometry_msgs::PoseStamped& target_pose)
{

  planning_scene::PlanningScenePtr sandbox = scene->diff();

  std::uniform_real_distribution<double> distribution(0.0, 1.0);

  // NOTE: use attached object as workaround to find collisions. See moveit
  // #2097
  moveit_msgs::AttachedCollisionObject attached_object;
  sandbox->getAttachedCollisionObjectMsg(attached_object, object_id);
  moveit_msgs::CollisionObject object = attached_object.object;

  attached_object.object.id = PLACEHOLDER_;
  attached_object.object.operation = moveit_msgs::CollisionObject::ADD;
  attached_object.link_name = fixed_link_;

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  // req.group_name = "";
  req.contacts = true;
  req.max_contacts = 100;
  req.max_contacts_per_pair = 1; // 100;

  for (int attempts = 0; attempts < max_attempts_; ++attempts)
  {

    // std::cout << "Random: " << distribution(rng_) << std::endl;

    double x = distribution(rng_) *
                   (p_max.translation().x() - p_min.translation().x()) +
               p_min.translation().x();
    double y = distribution(rng_) *
                   (p_max.translation().y() - p_min.translation().y()) +
               p_min.translation().y();

    attached_object.object.primitive_poses[0].position.x = x;
    attached_object.object.primitive_poses[0].position.y = y;
    attached_object.object.primitive_poses[0].position.z =
        attached_object.object.primitives[0].dimensions[0] / 2 +
        0.001; // TODO attenzione funziona solo con i cilindri
    attached_object.object.primitive_poses[0].orientation.x = 0;
    attached_object.object.primitive_poses[0].orientation.y = 0;
    attached_object.object.primitive_poses[0].orientation.z = 0;
    attached_object.object.primitive_poses[0].orientation.w = 1;
    attached_object.object.header.frame_id = frame_id;

    sandbox->processAttachedCollisionObjectMsg(attached_object);

#if MOVEIT_MASTER

    sandbox->checkCollision(req, res, sandbox->getCurrentState());

#else
    sandbox->getCollisionWorld()->checkCollision(
        req, res, sandbox->getCollisionRobotUnpadded(),
        sandbox->getCurrentState(), sandbox->getAllowedCollisionMatrix());
#endif

    target_pose.header.frame_id = frame_id;
    target_pose.pose = attached_object.object.primitive_poses[0];

    // if (!res.collision)
    //  return true;

    bool collision = false;

    for (const auto& contact : res.contacts)
    {
      const collision_detection::Contact& c = contact.second.front();

      if (c.body_name_1 == PLACEHOLDER_ || c.body_name_2 == PLACEHOLDER_)
      {
        ROS_WARN_STREAM("Collision not considered between "
                        << c.body_name_1 << " " << c.body_type_1 << " and "
                        << c.body_name_2 << " " << c.body_type_2);
        // TODO improve
        if (c.body_type_1 != collision_detection::BodyTypes::ROBOT_LINK &&
            c.body_type_2 != collision_detection::BodyTypes::ROBOT_LINK)
        {
          ROS_WARN_STREAM("Collision between " << c.body_name_1 << " and "
                                               << c.body_name_2);
          collision = true;
          break;
        }
      }
    }


    if (!collision)
      return true;
  }

  return false;
}

} // namespace moveit_tmp
