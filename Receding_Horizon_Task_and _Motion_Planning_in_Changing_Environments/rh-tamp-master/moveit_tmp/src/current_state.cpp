#include <moveit_tmp/current_state.h>

namespace moveit_tmp
{
CurrentState::CurrentState(const moveit::core::RobotModelConstPtr& robot_model)
    : timeout_(-1.0)
{
  robot_model_ = robot_model;
  // scene_.reset();
}

bool CurrentState::compute(planning_scene::PlanningScenePtr& scene)
{
  scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);

  ros::NodeHandle h;
  ros::ServiceClient client =
      h.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

  ros::Duration timeout(timeout_);
  if (client.waitForExistence(timeout))
  {
    moveit_msgs::GetPlanningScene::Request req;
    moveit_msgs::GetPlanningScene::Response res;

    req.components.components =
        moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS |
        moveit_msgs::PlanningSceneComponents::ROBOT_STATE |
        moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
        moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES |
        moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
        moveit_msgs::PlanningSceneComponents::OCTOMAP |
        moveit_msgs::PlanningSceneComponents::TRANSFORMS |
        moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
        moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING |
        moveit_msgs::PlanningSceneComponents::OBJECT_COLORS;

    if (client.call(req, res))
    {
      scene->setPlanningSceneMsg(res.scene);
      return true;
    }
  }
  ROS_WARN("Failed to acquire current PlanningScene");
  return false;
}
} // namespace moveit_tmp
