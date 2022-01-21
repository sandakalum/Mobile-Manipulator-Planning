/*
 * Author: Nicola Castaman
 */

// ROS
#include <ros/ros.h>

#include <iostream>

#include <moveit_tmp/pick.h>
#include <moveit_tmp/place.h>
#include <moveit_tmp/task_planner.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_state/conversions.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <moveit_tmp/current_state.h>
#include <moveit_tmp/modify_planning_scene.h>
#include <moveit_tmp/move.h>
#include <moveit_tmp/place_pose_generator.h>
#include <moveit_tmp/planner.h>
//#include <moveit_tmp/move_cartesian.h>

#include <moveit_tmp/knowledge_base.h>

#include <moveit_tmp_msgs/ExecuteTMPSolutionAction.h>

#include <actionlib/client/simple_action_client.h>

// Rviz Visualization Tool
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <moveit_tmp/action_cache.h>
#include <moveit_tmp/pick.h>
#include <moveit_tmp/place.h>
#include <moveit_tmp/task_planner.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>

typedef std::pair<moveit_tmp::Action, std::vector<double>> ActionSolution;

static const std::string LOGNAME = "moveit_tmp_demo";

std::string problem_path = "/home/nicola/Desktop/PDDL/problem_.pddl";

int horizon_ = 4;

// Arm
const robot_model::JointModelGroup* arm_jmg_;

// Robot
robot_model::RobotModelPtr robot_model_;

// Planner
moveit_tmp::PlannerPtr planner_;

// Choose which arm to use
std::string hand_group_name_;
std::string arm_group_name_;
std::string eef_name_;
std::string hand_frame_;
std::string world_frame_;
Eigen::Isometry3d ik_frame_;

std::string hand_open_pose_;
std::string hand_close_pose_;
std::string arm_home_pose_;

// KB
moveit_tmp::KnowledgeBase kb;

moveit::planning_interface::PlanningSceneInterfacePtr psi_;

rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

void loadParameters()
{
  ROS_INFO_NAMED(LOGNAME, "Loading task parameters");
  ros::NodeHandle pnh("~");

  // Planning group properties
  size_t errors = 0;
  errors +=
      !rosparam_shortcuts::get(LOGNAME, pnh, "arm_group_name", arm_group_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_group_name",
                                     hand_group_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "eef_name", eef_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_frame", hand_frame_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "world_frame", world_frame_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "ik_frame", ik_frame_);

  // Predefined pose targets
  errors +=
      !rosparam_shortcuts::get(LOGNAME, pnh, "hand_open_pose", hand_open_pose_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_close_pose",
                                     hand_close_pose_);
  errors +=
      !rosparam_shortcuts::get(LOGNAME, pnh, "arm_home_pose", arm_home_pose_);

  // Horizon size
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "horizon", horizon_);

  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

void initialize()
{
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;
  robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(
      "robot_description");

  // Load the robot model
  robot_model_ = robot_model_loader->getModel();
  arm_jmg_ = robot_model_->getJointModelGroup(arm_group_name_);

  psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  visual_tools_ = std::make_shared<rviz_visual_tools::RvizVisualTools>(
      robot_model_->getModelFrame());
}

/*
void createTaskProblem(
    std::vector<std::string>& objects,
    std::map<std::string, std::vector<std::string>>& constraints,
    std::string holding_obj)
    */
void createTaskProblem()
{

  std::ofstream problem_file;
  problem_file.open((problem_path).c_str());

  problem_file << "(define (problem non_mono)(:domain manipulation)";

  problem_file << kb.getPDDL();

  problem_file
      << "(:goal (and (on object_0 surface_6) (on object_1 surface_7) "
         "(on object_2 surface_8) (on object_3 surface_0) (on object_4 "
         "surface_1) (on object_5 surface_2))))";

  problem_file.close();

  std::cout << kb.getPDDL();
}

bool convertDoublesToEigen(std::vector<double> values,
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

void spawnObject(const moveit_msgs::CollisionObject& object,
                 const rviz_visual_tools::colors& color)
{

  psi_->applyCollisionObject(object, visual_tools_->getColor(color));
}

void spawnObject(const moveit_msgs::CollisionObject& object)
{

  psi_->applyCollisionObject(object);
}

moveit_msgs::CollisionObject createTable(std::string name, double x, double y)
{
  std::string table_name = name, table_reference_frame = world_frame_;
  std::vector<double> table_dimensions = {1.3, 1.3, 0.05}; //{0.5, 0.4, 0.1};
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;

  moveit_msgs::CollisionObject object;
  object.id = table_name;
  object.header.stamp = ros::Time::now();
  object.header.frame_id = table_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  object.primitives[0].dimensions = table_dimensions;
  pose.position.z -= 0.5 * table_dimensions[2] +
                     0.005; // align surface with world TODO SISTEMARE
  object.primitive_poses.push_back(pose);

  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

moveit_msgs::CollisionObject createCylinder(std::string name, double x,
                                            double y, double z, double radius,
                                            double height, std::string frame)
{
  std::string object_name = name, object_reference_frame = frame;
  std::vector<double> object_dimensions = {height, radius}; //{0.20, 0.015};
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z += 0.5 * object_dimensions[0] + z;

  moveit_msgs::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = object_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = object_dimensions;
  object.primitive_poses.push_back(pose);

  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

moveit_msgs::CollisionObject
moveCollisionObject(const geometry_msgs::PoseStamped& pose,
                    const std::string& name)
{
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = pose.header.frame_id;
  collision_obj.id = name;
  collision_obj.operation = moveit_msgs::CollisionObject::MOVE;

  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = pose.pose;

  return collision_obj;
}

void generateGraspingPoints(const Eigen::Isometry3d& target_offset,
                            const int num_points,
                            std::vector<Eigen::Isometry3d>& grasping_points)
{
  double current_angle_ =
      -M_PI + (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) *
                  (2.0 * M_PI);

  for (int i = 0; i < num_points; i++)
  {
    // rotate object pose about z-axis
    Eigen::Isometry3d target_pose(target_offset);
    target_pose.rotate(
        Eigen::AngleAxisd(current_angle_, Eigen::Vector3d::UnitZ()));

    grasping_points.push_back(target_pose);

    current_angle_ += 2 * M_PI / num_points;

    if (current_angle_ > M_PI)
      current_angle_ -= (2 * M_PI);
  }
}

/* TODO RIPRISTINARE
bool execute(moveit_tmp_msgs::ExecuteTMPSolutionGoal& execute_goal)
{
  ROS_INFO("Executing solution trajectory");
  execute_.sendGoal(execute_goal);
  execute_.waitForResult();
  moveit_msgs::MoveItErrorCodes execute_result =
      execute_.getResult()->error_code;

  if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_ERROR_STREAM("Task execution failed and returned: "
                     << execute_.getState().toString());
    return false;
  }

  return true;
}
*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_tmp_baseline");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh("~");

  // TODO Spostare fuori da main
  // Execution
  actionlib::SimpleActionClient<moveit_tmp_msgs::ExecuteTMPSolutionAction>
      execute_("execute_tmp_solution", true);

  loadParameters();

  initialize();

  srand(time(NULL));

  // std::vector<double> ik_frame = {0, 0, 0.13, 1.571, 0.785, 1.571}; // 0.105
  // convertDoublesToEigen(ik_frame, ik_frame_);

  // Create scene
  spawnObject(createTable("table", 0.3, 0.3), rviz_visual_tools::colors::BROWN);

  std::vector<double> min = {0.3, -0.3, 0.03, 0.0, 0.0, 0.0};
  std::vector<double> max = {0.7, 0.7, 0.03, 0.0, 0.0, 0.0};
  Eigen::Isometry3d p_min, p_max;
  convertDoublesToEigen(min, p_min);
  convertDoublesToEigen(max, p_max);

  moveit_tmp::Surface sur = {"world", p_min, p_max};
  moveit_tmp::Object obj = {"table", moveit_tmp::ObjectType::SURFACE, true,
                            sur};
  kb.addObject(obj);
  // kb.addObject("table", moveit_tmp::ObjectType::SURFACE);

  int num_surfaces;
  rosparam_shortcuts::get(LOGNAME, nh, "num_surfaces", num_surfaces);
  for (int i = 0; i < num_surfaces; i++)
  {
    std::string surface_id = "surface_" + std::to_string(i);
    std::vector<double> object_pose;
    std::string frame_id;
    rosparam_shortcuts::get(LOGNAME, nh, surface_id + "/pose", object_pose);
    rosparam_shortcuts::get(LOGNAME, nh, surface_id + "/frame_id", frame_id);

    Eigen::Isometry3d p_min = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d p_max = Eigen::Isometry3d::Identity();

    moveit_tmp::Surface sur = {surface_id, p_min, p_max};
    moveit_tmp::Object obj = {"surface_" + std::to_string(i),
                              moveit_tmp::ObjectType::FIXED, true, sur};

    rviz_visual_tools::colors col;
    if (i < 3)
    {
      col = rviz_visual_tools::colors::RED;
    }
    else if (i >= 6 && i < 9)
    {
      col = rviz_visual_tools::colors::BLUE;
    }
    else
    {
      col = rviz_visual_tools::colors::GREY;
    }

    spawnObject(createCylinder(surface_id, object_pose[0], object_pose[1],
                               object_pose[2], 0.03, 0.001, frame_id),
                col);

    kb.addObject(obj);
    kb.getIsStackable()->add(surface_id, true);
  }

  int num_objects;
  rosparam_shortcuts::get(LOGNAME, nh, "num_objects", num_objects);
  for (int i = 0; i < num_objects; i++)
  {
    std::string object_name = "object_" + std::to_string(i);
    std::vector<double> object_pose;
    std::string frame_id;
    rosparam_shortcuts::get(LOGNAME, nh, object_name + "/pose", object_pose);
    rosparam_shortcuts::get(LOGNAME, nh, object_name + "/frame_id", frame_id);

    rviz_visual_tools::colors col;

    if (i < 3)
    {
      col = rviz_visual_tools::colors::BLUE;
    }
    else
    {
      col = rviz_visual_tools::colors::RED;
    }

    spawnObject(createCylinder(object_name, object_pose[0], object_pose[1],
                               object_pose[2], 0.015, 0.20, frame_id),
                col);
    moveit_tmp::Object obj = {object_name, moveit_tmp::ObjectType::MOVABLE,
                              false};
    kb.addObject(obj);
    kb.getOn()->add(object_name, frame_id);
  }

  std::ofstream myfile;
  myfile.open("/home/nicola/" + std::to_string(horizon_) + "_" +
                  std::to_string(num_objects) + ".csv",
              std::ios_base::app);

  // Inizializzo libreria
  moveit_tmp::TaskPlanner task_planner;
  task_planner.setDomainPath(
      "/home/nicola/Desktop/PDDL/pddl_domain_new_2.pddl");

  moveit_tmp::ModifyPlanningScene mps;
  moveit_tmp::PlacePoseGenerator ppg;
    ppg.setFixedLink("base_link");

  planner_ = std::make_shared<moveit_tmp::Planner>(robot_model_);

  moveit_tmp::Pick pick("pick", planner_);
  pick.setIKFrame(ik_frame_, hand_frame_);
  pick.setGroup(arm_group_name_);
  pick.setEndEffector(eef_name_);

  moveit_tmp::Place place("place", planner_);
  place.setIKFrame(ik_frame_, hand_frame_);
  place.setGroup(arm_group_name_);
  place.setEndEffector(eef_name_);

  planning_scene::PlanningScenePtr ps;

  moveit_tmp::CurrentState cs(robot_model_);
  cs.compute(ps);

  /*
  mps.allowCollisions("table", "panda_link0", true);
  mps.allowCollisions("table", "panda_link1", true);
  mps.apply(ps, ps);
  */

  moveit_msgs::PlanningScene psm;
  ps->getPlanningSceneMsg(psm);
  psi_->applyPlanningScene(psm);

  std::vector<moveit_msgs::ObjectColor> object_colors = psm.object_colors;

  // Reset gripper
  /*
  {
    planning_scene::PlanningScenePtr planning_scene;

    moveit_tmp::CurrentState current_state(robot_model_);
    current_state.compute(planning_scene);

    planning_scene::PlanningScenePtr scene;
    moveit_tmp::Move move(planner_);
    move.setGroup(hand_group_name_);
    robot_trajectory::RobotTrajectoryPtr traj;
    // move.compute(planning_scene, "open", scene, traj);
    std::vector<double> open = {0.02, 0.02};
    move.compute(planning_scene, open, scene, traj);

    ROS_INFO("Create execution message");

    moveit_tmp_msgs::ExecuteTMPSolutionGoal execute_goal;

    execute_goal.solution.emplace_back();
    moveit_tmp_msgs::SubTrajectory& t = execute_goal.solution.back();
    scene->getPlanningSceneDiffMsg(t.scene_diff);
    traj->getRobotTrajectoryMsg(t.trajectory);

    ROS_INFO("Executing solution trajectory");
    execute_.sendGoal(execute_goal);
    execute_.waitForResult();
    moveit_msgs::MoveItErrorCodes execute_result =
        execute_.getResult()->error_code;

    if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_ERROR_STREAM("Task execution failed and returned: "
                       << execute_.getState().toString());
    }
  }
  */

  clock_t begin = clock();

  moveit_tmp::ActionCache cache;

  bool f = false;
  bool status_changed = true;
  int j = 0;

  std::vector<moveit_tmp::Action> task_solution;

  double reasoning_time = 0.0;

  while (ros::ok())
  {

    moveit_tmp::CurrentState current_state(robot_model_);
    planning_scene::PlanningScenePtr current_scene;
    current_state.compute(current_scene);

    current_scene->getPlanningSceneMsg(psm);
    psm.object_colors = object_colors;

    psi_->applyPlanningScene(psm);

    // se lo stato è inaspettato resetto
    // TODO: invalidare solo stati non più validi
    if (status_changed)
    {
      ROS_INFO("State is changed");

      // Remove constraints
      kb.getObstruct()->reset();
      kb.getLeaveClean()->reset();

      cache.reset();
      f = false;
      status_changed = false;
    }

    // se no ho una soluzione valida rigenero il task plan
    if (!f)
    {
      std::vector<const moveit::core::AttachedBody*> attached_bodies;
      current_scene->getCurrentState().getAttachedBodies(attached_bodies);

      for (int i = 0; i < attached_bodies.size(); i++)
      {
        std::string in_hand = attached_bodies[i]->getName();

        std::cout << "check in-hand: " << in_hand << std::endl;

        kb.getInHand()->add(in_hand);
      }

      // createTaskProblem(objects, constraints, in_hand);
      createTaskProblem();

      if (!task_planner.plan(task_solution))
      {
        ROS_INFO("Task Unsolvable");
        break;
      }
      f = true;
      j = 0;
    }

    ROS_INFO_STREAM("j: " << j << " size: " << task_solution.size());

    if (j >= task_solution.size())
    {
      ROS_INFO("Task Complete");
      break;
    }

    ROS_INFO("Start Reasoning");

    clock_t reasoning_start = clock();

    moveit_tmp::KnowledgeBase sandbox_kb(kb);

    // TODO ottimizare, non ricalcolare sempre da capo
    std::vector<ActionSolution> action_solutions;
    current_state.compute(current_scene);

    for (int i = j; i < task_solution.size(); i++)
    {
      ROS_INFO_STREAM(task_solution[i]);

      planning_scene::PlanningScenePtr scene_out;

      std::string action_name = task_solution[i].name;

      if (action_name == "pickup" || action_name == "unstack")
      {
        bool found = false;
        std::string object_name = task_solution[i].parameters[0];
        std::string surface_name = task_solution[i].parameters[1];

        // search solution in cache
        std::vector<double> joint_values;
        moveit_tmp::ActionCache::ActionID id =
            std::make_tuple(action_name, object_name, surface_name);
        if (cache.retrieve(id, joint_values))
        {
          scene_out = current_scene->diff();

          robot_state::RobotState& robot_state =
              scene_out->getCurrentStateNonConst();

          robot_state.setJointGroupPositions(arm_group_name_, joint_values);
          robot_state.update();

          collision_detection::CollisionRequest req;
          collision_detection::CollisionResult res;
          scene_out->checkCollision(req, res);

          found = !res.collision;

          if (!found)
          {
            cache.remove(id);
          }
          else
          {
            action_solutions.emplace_back(task_solution[i], joint_values);
          }
        }

        // solution not founded in cache
        if (!found)
        {
          std::vector<std::string> collisions;
          std::vector<Eigen::Isometry3d> grasping_poses;

          generateGraspingPoints(Eigen::Isometry3d::Identity(), 40,
                                 grasping_poses);

          bool res = pick.reason(current_scene, object_name, grasping_poses,
                                 scene_out, collisions);

          // std::cout << "collisions: " << collisions.size() << std::endl;

          // std::vector<double> joint_values;
          scene_out->getCurrentState().copyJointGroupPositions(arm_group_name_,
                                                               joint_values);

          // cache[action_name + "_" + object_name] = joint_values;

          cache.insert(id, joint_values);

          action_solutions.emplace_back(task_solution[i], joint_values);

          if (!res)
          {

            for (int i = 0; i < collisions.size(); i++)
            {

              // avoid object collide with themself
              if (collisions[i] == object_name)
              {
                ROS_WARN("Object collide with themself.");
                continue;
              }

              // kb.addObject(collisions[i], moveit_tmp::MOVABLE);
              kb.getObstruct()->add(collisions[i], object_name);

              moveit_tmp::Object surface;
              if (sandbox_kb.getOn()->getUnderById(collisions[i], surface))
              {
                if (surface.type != moveit_tmp::ObjectType::SURFACE)
                  kb.getLeaveClean()->add(surface.id, object_name);
              }
            }
            f = false;
            ROS_WARN("Collisions found. Update contraints and replan.");
            break; // continue
          }
        }

        sandbox_kb.getOn()->removeObject(object_name);

        scene_out->decoupleParent();
        current_scene = scene_out;

        mps.attachObject(object_name, hand_frame_);
        mps.apply(current_scene, scene_out);

        scene_out->decoupleParent();
        current_scene = scene_out;
      }
      else if (action_name == "putdown" || action_name == "stack")
      {
        bool found = false;
        std::string object_name = task_solution[i].parameters[0];
        std::string surface_name = task_solution[i].parameters[1];

        // search solution in cache
        std::vector<double> joint_values;
        moveit_tmp::ActionCache::ActionID id =
            std::make_tuple(action_name, object_name, surface_name);
        if (cache.retrieve(id, joint_values))
        {
          scene_out = current_scene->diff();

          robot_state::RobotState& robot_state =
              scene_out->getCurrentStateNonConst();

          robot_state.setJointGroupPositions(arm_group_name_, joint_values);
          robot_state.update();

          collision_detection::CollisionRequest req;
          collision_detection::CollisionResult res;
          scene_out->checkCollision(req, res);

          found = !res.collision;

          if (!found)
          {
            cache.remove(id);
          }
          else
          {
            action_solutions.emplace_back(task_solution[i], joint_values);
          }
        }

        // solution not founded in cache
        if (!found)
        {

          moveit_tmp::Object surface;
          if (!kb.getObjectById(surface_name, surface))
          {
            ROS_ERROR_STREAM("Surface " << surface_name << " not found.");
            return 1; // TODO
          }

          geometry_msgs::PoseStamped target;
          Eigen::Isometry3d pose;

          if (!ppg.compute(current_scene, object_name, surface.surface.frame_id,
                           surface.surface.p_min, surface.surface.p_max,
                           target))
          {
            // kb.getIsStackable()->add(surface.id, false);
            f = false;
            ROS_WARN("Place surface not empty. Update contraints and replan.");
            break;
          }

          tf::poseMsgToEigen(target.pose, pose);

          std::vector<Eigen::Isometry3d> poses;
          generateGraspingPoints(pose, 40, poses);

          std::vector<std::string> collisions;
          bool res = place.reason(current_scene, target.header.frame_id, poses,
                                  scene_out, //"panda_link0"
                                  collisions);

          std::vector<double> joint_values;
          scene_out->getCurrentState().copyJointGroupPositions(arm_group_name_,
                                                               joint_values);

          cache.insert(id, joint_values);

          action_solutions.emplace_back(task_solution[i], joint_values);

          if (!res)
          {

            for (int i = 0; i < collisions.size(); i++)
            {

              // avoid object collide with themself
              if (collisions[i] == object_name)
              {
                ROS_WARN("Object collide with themself.");
                continue;
              }

              // kb.addObject(collisions[i], moveit_tmp::MOVABLE);
              kb.getObstruct()->add(collisions[i], surface_name);

              moveit_tmp::Object surface;
              if (sandbox_kb.getOn()->getUnderById(collisions[i], surface))
              {
                if (surface.type != moveit_tmp::ObjectType::SURFACE)
                  kb.getLeaveClean()->add(surface.id, object_name);
              }
            }
            f = false;
            ROS_WARN("Collisions found. Update contraints and replan.");
            break; // continue
          }
        }

        sandbox_kb.getOn()->add(object_name, surface_name);

        scene_out->decoupleParent();
        current_scene = scene_out;

        mps.detachObject(object_name, hand_frame_);
        mps.apply(current_scene, scene_out);

        scene_out->decoupleParent();
        current_scene = scene_out;
      }
      else
      {
        ROS_ERROR("Undefined action name.");
        return -1;
      }

      std::cout << kb.getObstruct()->getPDDL() << std::endl;
    }

    clock_t reasoning_end = clock();
    double reasoning_secs =
        double(reasoning_end - reasoning_start) / CLOCKS_PER_SEC;
    reasoning_time += reasoning_secs;
    ROS_INFO_STREAM("Reasoning Time: " << reasoning_secs);

    ROS_INFO("End Geometry Reasoning");

    // se ho una soluzione valida ...
    if (f)
    {
      ROS_INFO("Execution");

      planning_scene::PlanningScenePtr scene;
      current_state.compute(scene);

      std::vector<moveit_tmp::ActionPipeline> execute;

      for (int i = 0; i < action_solutions.size(); i++)
      {
        ROS_INFO_STREAM("Planning: "
                        << action_solutions[i].first.name << " "
                        << action_solutions[i].first.parameters[0]);

        std::vector<double> joint_values = action_solutions[i].second;

        robot_trajectory::RobotTrajectoryPtr traj;

        execute.emplace_back();
        moveit_tmp::ActionPipeline& p = execute.back();

        bool s = false;
        int count = 0;
        while (!s && count < 3)
        {

          if (action_solutions[i].first.name == "pickup" ||
              action_solutions[i].first.name == "unstack")
          {
            s = pick.plan(scene, action_solutions[i].first.parameters[0],
                          joint_values, p);

            // if (s)
            //{
            // kb.getOn()->removeObject(action_solutions[0].first.parameters[0]);
            //}
          }
          else if (action_solutions[i].first.name == "putdown" ||
                   action_solutions[i].first.name == "stack")
          {
            s = place.plan(scene, action_solutions[i].first.parameters[0],
                           joint_values, p);
            // if (s)
            //{
            //  kb.getOn()->add(action_solutions[0].first.parameters[0],
            //                  action_solutions[0].first.parameters[1]);
            //  kb.getInHand()->empty();
            //}
          }
          else
          {
            ROS_ERROR("Undefined action name.");
            return -1;
          }
          count++;
        }

        ROS_INFO("Done");

        if (!s || (double(clock() - begin) / CLOCKS_PER_SEC) > 600)
        {
          ros::shutdown();
          return 0;
        }

        scene = p.back().first->diff();

        moveit_tmp::ActionCache::ActionID id =
            std::make_tuple(action_solutions[i].first.name,
                            action_solutions[i].first.parameters[0],
                            action_solutions[i].first.parameters[1]);
        cache.remove(id);
      }

      ROS_INFO("Create execution message");
      // Compongo messaggio
      for (int i = 0; i < execute.size() && !status_changed; i++)
      {
        moveit_tmp::ActionPipeline p = execute[i];
        moveit_tmp_msgs::ExecuteTMPSolutionGoal execute_goal;
        for (int j = 0; j < p.size(); j++)
        {
          execute_goal.solution.emplace_back();
          moveit_tmp_msgs::SubTrajectory& t = execute_goal.solution.back();
          p[j].first->getPlanningSceneDiffMsg(t.scene_diff);
          if (p[j].second)
            p[j].second->getRobotTrajectoryMsg(t.trajectory);
        }

        ROS_INFO("Executing solution trajectory");
        execute_.sendGoal(execute_goal);
        execute_.waitForResult();
        moveit_msgs::MoveItErrorCodes execute_result =
            execute_.getResult()->error_code;

        if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
          ROS_ERROR_STREAM("Task execution failed and returned: "
                           << execute_.getState().toString());

          status_changed = true;
          continue;
        }
        ROS_INFO("DONE");

        // UPDATE STATUS
        // if (action_solutions[0].first.name == "putdown")
        //{
        //  kb.getObstruct()->removeObject(action_solutions[0].first.parameters[0]);
        //}

        if (action_solutions[i].first.name == "pickup" ||
            action_solutions[i].first.name == "unstack")
        {
          kb.getOn()->removeObject(action_solutions[i].first.parameters[0]);
          kb.getObstruct()->removeObject(
              action_solutions[i].first.parameters[0]);
          kb.getLeaveClean()->removeObject(
              action_solutions[i].first.parameters[0]);
        }
        else if (action_solutions[i].first.name == "putdown" ||
                 action_solutions[i].first.name == "stack")
        {
          kb.getOn()->add(action_solutions[i].first.parameters[0],
                          action_solutions[i].first.parameters[1]);
          kb.getInHand()->empty();
        }

        ROS_INFO("UPDATED KB");

        j++;
        // } //tentativo movimento dopo ogni ciclo

        // Genero movimento casuale
        double r = (double)rand() / RAND_MAX;
        ROS_INFO_STREAM("RAND: " << r);
        if (r < 0.1)
        {
          ROS_WARN("Moved Object");

          planning_scene::PlanningScenePtr scene;
          current_state.compute(scene);

          int obj = rand() % num_objects;
          // int sur = rand() % num_surfaces;

          std::string object_name = "object_" + std::to_string(obj);
          // std::string surface_name = "surface_" + std::to_string(sur);

          geometry_msgs::PoseStamped target;
          target.header.frame_id = "world";
          target.pose.position.x =
              ((double)rand() / (double)RAND_MAX) * 0.3 + 0.3;
          target.pose.position.y =
              ((double)rand() / (double)RAND_MAX) * 0.3 + 0.3;
          target.pose.position.z = 0.001 + 0.5 * 0.2;
          target.pose.orientation.w = 1;

          spawnObject(moveCollisionObject(target, object_name));

          kb.getOn()->add(object_name, "table");

          status_changed = true;
        }
      }

    } // tentativo movimento dopo ogni movimento
  }

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  ROS_INFO_STREAM("Elapsed Time: " << elapsed_secs
                                   << " Reasoning Time: " << reasoning_time);
  // ROS_INFO_STREAM("Removed Objects: " << tot_removed);
  myfile << elapsed_secs << ";" /*<< tot_removed * 2 + 1 << ";"*/ << f << "\n";
  myfile.close();

  //
  // begin = clock();

  // end = clock();
  // elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  // ROS_INFO_STREAM("Elapsed Time: " << elapsed_secs);

  /* Wait for user input */
  // visual_tools.prompt(
  // b    "Press 'next' in the RvizVisualToolsGui window to finish the demo");

  ros::shutdown();
  return 0;
}
