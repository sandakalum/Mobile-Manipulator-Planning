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

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_state/conversions.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <moveit_tmp/current_state.h>
#include <moveit_tmp/modify_planning_scene.h>
#include <moveit_tmp/move.h>
#include <moveit_tmp/planner.h>
//#include <moveit_tmp/move_cartesian.h>

#include <moveit_tmp_msgs/ExecuteTMPSolutionAction.h>

#include <actionlib/client/simple_action_client.h>

// Rviz Visualization Tool
#include <rviz_visual_tools/rviz_visual_tools.h>

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

void createTaskProblem(
    std::vector<std::string>& objects,
    std::map<std::string, std::vector<std::string>>& constraints,
    std::string holding_obj)
{

  std::ofstream problem_file;
  problem_file.open((problem_path).c_str());

  problem_file << "(define (problem pb1)(:domain can)(:objects ";

  for (int i = 0; i < objects.size(); i++)
  {
    problem_file << objects[i] + " ";
  }
  problem_file << "- objs )(:init ";
  for (auto const& c : constraints)
  {
    // TODO generalizzare
    if (c.first == "object" || c.first == "table")
      continue;

    for (auto const& obj : c.second)
    {

      problem_file << "(obstruct " + c.first + " " + obj + ")";

      std::cout << "CONSTRAINTS "
                << "(obstruct " + c.first + " " + obj + ")" << std::endl;
    }
  }

  if (holding_obj != "")
    problem_file << "(not (arm-empty))(holding " + holding_obj + ")";
  else
    problem_file << "(arm-empty)";

  problem_file << ") (:goal (and (holding object))))";

  problem_file.close();
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

void spawnObject(
    const moveit_msgs::CollisionObject& object,
    const rviz_visual_tools::colors& color = rviz_visual_tools::colors::GREEN)
{

  psi_->applyCollisionObject(object, visual_tools_->getColor(color));
}

moveit_msgs::CollisionObject createTable(std::string name, double x,
                                         double y)
{
  std::string table_name = name, table_reference_frame = world_frame_;
  std::vector<double> table_dimensions = {1.3, 0.75, 0.05}; //{0.5, 0.4, 0.1};
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
  pose.position.z -=
      0.5 * table_dimensions[2] + 0.005; // align surface with world TODO SISTEMARE
  object.primitive_poses.push_back(pose);

  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

moveit_msgs::CollisionObject createCylinder(std::string name, double x,
                                            double y)
{
  std::string object_name = name, object_reference_frame = world_frame_;
  std::vector<double> object_dimensions = {0.20, 0.015};
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;

  moveit_msgs::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = object_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = object_dimensions;
  pose.position.z += 0.5 * object_dimensions[0] + 0.001;
  object.primitive_poses.push_back(pose);

  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

moveit_msgs::CollisionObject
moveCollisionObject(const geometry_msgs::Pose& pose, const std::string& name)
{
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = "world";
  collision_obj.id = name;
  collision_obj.operation = moveit_msgs::CollisionObject::MOVE;

  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = pose;

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
  spawnObject(createTable(), rviz_visual_tools::colors::BROWN);

  int num_objects;
  rosparam_shortcuts::get(LOGNAME, nh, "num_objects", num_objects);

  std::vector<double> target_object_pose;
  rosparam_shortcuts::get(LOGNAME, nh, "object_0", target_object_pose);

  spawnObject(
      createCylinder("object", target_object_pose[0], target_object_pose[1]),
      rviz_visual_tools::colors::RED);

  for (int i = 0; i < num_objects - 1; i++)
  {
    std::vector<double> object_pose;
    rosparam_shortcuts::get(LOGNAME, nh, "object_" + std::to_string(i + 1),
                            object_pose);

    spawnObject(createCylinder("object_" + std::to_string(i + 1),
                               object_pose[0], object_pose[1]));
  }

  std::ofstream myfile;
  myfile.open("/home/nicola/" + std::to_string(horizon_) + "_" +
                  std::to_string(num_objects) + ".csv",
              std::ios_base::app);

  // Inizializzo libreria
  moveit_tmp::TaskPlanner task_planner;

  moveit_tmp::ModifyPlanningScene mps;

  planner_ = std::make_shared<moveit_tmp::Planner>(robot_model_);

  moveit_tmp::Pick pick("pick", planner_);
  pick.setIKFrame(ik_frame_, hand_frame_);
  pick.setGroup(arm_group_name_);
  pick.setEndEffector(eef_name_);

  moveit_tmp::Place place("place", planner_);
  place.setIKFrame(ik_frame_, hand_frame_);
  place.setGroup(arm_group_name_);
  place.setEndEffector(eef_name_);

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

  std::vector<std::string> objects = {"object"};
  // std::vector<std::pair<std::string, std::string>> constraints;
  std::map<std::string, std::vector<std::string>> constraints;

  clock_t begin = clock();

  int tot_removed = 0;

  std::map<std::string, std::vector<double>> cache;

  bool f = false;
  bool status_changed = true;
  int j = 0;
  std::string target_object = "object";
  std::vector<moveit_tmp::Action> task_solution;

  double reasoning_time = 0.0;

  while (ros::ok())
  {

    moveit_tmp::CurrentState current_state(robot_model_);
    planning_scene::PlanningScenePtr current_scene;
    current_state.compute(current_scene);

    // se lo stato è inaspettato resetto
    //TODO eliminare solo le info dell'oggetto modificato
    if (status_changed)
    {
      ROS_INFO("State is changed");
      objects = {target_object};
      constraints.clear();
      cache.clear();
      f = false;
      status_changed = false;
    }

    // se no ho una soluzione valida rigenero il task plan
    if (!f)
    {
      std::vector<const moveit::core::AttachedBody*> attached_bodies;
      current_scene->getCurrentState().getAttachedBodies(attached_bodies);

      std::string in_hand = "";
      for (int i = 0; i < attached_bodies.size(); i++)
      {
        in_hand = attached_bodies[i]->getName();
        objects.push_back(in_hand);
      }

      createTaskProblem(objects, constraints, in_hand);
      if(!task_planner.plan(task_solution)){
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

    int removed = tot_removed; // TODO rimuovere?

    // TODO ottimizare, non ricalcolare sempre da capo
    std::vector<ActionSolution> action_solutions;
    current_state.compute(current_scene);

    for (int i = j; i < task_solution.size() && i < j + horizon_; i++)
    {
      ROS_INFO_STREAM(task_solution[i]);

      planning_scene::PlanningScenePtr scene_out;

      std::string action_name = task_solution[i].name;

      if (action_name == "pickup")
      {
        bool found = false;
        std::string object_name = task_solution[i].parameters[0];

        // search solution in cache
        std::map<std::string, std::vector<double>>::iterator it =
            cache.find(action_name + "_" + object_name);

        // solution founded in cache
        if (it != cache.end())
        {
          scene_out = current_scene->diff();

          robot_state::RobotState& robot_state =
              scene_out->getCurrentStateNonConst();

          std::vector<double> joint_values = it->second;

          robot_state.setJointGroupPositions(arm_group_name_, joint_values);
          robot_state.update();

          collision_detection::CollisionRequest req;
          collision_detection::CollisionResult res;
          scene_out->checkCollision(req, res);

          found = !res.collision;

          if (!found)
          {
            cache.erase(it);
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

          std::vector<double> joint_values;
          scene_out->getCurrentState().copyJointGroupPositions(arm_group_name_,
                                                               joint_values);

          cache[action_name + "_" + object_name] = joint_values;

          action_solutions.emplace_back(task_solution[i], joint_values);

          if (!res)
          {

            for (int i = 0; i < collisions.size(); i++)
            {

              // avoid object collide with themself
              if(collisions[i] == object_name)
              {
                ROS_WARN("Object collide with themself.");
                continue;
              }

              // evito i duplicati
              if (std::find(objects.begin(), objects.end(), collisions[i]) ==
                  objects.end())
              {
                objects.push_back(collisions[i]);
              }

              // constraints.push_back(std::make_pair(collisions[i],
              // object_name));
              std::map<std::string, std::vector<std::string>>::iterator
                  constraints_it = constraints.find(collisions[i]);
              if (constraints_it != constraints.end())
              {
                constraints_it->second.push_back(object_name);
              }
              else
              {
                constraints[collisions[i]] = {object_name};
              }
            }
            f = false;
            ROS_WARN("Collisions found. Update contraints and replan.");
            break; // continue
          }
        }

        scene_out->decoupleParent();
        current_scene = scene_out;

        mps.attachObject(object_name, hand_frame_);
        mps.apply(current_scene, scene_out);

        scene_out->decoupleParent();
        current_scene = scene_out;
      }
      else if (action_name == "putdown")
      {
        std::string object_name = task_solution[i].parameters[0];

        std::vector<double> tf;

        if (removed < 8)
          tf = {0.0 + 0.05 * (removed - 1), -0.3, 0.101, 0.0, 0.0, 0.0};
        else
          tf = {0.0 + 0.05 * (removed - 8), 0.3, 0.101, 0.0, 0.0, 0.0};

        // place(scene_in, object_name, pose, trajectory, scene_out);

        Eigen::Isometry3d pose;
        convertDoublesToEigen(tf, pose);

        std::vector<Eigen::Isometry3d> poses;
        generateGraspingPoints(pose, 40, poses);

        std::vector<std::string> collisions;
        bool res = place.reason(current_scene, world_frame_, poses, scene_out, //"panda_link0"
                                collisions);

        std::vector<double> joint_values;
        scene_out->getCurrentState().copyJointGroupPositions(arm_group_name_,
                                                             joint_values);

        action_solutions.emplace_back(task_solution[i], joint_values);

        if (res)
        {
          scene_out->decoupleParent();
          current_scene = scene_out;

          mps.detachObject(object_name, hand_frame_);
          mps.apply(current_scene, scene_out);

          removed++;

          scene_out->decoupleParent();
          current_scene = scene_out;
        }
        else
        {
          f = false;
        }
      }
      else
      {
        ROS_ERROR("Undefined action name.");
        return -1;
      }
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

      planning_scene::PlanningScenePtr scene;
      current_state.compute(scene);

      moveit_tmp_msgs::ExecuteTMPSolutionGoal execute_goal;

      std::vector<moveit_tmp::ActionPipeline> execute;

      removed = tot_removed;

      // for (int i = 0; i < actions.size() && i < horizon_; i++)
      //{

      ROS_INFO_STREAM("Planning: " << action_solutions[0].first.name << " "
                                   << action_solutions[0].first.parameters[0]);

      std::vector<double> joint_values = action_solutions[0].second;

      robot_trajectory::RobotTrajectoryPtr traj;

      execute.emplace_back();
      moveit_tmp::ActionPipeline& p = execute.back();

      bool s = false;
      int count = 0;
      while (!s && count < 3)
      {

        if (action_solutions[0].first.name == "pickup")
        {
          s = pick.plan(scene, action_solutions[0].first.parameters[0],
                        joint_values, p);
        }
        else
        {
          s = place.plan(scene, action_solutions[0].first.parameters[0],
                         joint_values, p);
          // removed++;
        }
        count++;
      }

      ROS_INFO("Done");

      if (!s)
      {
        ros::shutdown();
        return 0;
      }

      ROS_INFO("Create execution message");
      // Compongo messaggio
      for (int i = 0; i < execute.size(); i++)
      {
        moveit_tmp::ActionPipeline p = execute[i];
        for (int j = 0; j < p.size(); j++)
        {
          execute_goal.solution.emplace_back();
          moveit_tmp_msgs::SubTrajectory& t = execute_goal.solution.back();
          p[j].first->getPlanningSceneDiffMsg(t.scene_diff);
          if (p[j].second)
            p[j].second->getRobotTrajectoryMsg(t.trajectory);
        }
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
      if (action_solutions[0].first.name == "putdown")
      {
        constraints.erase(action_solutions[0].first.parameters[0]);
        removed++;
      }

      tot_removed = removed;
      j++;
    }

    // Genero movimento casuale
    double r = (double)rand() / RAND_MAX;
    ROS_INFO_STREAM("RAND: " << r);
    if (r < 0.2)
    {

      ROS_WARN("Moved Object");
      int obj = rand() % num_objects;

      std::vector<double> object_pose;
      rosparam_shortcuts::get("test", nh, "object_" + std::to_string(obj),
                              object_pose);

      geometry_msgs::Pose pose;
      pose.position.x = object_pose[0] + 0.02;
      pose.position.y = object_pose[1] + 0.02;
      pose.position.z = 0.5 * 0.2;

      spawnObject(moveCollisionObject(pose, "object_" + std::to_string(obj)),
                  rviz_visual_tools::colors::BLUE);

      status_changed = true;
    }
  }

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  ROS_INFO_STREAM("Elapsed Time: " << elapsed_secs
                                   << " Reasoning Time: " << reasoning_time);
  ROS_INFO_STREAM("Removed Objects: " << tot_removed);
  myfile << elapsed_secs << ";" << tot_removed * 2 + 1 << ";" << f << "\n";
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
