cmake_minimum_required( VERSION 2.8.11 )

project( ktmpb )

find_package( catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs 
    message_generation 
    roslib
    ff
    kautham
    genmsg
    rospy
    geometry_msgs
    
)
#catkin_python_setup()
#find_package( 
#   kautham REQUIRED
#)

if(KAUTHAM_INCLUDE_DIR)
    message(STATUS "Kautham found and its include dir is $(KAUTHAM_INCLUDE_DIR")
endif()

add_message_files(
  FILES
  fVector.msg
)
add_service_files(
  FILES
  AddRobot.srv
  AddObstacle.srv
  AttachObstacle2RobotLink.srv
  CheckCollision.srv
  ClearSampleSet.srv
  CloseProblem.srv
  Connect.srv
  DetachObstacle.srv
  FindIK.srv
  GetLastPlanComputationTime.srv
  GetNumEdges.srv
  GetNumVertices.srv
  GetPath.srv
  ObsPos.srv
  OpenProblem.srv
  OpenProblemStream.srv
  ProblemOpened.srv
  RemoveObstacle.srv
  RemoveRobot.srv
  SetDefaultRobControls.srv
  SetFixedObsControls.srv
  SetGoal.srv
  SetInit.srv
  SetInitObs.srv
  SetObsControls.srv
  SetObsControlsStream.srv
  SetObstaclesConfig.srv
  SetPlanner.srv
  SetPlannerByName.srv
  SetPlannerParameter.srv
  SetPlannerStream.srv
  SetQuery.srv
  SetRobControls.srv
  SetRobControlsNoQuery.srv
  SetRobControlsStream.srv
  SetRobotsConfig.srv
  Solve.srv
  Plan.srv
  TiagoIk.srv
  GraspPlanning.srv
)


########### Modules path and system options ###############
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)


generate_messages(DEPENDENCIES roscpp std_msgs geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})




