cmake_minimum_required(VERSION 3.0.2)
project(moveit_tmp)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  eigen_conversions
  interactive_markers
  eigen_conversions
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_tmp_msgs
  rosparam_shortcuts
)

catkin_package(
    LIBRARIES
            ${PROJECT_NAME}
    INCLUDE_DIRS
            include
    CATKIN_DEPENDS
            moveit_core
            moveit_tmp_msgs
            moveit_visual_tools
            moveit_ros_planning_interface
)

# check for MOVEIT_MASTER
include(CheckIncludeFileCXX)
set(CMAKE_REQUIRED_INCLUDES ${moveit_core_INCLUDE_DIRS})
set(CMAKE_REQUIRED_FLAGS "-std=c++11")
CHECK_INCLUDE_FILE_CXX(moveit/collision_detection/collision_env.h MOVEIT_MASTER)
if(NOT MOVEIT_MASTER)
  set(MOVEIT_MASTER 0)
endif()
add_definitions(-DMOVEIT_MASTER=${MOVEIT_MASTER})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DDATA_PATH="${PROJECT_SOURCE_DIR}/pddl/")

add_library(${PROJECT_NAME}
    src/action_cache.cpp
    src/action_planner.cpp
    src/compute_ik.cpp
    src/current_state.cpp
    src/knowledge_base.cpp
    src/modify_planning_scene.cpp
    src/move.cpp
    src/move_cartesian.cpp
    src/place_pose_generator.cpp
    src/pick.cpp
    src/place.cpp
    src/planner.cpp
    src/task_planner.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(finite apps/finite_window.cpp)
target_link_libraries(finite ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(new_prob apps/non_mono.cpp)
target_link_libraries(new_prob ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(baseline apps/finite_window_baseline.cpp)
target_link_libraries(baseline ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(non_mono_baseline apps/non_mono_baseline.cpp)
target_link_libraries(non_mono_baseline ${catkin_LIBRARIES} ${PROJECT_NAME})

install(
  DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY common/bin
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common/bin/
  FILE_PERMISSIONS OWNER_EXECUTE
)
