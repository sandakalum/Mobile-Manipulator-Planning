# cmake_minimum_required(VERSION 2.8.3)
# project(tms_rc_katana)
#
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   tms_msg_rc
# )
#
# ###################################
# ## catkin specific configuration ##
# ###################################
# catkin_package(
# )
#
# ###########
# ## Build ##
# ###########
#
# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# # include_directories(include)
# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   ./KNI_4.3.0/include
#   ./KNI_4.3.0/demo/common
# )

## Declare a cpp library
# add_library(ros_katana
#   src/${PROJECT_NAME}/ros_katana.cpp
# )

# execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/common/ make
#                 RESULT_VARIABLE _make_failed)
# if(_make_failed)
#   message(FATAL_ERROR "Build of common files failed")
# endif(_make_failed)
#
# execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/Base/ make
#                 RESULT_VARIABLE _make_failed)
# if(_make_failed)
#   message(FATAL_ERROR "Build of Base library failed")
# endif(_make_failed)
#
# execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/lib/kinematics/roboop/ make
#                 RESULT_VARIABLE _make_failed)
# if(_make_failed)
#   message(FATAL_ERROR "Build of roboop library failed")
# endif(_make_failed)
#
# execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/lib/kinematics/ make
#                 RESULT_VARIABLE _make_failed)
# if(_make_failed)
#   message(FATAL_ERROR "Build of kinematics library failed")
# endif(_make_failed)
#
# execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/InvKin/ make
#                 RESULT_VARIABLE _make_failed)
# if(_make_failed)
#   message(FATAL_ERROR "Build of InvKin library failed")
# endif(_make_failed)
#
# execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/LM/ make
#                 RESULT_VARIABLE _make_failed)
# if(_make_failed)
#   message(FATAL_ERROR "Build of LM library failed")
# endif(_make_failed)

# ## Declare a cpp executable
# link_directories(./KNI_4.3.0/lib/linux)
# add_executable(roskatana_test1 src/roskatana_test1.cpp)
# target_link_libraries(roskatana_test1 ${catkin_LIBRARIES} KNIBase Kinematics KNI_InvKin KNI_LM pthread)
# add_dependencies(roskatana_test1 DEPENDS tms_msg_rc_gencpp)
#
# add_executable(roskatana_test2 src/roskatana_test2.cpp)
# target_link_libraries(roskatana_test2 ${catkin_LIBRARIES})
# add_dependencies(roskatana_test2 DEPENDS tms_msg_rc_gencpp)
