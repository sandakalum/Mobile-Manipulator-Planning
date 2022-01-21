# cmake_minimum_required(VERSION 2.8.3)
# project(tms_ss_ibs)
#
# find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
#
# ###################################
# ## catkin specific configuration ##
# ###################################
# catkin_package(
#    LIBRARIES tms_ss_ibs
#    CATKIN_DEPENDS roscpp std_msgs message_runtime
# )
#
# ###########
# ## Build ##
# ###########
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )
#
# add_executable(ibs src/ibs.cpp)
# add_dependencies(ibs DEPENDS tms_msg_db_gencpp tms_msg_ss_gencpp)
# target_link_libraries(ibs ${catkin_LIBRARIES})
