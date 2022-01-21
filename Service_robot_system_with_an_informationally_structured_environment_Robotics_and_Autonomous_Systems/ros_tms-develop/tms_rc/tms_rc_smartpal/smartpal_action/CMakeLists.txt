###############################
## cmake and project setting ##
###############################

cmake_minimum_required(VERSION 2.8.3)
project(tms_rc_smartpal_action)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES tms_rc_smartpal_action
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  #DEPENDS eigen
)


#######################################
## choreonoid specific configuration ##
#######################################

set(ChoreonoidPath ~/choreonoid)
set(graspPluginPath ${ChoreonoidPath}/extplugin/graspPlugin)
set(prmPath ${ChoreonoidPath}/extplugin/graspPlugin/PRM)
set(ChoreonoidLibPath ${ChoreonoidPath}/lib/choreonoid-1.4)

set(libname CnoidSmartpalActionPlugin)


###############################
## Qt specific configuration ##
###############################

find_package(Qt5Core REQUIRED)
set(QT_USE_QTOPENGL TRUE)
#include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIR} ${QT_OPENGL_INCLUDE_DIR})
add_definitions(-DQT_NO_KEYWORDS)


###########
## Build ##
###########

# include_directories(
#   #include
#   ${catkin_INCLUDE_DIRS}
#   ${ChoreonoidPath}
#   ${ChoreonoidPath}/include
#   ${graspPluginPath}
#   ${prmPath}
#   /usr/include/eigen3
# )

# link_directories(
#   ${ChoreonoidLibPath}
# )

# add_library(
#   ${libname}
#   SHARED
#   #src/smartpal4_action.cpp
#   src/smartpal5_action.cpp
# )

# add_dependencies(${libname} DEPENDS tms_msg_rc_gencpp tms_msg_rp_gencpp)

# target_link_libraries(
#   ${libname}
#   CnoidBodyPlugin
#   CnoidPoseSeqPlugin
#   CnoidGraspPlugin
#   CnoidPRMPlugin
#   uuid
#   dl
#   pthread
#   omniORB4
#   omnithread
#   omniDynamic4
#   ${catkin_LIBRARIES}
# )

# set_target_properties(
#   ${libname}
#   PROPERTIES LIBRARY_OUTPUT_DIRECTORY
#   ${ChoreonoidLibPath}
# )
