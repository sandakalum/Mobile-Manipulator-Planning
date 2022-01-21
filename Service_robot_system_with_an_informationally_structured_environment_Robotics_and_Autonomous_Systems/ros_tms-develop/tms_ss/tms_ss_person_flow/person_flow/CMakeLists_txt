# cmake_minimum_required(VERSION 2.8.3)
# project(tms_ss_pot)
#
# ###################################
# ## packege  find   configuration ##
# ###################################
#
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   sensor_msgs
#   tms_msg_ss
#   tms_msg_db
#   cv_bridge
#   )
#
# find_package(Qt4 4.6 REQUIRED COMPONENTS QtCore QtGui)
# include(${QT_USE_FILE})
# find_package(Boost REQUIRED COMPONENTS thread)
#
# ###################################
# ## catkin specific configuration ##
# ###################################
# catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES tms_ss_pot
#    CATKIN_DEPENDS message_runtime roscpp std_msgs
#    DEPENDS system_lib
# )
#
# ###########
# ## Build ##
# ###########
#
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   /usr/local/include/opencv2
#   /usr/local/include/opencv
#   ${BOOST_INCLUDE_DIRS}
#   ${QT_INCLUDE_DIRS}
# )
#
# ## Declare a cpp library
#  add_library(tms_ss_pot
#    include/tms_ss_pot/Particle_filter.cpp
#    include/tms_ss_pot/Target.cpp
#    include/tms_ss_pot/Laser.cpp
#    include/tms_ss_pot/Multiple_particle_filter.cpp
#  )
#
#
# add_executable(pot_ctrl src/pot_ctrl.cpp include/tms_ss_pot/Particle_filter.cpp include/tms_ss_pot/Target.cpp include/tms_ss_pot/Laser.cpp include/tms_ss_pot/Multiple_particle_filter.cpp)
# add_executable(pot_tracker_s src/pot_tracker_single.cpp include/tms_ss_pot/Particle_filter.cpp include/tms_ss_pot/Target.cpp include/tms_ss_pot/Laser.cpp include/tms_ss_pot/Multiple_particle_filter.cpp)
# add_executable(pot_tracker_d src/pot_tracker_double.cpp include/tms_ss_pot/Particle_filter.cpp include/tms_ss_pot/Target.cpp include/tms_ss_pot/Laser.cpp include/tms_ss_pot/Multiple_particle_filter.cpp)
# add_executable(pot_tracker_msg src/pot_tracker_msg.cpp include/tms_ss_pot/Particle_filter.cpp include/tms_ss_pot/Target.cpp include/tms_ss_pot/Laser.cpp include/tms_ss_pot/Multiple_particle_filter.cpp)
# add_executable(pot_trackd src/pot_trackd.cpp include/tms_ss_pot/Particle_filter.cpp include/tms_ss_pot/Target.cpp include/tms_ss_pot/Laser.cpp include/tms_ss_pot/Multiple_particle_filter.cpp)
# add_executable(pot_tracker_m src/pot_tracker_multi.cpp include/tms_ss_pot/Particle_filter.cpp include/tms_ss_pot/Target.cpp include/tms_ss_pot/Laser.cpp include/tms_ss_pot/Multiple_particle_filter.cpp)
# add_executable(pot_tracker_p src/pot_tracker_psen.cpp include/tms_ss_pot/Particle_filter.cpp include/tms_ss_pot/Target.cpp include/tms_ss_pot/Laser.cpp include/tms_ss_pot/Multiple_particle_filter.cpp)
# add_executable(pot_mapping src/pot_mapping.cpp  include/tms_ss_pot/Target.cpp include/tms_ss_pot/Laser.cpp)
# add_executable(pot_cluster src/pot_cluster.cpp  include/tms_ss_pot/Target.cpp include/tms_ss_pot/Laser.cpp)
# add_executable(pot_urg_scan src/pot_urg_scan.cpp)
# add_executable(pot_urg_scans src/pot_urg_scans.cpp)
# add_executable(pot_manager src/pot_manager.cpp)
# add_executable(pot_psen_manager src/pot_psen_manager.cpp)
# add_executable(pot_psen_generalizar src/pot_psen_generalizar.cpp)
#
# add_dependencies(pot_ctrl tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
# add_dependencies(pot_tracker_s tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
# add_dependencies(pot_tracker_d tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
# add_dependencies(pot_urg_scan tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
# add_dependencies(pot_urg_scans tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
# add_dependencies(pot_tracker_msg tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
# add_dependencies(pot_manager tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
# add_dependencies(pot_cluster tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
# add_dependencies(pot_trackd tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
# add_dependencies(pot_mapping tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
# add_dependencies(pot_tracker_m tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
# add_dependencies(pot_tracker_p tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
# add_dependencies(pot_psen_manager tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
# add_dependencies(pot_psen_generalizar tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
#
# target_link_libraries(pot_ctrl        ${catkin_LIBRARIES})
# target_link_libraries(pot_tracker_s   ${catkin_LIBRARIES})
# target_link_libraries(pot_tracker_d   ${catkin_LIBRARIES})
# target_link_libraries(pot_urg_scan    ${catkin_LIBRARIES})
# target_link_libraries(pot_urg_scans   ${catkin_LIBRARIES})
# target_link_libraries(pot_tracker_msg ${catkin_LIBRARIES})
# target_link_libraries(pot_manager     ${catkin_LIBRARIES})
# target_link_libraries(pot_cluster     ${catkin_LIBRARIES})
# target_link_libraries(pot_trackd      ${catkin_LIBRARIES})
# target_link_libraries(pot_mapping     ${catkin_LIBRARIES})
# target_link_libraries(pot_tracker_m    ${catkin_LIBRARIES})
# target_link_libraries(pot_tracker_p    ${catkin_LIBRARIES})
# target_link_libraries(pot_psen_manager  ${catkin_LIBRARIES})
# target_link_libraries(pot_psen_generalizar  ${catkin_LIBRARIES})
