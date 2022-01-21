# ###############################
# ## cmake and project setting ##
# ###############################
# cmake_minimum_required(VERSION 2.8.3)
# project(corridor_viewer)
#
# find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
#
# ###################################
# ## catkin specific configuration ##
# ###################################
# catkin_package(
#   LIBRARIES corridor_viewer
#   CATKIN_DEPENDS roscpp std_msgs message_runtime
# )
#
# ###############################
# ## Qt specific configuration ##
# ###############################
# find_package(Qt4 REQUIRED)
#
# set(QT_USE_QTCORE TRUE)
# set(QT_USE_QTGUI TRUE)
# set(QT_USE_QTOPENGL TRUE)
# set(QT_USE_QTXML TRUE)
#
# include(${QT_USE_FILE})
#
# include_directories(${QT_INCLUDE_DIR} ${QT_OPENGL_INCLUDE_DIR})
#
# add_definitions(-DQT_NO_KEYWORDS)
#
#
# ###########
# ## Build ##
# ###########
#
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${CMAKE_CURRENT_BINARY_DIR}
#   ${CMAKE_CURRENT_SOURCE_DIR}
# )
#
# set( headers
#     src/mainwindow.h
#     src/viewer.h
# )
#
# qt4_wrap_cpp(moc_sources ${headers})
#
# set( sources
#     src/main.cpp
#     src/mainwindow.cpp
#     src/viewer.cpp
#     ${moc_sources}
#  )
#
# file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.ui)
# file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.qrc)
# file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.hpp)
# file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINK src/*.*)
#
# QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
# QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
# QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
#
# #############################################
# # Binaries
# #############################################
#
# add_executable(corridor_viewer ${sources} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${moc_sources})
#
# add_dependencies(corridor_viewer DEPENDS tms_msg_ss_gencpp)
#
# target_link_libraries(corridor_viewer ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB} ${QT_LIBRARIES})
#
#
#
