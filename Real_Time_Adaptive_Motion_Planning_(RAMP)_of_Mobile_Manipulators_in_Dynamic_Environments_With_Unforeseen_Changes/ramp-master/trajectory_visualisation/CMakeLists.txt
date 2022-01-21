cmake_minimum_required(VERSION 2.8.3)
project(trajectory_visualisation)


find_package(catkin REQUIRED COMPONENTS roscpp ramp_msgs)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtWebKit QtOpengl)
include(${QT_USE_FILE})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp qt-ros)



file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h)
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
qt4_automoc(${QT_SOURCES})

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

include_directories(include ${catkin_INCLUDE_DIRS} /usr/include/qt4)

set (CMAKE_CXX_FLAGS "-g")

##############################################################################
# Binaries
##############################################################################

add_executable(trajectory_visualisation ${QT_SOURCES} src/utility.cpp ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(trajectory_visualisation ${QT_LIBRARIES})
target_link_libraries(trajectory_visualisation ${catkin_LIBRARIES})
