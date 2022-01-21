cmake_minimum_required(VERSION 2.8.3)
project(tms_rc_smartpal_tagreader)

find_package(catkin REQUIRED COMPONENTS roscpp tms_msg_rc)

catkin_package(
   #INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable( smartpal_tagreader
   
                src/smartpal_tagreader.cpp       

                src/corba_client.cpp
                src/corba_cmd_tag.cpp
                  
                src/corba/tag.cpp)

add_dependencies(smartpal_tagreader tms_msg_rc_gencpp)

target_link_libraries(smartpal_tagreader ${catkin_LIBRARIES} omniORB4 omnithread)
