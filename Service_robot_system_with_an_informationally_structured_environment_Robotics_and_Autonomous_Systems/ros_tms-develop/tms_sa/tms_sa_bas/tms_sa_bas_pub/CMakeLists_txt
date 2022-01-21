cmake_minimum_required(VERSION 2.8.3)
project(tms_sa_bas_pub)
find_package(catkin REQUIRED COMPONENTS roscpp)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   LIBRARIES tms_sa_bas_pub
   CATKIN_DEPENDS roscpp message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(baspub src/baspub.cpp)
add_dependencies(baspub DEPENDS tms_msg_ss_gencpp)
target_link_libraries(baspub ${catkin_LIBRARIES})

