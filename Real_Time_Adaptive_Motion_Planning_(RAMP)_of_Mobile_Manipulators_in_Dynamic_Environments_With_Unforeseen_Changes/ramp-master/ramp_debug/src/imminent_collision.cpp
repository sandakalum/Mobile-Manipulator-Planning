#include <iostream>
#include "ros/ros.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "check_imminent_collision");
  ros::NodeHandle handle;

  bool immiC;
  while(ros::ok()) {
    ros::param::get("imminent_collision", immiC);

    if(immiC) {
      std::cout<<"\nImminent Collision: True\n Press Enter to change it to false!\n";
      std::cin.get();
      ros::param::set("imminent_collision", false);
    }
    else {
      std::cout<<"\nImminent Collision: False\n Press Enter to change it to true!\n";
      std::cin.get();
      ros::param::set("imminent_collision", true);
    }

    ros::spinOnce();
  }

  std::cout<<"\nExiting Normally\n";
  return 0;
}
