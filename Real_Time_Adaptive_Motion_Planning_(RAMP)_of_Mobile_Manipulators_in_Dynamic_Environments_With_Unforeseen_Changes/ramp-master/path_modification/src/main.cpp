#include "ros/ros.h"
#include "ramp_msgs/Range.h"
#include "utility.h"
#include "modifier.h"

Utility u;

bool handleRequest(ramp_msgs::ModificationRequest::Request& req,
                   ramp_msgs::ModificationRequest::Response& res)
{
  /*std::cout<<"\npath_modification: In handleRequest\n";

  std::cout<<"\nNumber of paths received: "<<req.paths.size();
  std::cout<<"\nPaths received:";
  for(unsigned int i=0;i<req.paths.size();i++) {
    std::cout<<"\n"<<u.toString(req.paths.at(i));
  }
  std::cout<<"\noperator: "<<req.op<<"\n";*/

  Modifier mod(req);
  res.mod_paths = mod.perform();
  
  //Insert insert(req.paths.at(0));
  //res.mod_paths.push_back(insert.perform());
  
  //Delete del(req.paths.at(0));
  //res.mod_paths.push_back(del.perform());
  
  //Change change(req.paths.at(0));
  //res.mod_paths.push_back(change.perform());

  //Swap swap(req.paths.at(0));
  //res.mod_paths.push_back(swap.perform());
  
  //Crossover cross(req.paths.at(0), req.paths.at(1));
  //res.mod_paths = cross.perform();
  
  /*std::cout<<"\nPath(s) after modification:";
  for(unsigned int i=0;i<res.mod_paths.size();i++) {
    std::cout<<"\n"<<u.toString(res.mod_paths.at(i));
  }*/

  return true;
}




int main(int argc, char** argv) {
  ros::init(argc, argv, "path_modification");
  srand(time(0)); 

  ros::NodeHandle handle;

  ros::ServiceServer service = handle.advertiseService("path_modification", handleRequest); 

  Utility u;

  ramp_msgs::Path p1;
  for(unsigned int i=0;i<10;i++) {
    ramp_msgs::KnotPoint kp;
    kp.motionState.positions.push_back(i);
    kp.motionState.positions.push_back(i+1);
    kp.motionState.positions.push_back(i+2);

    p1.points.push_back(kp);
  }
  
  ROS_INFO("Path before modification: %s", u.toString(p1).c_str());

  // Test change
  //Change change(p1);
  //change.perform();
  //Swap swap(p1);
  //swap.perform();
  Move m(p1);
  m.dir_ = PI/4.;
  m.dist_ = 1;
  m.perform();

  ROS_INFO("Path after modification: %s", u.toString(p1).c_str());

/*  ramp_msgs::Path p2;
  for(unsigned int i=5;i>0;i--) {
    ramp_msgs::KnotPoint kp;
    kp.configuration.K.push_back(i);
    kp.configuration.ranges.push_back(r); 

    p2.points.push_back(kp);
  }
  
  std::cout<<"\nPath p2:"<<u.toString(p2);
  //Insert insert(p);
  //Delete del(p);
  //Change cha(p);
  //Swap swap(p);
  Crossover cross(p1, p2);

  //ramp_msgs::Path a = insert.perform();
  //ramp_msgs::Path a = del.perform();
  //ramp_msgs::Path a = cha.perform();
  //ramp_msgs::Path a = swap.perform();
  std::vector<ramp_msgs::Path> as = cross.perform();
  std::cout<<"\nnew path1:"<<u.toString(as.at(0));
  std::cout<<"\nnew path2:"<<u.toString(as.at(1));*/


  std::cout<<"\nSpinning...\n";
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
