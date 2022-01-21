#include "modifier.h"

Modifier::Modifier(ramp_msgs::ModificationRequest::Request& req) : mod_req(req) {}



const std::vector<ramp_msgs::Path> Modifier::perform() {
  std::vector<ramp_msgs::Path> result;

  //std::cout<<"\nModifier received:"<<u.toString(mod_req.paths.at(0))<<"\n";

  if(mod_req.op == "insert") 
  {
    in_.path_ = mod_req.paths.at(0); 
    result.push_back(in_.perform());
  }

  else if(mod_req.op == "delete") 
  {
    del_.path_ = mod_req.paths.at(0);
    result.push_back(del_.perform());
  }
  
  else if(mod_req.op == "change") 
  {
    chg_.path_ = mod_req.paths.at(0);
    result.push_back(chg_.perform());
  }

  else if(mod_req.op == "swap_") 
  {
    swap_.path_ = mod_req.paths.at(0);
    result.push_back(swap_.perform());
  }

  else if(mod_req.op == "crossover") 
  {
    cross_.path1_ = mod_req.paths.at(0);
    cross_.path2_ = mod_req.paths.at(1);
    result = cross_.perform();
  }

  else if(mod_req.op == "move")
  {
    move_.path_ = mod_req.paths[0];
    move_.dir_ = mod_req.move_dir;
    move_.dist_ = mod_req.move_dist;
    result.push_back(move_.perform());
  }

  //std::cout<<"\nModifier returning:"<<u.toString(result.at(0))<<"\n";

  return result;
}
