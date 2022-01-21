#include <crossover.h>

Crossover::Crossover(const ramp_msgs::Path p1, const ramp_msgs::Path p2) : path1_(p1), path2_(p2) {} 


const std::vector<ramp_msgs::Path> Crossover::perform() 
{
  std::vector<ramp_msgs::Path> result;

  //Randomly choose a configuration in each path to split on
  unsigned int i_knotPoint1 = rand() % (path1_.points.size()-1); 
  unsigned int i_knotPoint2 = rand() % (path2_.points.size()-1); 


  ramp_msgs::Path r1;
  //Push on the first part of the first path
  for(unsigned int i=0; i<=i_knotPoint1; i++) 
  {
    r1.points.push_back(path1_.points.at(i));
  }


  //Push on the second part of the second path
  for(unsigned int i=i_knotPoint2+1; i<path2_.points.size(); i++) 
  {
    r1.points.push_back(path2_.points.at(i));
  }


  ramp_msgs::Path r2;
  //Push on the first part of the second path
  for(unsigned int i=0; i<=i_knotPoint2; i++) 
  {
    r2.points.push_back(path2_.points.at(i));
  }
  
  
  //Push on the second part of the first path
  for(unsigned int i=i_knotPoint1+1; i<path1_.points.size(); i++) 
  {
    r2.points.push_back(path1_.points.at(i));
  }

  result.push_back(r1);
  result.push_back(r2);

  return result;
}

