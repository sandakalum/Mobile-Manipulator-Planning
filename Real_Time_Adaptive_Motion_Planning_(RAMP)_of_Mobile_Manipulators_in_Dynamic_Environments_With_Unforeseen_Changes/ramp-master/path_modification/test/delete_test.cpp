
#include <gtest/gtest.h>
#include "../include/delete.h"


TEST(TestSuite, perform)
{

  ramp_msgs::KnotPoint kp1;
  kp1.motionState.positions.push_back(1);
  kp1.motionState.positions.push_back(1);
  kp1.motionState.positions.push_back(1);

  ramp_msgs::KnotPoint kp2;
  kp2.motionState.positions.push_back(2);
  kp2.motionState.positions.push_back(2);
  kp2.motionState.positions.push_back(2);

  ramp_msgs::KnotPoint kp3;
  kp3.motionState.positions.push_back(3);
  kp3.motionState.positions.push_back(3);
  kp3.motionState.positions.push_back(3);

  ramp_msgs::Path p;
  p.points.push_back(kp1); 
  p.points.push_back(kp2);
  p.points.push_back(kp3);

  size_t p_size = p.points.size();

  Delete d;
  d.path_ = p;   
  d.perform();


  // Check the size
  EXPECT_EQ(d.path_.points.size(), p_size-1);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
