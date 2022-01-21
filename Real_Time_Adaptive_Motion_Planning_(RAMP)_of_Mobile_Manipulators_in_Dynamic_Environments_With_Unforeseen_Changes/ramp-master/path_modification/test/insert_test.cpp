
#include <gtest/gtest.h>
#include "../include/insert.h"


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

  Insert i;
  i.path_ = p;   
  i.perform();


  // Check the size
  EXPECT_EQ(i.path_.points.size(), p_size+1);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
