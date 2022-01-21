
#include <gtest/gtest.h>
#include "../include/swap.h"


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

  ramp_msgs::KnotPoint kp4;
  kp4.motionState.positions.push_back(4);
  kp4.motionState.positions.push_back(4);
  kp4.motionState.positions.push_back(4);

  ramp_msgs::Path p;
  p.points.push_back(kp1); 
  p.points.push_back(kp2);
  p.points.push_back(kp3);
  p.points.push_back(kp4);

  size_t p_size = p.points.size();

  Swap s;
  s.path_ = p;
  ramp_msgs::Path old = p;  
  s.perform();


  // Check the size
  EXPECT_EQ(s.path_.points.size(), p_size);


  // Check that TWO states have changed after perform()
  uint8_t count=0;
  for(uint8_t i=0;i<s.path_.points.size();i++)
  {
    ramp_msgs::MotionState m = s.path_.points.at(i).motionState;
    ramp_msgs::MotionState m_old = old.points.at(i).motionState;
    if( sqrt( pow( m.positions.at(0) - m_old.positions.at(0), 2) +
              pow( m.positions.at(1) - m_old.positions.at(1), 2) ) > 0.0001)
    {
      count++;
    } // end if
  } // end for

  EXPECT_EQ(count, 2);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
