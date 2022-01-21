#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

namespace gazebo
{
class smartpal : public ModelPlugin
{
private:
  physics::ModelPtr model;
  physics::JointController *jcl1;
  physics::JointController *jcl2;
  physics::JointController *jcl3;
  physics::JointController *jcl4;
  physics::JointController *jcl5;
  physics::JointController *jcl6;
  physics::JointController *jcl7;
  physics::JointController *jcl_gripper;
  physics::JointPtr jl1;
  physics::JointPtr jl2;
  physics::JointPtr jl3;
  physics::JointPtr jl4;
  physics::JointPtr jl5;
  physics::JointPtr jl6;
  physics::JointPtr jl7;
  physics::JointPtr jl_gripper;
  physics::JointController *jcr1;
  physics::JointController *jcr2;
  physics::JointController *jcr3;
  physics::JointController *jcr4;
  physics::JointController *jcr5;
  physics::JointController *jcr6;
  physics::JointController *jcr7;
  physics::JointController *jcr_gripper;
  physics::JointPtr jr1;
  physics::JointPtr jr2;
  physics::JointPtr jr3;
  physics::JointPtr jr4;
  physics::JointPtr jr5;
  physics::JointPtr jr6;
  physics::JointPtr jr7;
  physics::JointPtr jr_gripper;
  event::ConnectionPtr updateConnection;
  ros::NodeHandle n;
  ros::Subscriber sub;

  float angle_l[8], angle_r[8];
  float angle_buf_l[8], angle_buf_r[8];
  int time;
  double wdt;

public:
  smartpal()
  {
  }
  ~smartpal()
  {
    this->n.shutdown();
  }

  void Load(physics::ModelPtr _parent, sdf::ElementPtr)
  {
    int argc = 0;
    char **argv = NULL;

    ros::init(argc, argv, "smartpal");
    sub = n.subscribe("joint_angle_smartpal", 1, &smartpal::callback_goto, this);

    model = _parent;
    jcl1 = new physics::JointController(model);
    jcl2 = new physics::JointController(model);
    jcl3 = new physics::JointController(model);
    jcl4 = new physics::JointController(model);
    jcl5 = new physics::JointController(model);
    jcl6 = new physics::JointController(model);
    jcl7 = new physics::JointController(model);
    jcl_gripper = new physics::JointController(model);

    jl1 = model->GetJoint("l_arm_j1_joint");
    jl2 = model->GetJoint("l_arm_j2_joint");
    jl3 = model->GetJoint("l_arm_j3_joint");
    jl4 = model->GetJoint("l_arm_j4_joint");
    jl5 = model->GetJoint("l_arm_j5_joint");
    jl6 = model->GetJoint("l_arm_j6_joint");
    jl7 = model->GetJoint("l_arm_j7_joint");
    jl_gripper = model->GetJoint("l_gripper_thumb_joint");

    jcr1 = new physics::JointController(model);
    jcr2 = new physics::JointController(model);
    jcr3 = new physics::JointController(model);
    jcr4 = new physics::JointController(model);
    jcr5 = new physics::JointController(model);
    jcr6 = new physics::JointController(model);
    jcr7 = new physics::JointController(model);
    jcr_gripper = new physics::JointController(model);

    jr1 = model->GetJoint("r_arm_j1_joint");
    jr2 = model->GetJoint("r_arm_j2_joint");
    jr3 = model->GetJoint("r_arm_j3_joint");
    jr4 = model->GetJoint("r_arm_j4_joint");
    jr5 = model->GetJoint("r_arm_j5_joint");
    jr6 = model->GetJoint("r_arm_j6_joint");
    jr7 = model->GetJoint("r_arm_j7_joint");
    jr_gripper = model->GetJoint("r_gripper_thumb_joint");

    updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&smartpal::OnUpdate, this, _1));
    time = 0;

    angle_l[0] = angle_buf_l[0] = 0;
    angle_l[1] = angle_buf_l[1] = 0;
    angle_l[2] = angle_buf_l[2] = 0;
    angle_l[3] = angle_buf_l[3] = 0;
    angle_l[4] = angle_buf_l[4] = 0;
    angle_l[5] = angle_buf_l[5] = 0;
    angle_l[6] = angle_buf_l[6] = 0;
    angle_l[7] = angle_buf_l[7] = 0;

    angle_r[0] = angle_buf_r[0] = 0;
    angle_r[1] = angle_buf_r[1] = 0;
    angle_r[2] = angle_buf_r[2] = 0;
    angle_r[3] = angle_buf_r[3] = 0;
    angle_r[4] = angle_buf_r[4] = 0;
    angle_r[5] = angle_buf_r[5] = 0;
    angle_r[6] = angle_buf_r[6] = 0;
    angle_r[7] = angle_buf_r[7] = 0;

    wdt = 0;
  }

  void callback_goto(const std_msgs::Float32MultiArrayPtr &msg)
  {
    angle_buf_l[0] = msg->data[0];
    angle_buf_l[1] = msg->data[1];
    angle_buf_l[2] = msg->data[2];
    angle_buf_l[3] = msg->data[3];
    angle_buf_l[4] = msg->data[4];
    angle_buf_l[5] = msg->data[5];
    angle_buf_l[6] = msg->data[6];
    angle_buf_l[7] = msg->data[7];
  }

  // 1000Hz =  0.001sec
  void OnUpdate(const common::UpdateInfo &)
  {
    time += 1;
    wdt += 1;

    if (time >= 100)  // 0.1sec
    {
      time = 0;
      if (angle_l[0] > angle_buf_l[0])
        angle_l[0] -= 0.01;
      if (angle_l[0] < angle_buf_l[0])
        angle_l[0] += 0.01;
      if (angle_l[1] > angle_buf_l[1])
        angle_l[1] -= 0.01;
      if (angle_l[1] < angle_buf_l[1])
        angle_l[1] += 0.01;
      if (angle_l[2] > angle_buf_l[2])
        angle_l[2] -= 0.01;
      if (angle_l[2] < angle_buf_l[2])
        angle_l[2] += 0.01;
      if (angle_l[3] > angle_buf_l[3])
        angle_l[3] -= 0.01;
      if (angle_l[3] < angle_buf_l[3])
        angle_l[3] += 0.01;
      if (angle_l[4] > angle_buf_l[4])
        angle_l[4] -= 0.01;
      if (angle_l[4] < angle_buf_l[4])
        angle_l[4] += 0.01;
      if (angle_l[5] > angle_buf_l[5])
        angle_l[5] -= 0.01;
      if (angle_l[5] < angle_buf_l[5])
        angle_l[5] += 0.01;
      if (angle_l[6] > angle_buf_l[6])
        angle_l[6] -= 0.01;
      if (angle_l[6] < angle_buf_l[6])
        angle_l[6] += 0.01;
      if (angle_l[7] > angle_buf_l[7])
        angle_l[7] -= 0.01;
      if (angle_l[7] < angle_buf_l[7])
        angle_l[7] += 0.01;
    }

    jcl1->SetJointPosition(jl1, angle_l[0]);
    jcl2->SetJointPosition(jl2, angle_l[1]);
    jcl3->SetJointPosition(jl3, angle_l[2]);
    jcl4->SetJointPosition(jl4, angle_l[3]);
    jcl5->SetJointPosition(jl5, angle_l[4]);
    jcl6->SetJointPosition(jl6, angle_l[5]);
    jcl7->SetJointPosition(jl7, angle_l[6]);
    jcl_gripper->SetJointPosition(jl_gripper, angle_l[7]);

    jcr1->SetJointPosition(jr1, angle_r[0]);
    jcr2->SetJointPosition(jr2, -0.08);
    jcr3->SetJointPosition(jr3, angle_r[2]);
    jcr4->SetJointPosition(jr4, angle_r[3]);
    jcr5->SetJointPosition(jr5, angle_r[4]);
    jcr6->SetJointPosition(jr6, angle_r[5]);
    jcr7->SetJointPosition(jr7, angle_r[6]);
    jcr_gripper->SetJointPosition(jr_gripper, angle_r[7]);
  }
};
GZ_REGISTER_MODEL_PLUGIN(smartpal)
}
