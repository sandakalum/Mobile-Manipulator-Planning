#ifndef _CALC_JOINT_ANGLES_H_
#define _CALC_JOINT_ANGLES_H_

#include <math.h>

#include <vector>
#include <map>
#include <Eigen/Eigen>

#include <tms_msg_ss/Skeleton.h>

using namespace std;

typedef enum
{
  SpineBase,
  SpineMid,
  Neck,
  Head,
  ShoulderLeft,
  ElbowLeft,
  WristLeft,
  HandLeft,
  ShoulderRight,
  ElbowRight,
  WristRight,
  HandRight,
  HipLeft,
  KneeLeft,
  AnkleLeft,
  FootLeft,
  HipRight,
  KneeRight,
  AnkleRight,
  FootRight,
  SpineShoulder,
  HandTipLeft,
  ThumbLeft,
  HandTipRight,
  ThumbRight,
  JOINT_NUM
} JOINT_NAME;

const char *kJointName[] = {
    //"WAIST_JOINT1",
    //"WAIST_JOINT2",
    "R_ARM_JOINT1", "R_ARM_JOINT2", "R_ARM_JOINT3", "R_ARM_JOINT4", "R_ARM_JOINT5", "R_ARM_JOINT6", "R_ARM_JOINT7",
    //"R_ARM_JOINT8",
    "L_ARM_JOINT1", "L_ARM_JOINT2", "L_ARM_JOINT3", "L_ARM_JOINT4", "L_ARM_JOINT5", "L_ARM_JOINT6", "L_ARM_JOINT7",
    //"L_ARM_JOINT8",
    "NECK_JOINT0",  "NECK_JOINT1",  "NECK_JOINT2",  "R_LEG_JOINT1", "R_LEG_JOINT2", "R_LEG_JOINT3", "R_LEG_JOINT4",
    "R_LEG_JOINT5", "R_LEG_JOINT6",
    //"R_LEG_JOINT7",
    "L_LEG_JOINT1", "L_LEG_JOINT2", "L_LEG_JOINT3", "L_LEG_JOINT4", "L_LEG_JOINT5", "L_LEG_JOINT6"
    //"L_LEG_JOINT7",
};

const int kModel01BaseLink = SpineMid;

// Keep consistency with kJointName
const int kJointDoF = 29;

template < class T >
int calcForModel01(const tms_msg_ss::Skeleton &in, Eigen::Matrix< T, 3, 1 > &position, Eigen::Quaternion< T > &rotation,
                   std::map< std::string, T > &out)
{
  if (in.user_id < 0)
  {
    // Set data to disappear from environment
    position = Eigen::Matrix< T, 3, 1 >(0.0, 0.0, 0.0);
    rotation = Eigen::Quaternion< T >(1.0, 0.0, 0.0, 0.0);
    for (int j = 0; j < kJointDoF; j++)
    {
      out[kJointName[j]] = 0.0;
    }
    return -1;
  }
  else
  {
    std::vector< Eigen::Matrix< T, 3, 1 > > j;
    j.resize(JOINT_NUM);
    for (int i = 0; i < JOINT_NUM; i++)
    {
      j[i] = Eigen::Matrix< T, 3, 1 >(in.position[i].x, in.position[i].y, in.position[i].z);
    }
    // Calculation of position.
    position = Eigen::Matrix< T, 3, 1 >(j[SpineMid][0], j[SpineMid][1], j[SpineMid][2]);

    // Calculation of rotation.
    Eigen::Matrix< T, 3, 1 > x, y, z;
    y = ((j[SpineMid] - j[SpineBase]).cross(j[HipRight] - j[HipLeft])).normalized();
    z = (j[SpineMid] - j[SpineBase]).normalized();
    x = (y.cross(z)).normalized();
    Eigen::Matrix< T, 3, 3 > mat;
    mat << x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2];

    rotation = mat;

    // Avoiding Error : I don't know why this error happens.
    if (isnan(rotation.x()))
    {
      // It happens when gets same j[SpinMid], j[SpineBase], j[HipRight] and j[HipLeft]
      rotation = Eigen::Quaternion< T >(1.0, 0.0, 0.0, 0.0);
    }

    // Initialize
    for (int i = 0; i < kJointDoF; i++)
    {
      out[kJointName[i]] = 0.0;
    }

    // Calculation of joint angles.
    Eigen::Matrix< T, 3, 1 > vec;
    Eigen::Quaternion< T > rot[2];
    // -*- Invalid calculation -*-
    // if (1)//req.skeleton.confidence[SpineMid] == 2 &&
    //    //req.skeleton.confidence[SpineBase] == 2 &&
    //    //req.skeleton.confidence[SpineShoulder] == 2 &&
    //    //req.skeleton.confidence[ShoulderLeft] == 2 &&
    //    //req.skeleton.confidence[ShoulderRight] == 2)
    //{
    //  // 0:WAIST_JOINT1
    //  z = (j[SpineMid]-j[SpineBase]).normalized();
    //  x = ((j[HipLeft]-j[HipRight]).cross(j[SpineMid]-j[SpineBase])).normalized();
    //  y = (j[HipLeft]-j[HipRight]).normalized();
    //  vec = (j[SpineShoulder]-j[SpineMid]).cross(j[ShoulderLeft]-j[ShoulderRight]);
    //  out[kJointName[0]] = atan2(y.dot(vec),x.dot(vec));
    //  rot[0] = Eigen::AngleAxis<T>(out[kJointName[0]],x.cross(y));
    //  // 1:WAIST_JOINT2
    //  x = rot[0] * (j[SpineMid]-j[SpineBase]).normalized();
    //  y = rot[0] * ((j[SpineMid]-j[SpineBase]).cross(j[HipRight]-j[HipLeft])).normalized();
    //  vec = j[SpineShoulder]-j[SpineMid];
    //  out[kJointName[1]] = atan2(y.dot(vec),x.dot(vec));
    //}
    // Right Shoulder
    if (in.confidence[SpineShoulder] == 2 && in.confidence[ShoulderRight] == 2 && in.confidence[SpineMid] == 2 &&
        in.confidence[ElbowRight] == 2)
    {
      // 2:R_ARM_JOINT1
      x = (j[SpineMid] - j[SpineShoulder]).normalized();
      y = ((j[SpineShoulder] - j[ShoulderRight]).cross(j[SpineMid] - j[SpineShoulder])).normalized();
      vec = j[ElbowRight] - j[ShoulderRight];
      out["R_ARM_JOINT1"] = atan2(y.dot(vec), x.dot(vec));
      rot[0] = Eigen::AngleAxis< T >(out["R_ARM_JOINT1"], x.cross(y));
      // 3:R_ARM_JOINT2
      x = rot[0] * (j[SpineMid] - j[SpineShoulder]).normalized();
      y = rot[0] * (j[SpineShoulder] - j[ShoulderRight]).normalized();
      vec = j[ElbowRight] - j[ShoulderRight];
      out["R_ARM_JOINT2"] = atan2(y.dot(vec), x.dot(vec));
      rot[1] = Eigen::AngleAxis< T >(out["R_ARM_JOINT2"], x.cross(y));
      // 4:R_ARM_JOINT3
      x = rot[1] * rot[0] * (j[ShoulderRight] - j[SpineShoulder]).normalized();
      y = rot[1] * rot[0] * ((j[ShoulderRight] - j[SpineShoulder]).cross(j[SpineMid] - j[SpineShoulder])).normalized();
      vec = (j[WristRight] - j[ElbowRight]).cross(j[ShoulderRight] - j[ElbowRight]);
      out["R_ARM_JOINT3"] = atan2(y.dot(vec), x.dot(vec));
    }
    // Right Elbow
    if (in.confidence[ShoulderRight] == 2 && in.confidence[ElbowRight] == 2 && in.confidence[WristRight] == 2)
    {
      // 5:R_ARM_JOINT4
      z = (j[ShoulderRight] - j[ElbowRight]).cross(j[WristRight] - j[ElbowRight]);
      y = (z.cross(j[ElbowRight] - j[ShoulderRight])).normalized();
      x = (j[ElbowRight] - j[ShoulderRight]).normalized();
      vec = j[WristRight] - j[ElbowRight];
      out["R_ARM_JOINT4"] = atan2(y.dot(vec), x.dot(vec));
    }
    // Left Shoulder
    if (in.confidence[SpineShoulder] == 2 && in.confidence[ShoulderLeft] == 2 && in.confidence[SpineMid] == 2 &&
        in.confidence[ElbowLeft] == 2)
    {
      // 10:L_ARM_JOINT1
      z = j[ShoulderLeft] - j[SpineShoulder];
      x = (j[SpineMid] - j[SpineShoulder]).normalized();
      y = (z.cross(j[SpineMid] - j[SpineShoulder])).normalized();
      vec = j[ElbowLeft] - j[ShoulderLeft];
      out["L_ARM_JOINT1"] = atan2(y.dot(vec), x.dot(vec));
      rot[0] = Eigen::AngleAxis< T >(out["L_ARM_JOINT1"], x.cross(y));
      // 11:L_ARM_JOINT2
      x = rot[0] * (j[SpineMid] - j[SpineShoulder]).normalized();
      y = rot[0] * (j[ShoulderLeft] - j[SpineShoulder]).normalized();
      vec = j[ElbowLeft] - j[ShoulderLeft];
      out["L_ARM_JOINT2"] = atan2(y.dot(vec), x.dot(vec));
      rot[1] = Eigen::AngleAxis< T >(out["L_ARM_JOINT2"], x.cross(y));
      // 12:L_ARM_JOINT3
      x = rot[1] * rot[0] * (j[ShoulderLeft] - j[SpineShoulder]).normalized();
      y = rot[1] * rot[0] * ((j[ShoulderLeft] - j[SpineShoulder]).cross(j[SpineMid] - j[SpineShoulder])).normalized();
      vec = (j[ShoulderLeft] - j[ElbowLeft]).cross(j[WristLeft] - j[ElbowLeft]);
      out["L_ARM_JOINT3"] = atan2(y.dot(vec), x.dot(vec));
    }
    // Left Elbow
    if (in.confidence[SpineMid] == 2 && in.confidence[ElbowLeft] == 2 && in.confidence[WristLeft] == 2)
    {
      // 13:L_ARM_JOINT4
      z = (j[ShoulderLeft] - j[ElbowLeft]).cross(j[WristLeft] - j[ElbowLeft]);
      x = (j[ElbowLeft] - j[ShoulderLeft]).normalized();
      y = (z.cross(j[ElbowLeft] - j[ShoulderLeft])).normalized();
      vec = j[WristLeft] - j[ElbowLeft];
      out["L_ARM_JOINT4"] = atan2(y.dot(vec), x.dot(vec));
    }
    // Right Hip
    if (in.confidence[SpineBase] == 2 && in.confidence[SpineMid] == 2 && in.confidence[HipRight] == 2 &&
        in.confidence[HipLeft] == 2 && in.confidence[ElbowRight] == 2)
    {
      // 20:R_LEG_JOINT1
      z = j[SpineBase] - j[HipRight];
      x = (j[SpineBase] - j[SpineMid]).normalized();
      y = (z.cross(j[SpineBase] - j[SpineMid])).normalized();
      vec = j[KneeRight] - j[HipRight];
      out["R_LEG_JOINT1"] = atan2(y.dot(vec), x.dot(vec));
      rot[0] = Eigen::AngleAxis< T >(out["R_LEG_JOINT1"], x.cross(y));
      // 21:R_LEG_JOINT2
      z = rot[0] * (j[HipRight] - j[HipLeft]).cross(j[SpineBase] - j[SpineMid]);
      x = rot[0] * (j[SpineBase] - j[SpineMid]).normalized();
      y = (z.cross(j[SpineBase] - j[SpineMid])).normalized();
      vec = j[KneeRight] - j[HipRight];
      out["R_LEG_JOINT2"] = atan2(y.dot(vec), x.dot(vec));
      rot[1] = Eigen::AngleAxis< T >(out["R_LEG_JOINT2"], x.cross(y));
      // 22:R_LEG_JOINT3
      x = rot[1] * rot[0] * (j[HipRight] - j[HipLeft]).normalized();
      y = rot[1] * rot[0] * ((j[HipRight] - j[HipLeft]).cross(j[SpineBase] - j[SpineMid])).normalized();
      vec = (j[HipRight] - j[KneeRight]).cross(j[AnkleRight] - j[KneeRight]);
      out["R_LEG_JOINT3"] = atan2(y.dot(vec), x.dot(vec));
    }
    // Right Knee
    if (in.confidence[HipRight] == 2 && in.confidence[ElbowRight] == 2 && in.confidence[AnkleRight] == 2)
    {
      // 23:R_LEG_JOINT4
      z = (j[AnkleRight] - j[KneeRight]).cross(j[HipRight] - j[KneeRight]);
      x = (j[KneeRight] - j[HipRight]).normalized();
      y = (z.cross(j[KneeRight] - j[HipRight])).normalized();
      vec = j[AnkleRight] - j[KneeRight];
      out["R_LEG_JOINT4"] = atan2(y.dot(vec), x.dot(vec));
    }
    // Left Hip
    if (in.confidence[SpineBase] == 2 && in.confidence[SpineMid] == 2 && in.confidence[HipRight] == 2 &&
        in.confidence[HipLeft] == 2 && in.confidence[ElbowLeft] == 2)
    {
      // 27:L_LEG_JOINT1
      z = j[HipLeft] - j[SpineBase];
      x = (j[SpineBase] - j[SpineMid]).normalized();
      y = (z.cross(j[SpineBase] - j[SpineMid])).normalized();
      vec = j[KneeLeft] - j[HipLeft];
      out["L_LEG_JOINT1"] = atan2(y.dot(vec), x.dot(vec));
      rot[0] = Eigen::AngleAxis< T >(out["L_LEG_JOINT1"], x.cross(y));
      // 28:L_LEG_JOINT2
      z = rot[0] * (j[HipRight] - j[HipLeft]).cross(j[SpineBase] - j[SpineMid]);
      x = rot[0] * (j[SpineBase] - j[SpineMid]).normalized();
      y = (z.cross(j[SpineBase] - j[SpineMid])).normalized();
      vec = j[KneeLeft] - j[HipLeft];
      out["L_LEG_JOINT2"] = atan2(y.dot(vec), x.dot(vec));
      rot[1] = Eigen::AngleAxis< T >(out["L_LEG_JOINT2"], x.cross(y));
      // 29:L_LEG_JOINT3
      z = j[HipLeft] - j[KneeLeft];
      x = rot[1] * rot[0] * (j[HipRight] - j[HipLeft]).normalized();
      y = (z.cross(x)).normalized();
      vec = (j[HipLeft] - j[KneeLeft]).cross(j[AnkleLeft] - j[KneeLeft]);
      out["L_LEG_JOINT3"] = atan2(y.dot(vec), x.dot(vec));
    }
    // Left Knee
    if (in.confidence[HipLeft] == 2 && in.confidence[ElbowLeft] == 2 && in.confidence[AnkleLeft] == 2)
    {
      // 30:L_LEG_JOINT4
      z = (j[AnkleLeft] - j[KneeLeft]).cross(j[HipLeft] - j[KneeLeft]);
      x = (j[KneeLeft] - j[HipLeft]).normalized();
      y = (z.cross(j[KneeLeft] - j[HipLeft])).normalized();
      vec = j[AnkleLeft] - j[KneeLeft];
      out["L_LEG_JOINT4"] = atan2(y.dot(vec), x.dot(vec));
    }
    for (int i = 0; i < kJointDoF; i++)
    {
      out[kJointName[i]] = (isnan(out[kJointName[i]]) ? 0.0 : out[kJointName[i]]);
    }
  }

  return 0;
}

#endif  //_CALC_JOINT_ANGLES_H_
