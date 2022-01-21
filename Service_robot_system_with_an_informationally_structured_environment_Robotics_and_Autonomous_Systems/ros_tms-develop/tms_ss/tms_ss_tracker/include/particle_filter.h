#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "condens/condens.hpp"

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

class CPF
{
public:
  CPF();
  virtual ~CPF();

public:
  void update();
  void initialize(int area[2]);
  void clear();
  void SetTarget(double t[2]);
  void GetTarget(double t[2]);
  double state[2];
  void SetID(int ID);
  int GetID();
  int GetCnt();

protected:
  CvConDensation* particle_filter;
  double sigma_;
  int Dimension;
  int Num_of_particles;
  double Area[4];
  double MaxVel;
  double target[2];
  int m_ID;
  int m_cnt;

  double likelihood(double p[2]);
};

#endif  // PARTICLE_FILTER_H
