#ifndef MULTIPLE_PARTICLE_FILTER_H
#define MULTIPLE_PARTICLE_FILTER_H

class CMultipleParticleFilter
{
public:
  CMultipleParticleFilter();
  virtual ~CMultipleParticleFilter();

  void update(CLaser *Laser);

public:
  std::vector< CPF > m_ParticleFilter;
  CLaser *m_pLaser;

protected:
  int m_max_ID;
  double m_min_distance;
  int m_ID;
};

#endif  // MULTIPLE_PARTICLE_FILTER_H
