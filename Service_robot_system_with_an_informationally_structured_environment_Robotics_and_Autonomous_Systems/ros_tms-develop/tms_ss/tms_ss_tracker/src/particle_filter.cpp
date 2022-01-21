#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "particle_filter.h"

CPF::CPF()
{
  sigma_ = 500.0;
  Dimension = 4;
  Num_of_particles = 100;
  Area[0] = Area[1] = Area[2] = Area[3] = 0;
  MaxVel = 100.0;
  state[0] = state[1] = 0;
  m_ID = -1;
  particle_filter = NULL;
  m_cnt = 0;
}

CPF::~CPF()
{
}

void CPF::initialize(int area[2])
{
  CvMat* lower_bound;
  CvMat* upper_bound;

  if (particle_filter)
  {
    cvReleaseConDensation(&particle_filter);
  }
  particle_filter = cvCreateConDensation(Dimension, 2, Num_of_particles);

  lower_bound = cvCreateMat(4, 1, CV_32FC1);
  upper_bound = cvCreateMat(4, 1, CV_32FC1);

  Area[0] = -area[0] / 2.0;
  Area[1] = -area[1] / 2.0;
  Area[2] = area[0] / 2.0;
  Area[3] = area[1] / 2.0;
  cvmSet(lower_bound, 0, 0, Area[0]);
  cvmSet(lower_bound, 1, 0, Area[1]);
  cvmSet(lower_bound, 2, 0, -MaxVel);
  cvmSet(lower_bound, 3, 0, -MaxVel);
  cvmSet(upper_bound, 0, 0, Area[2]);
  cvmSet(upper_bound, 1, 0, Area[3]);
  cvmSet(upper_bound, 2, 0, MaxVel);
  cvmSet(upper_bound, 3, 0, MaxVel);

  cvConDensInitSampleSet(particle_filter, lower_bound, upper_bound);

  cvReleaseMat(&lower_bound);
  cvReleaseMat(&upper_bound);

  const double matrix_elements[] = {
      1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
  };

  size_t n = sizeof(matrix_elements) / sizeof(matrix_elements[0]);

  for (size_t i = 0; i < n; ++i)
  {
    particle_filter->DynamMatr[i] = matrix_elements[i];
  }

  cvRandInit(&(particle_filter->RandS[0]), -50, 50, (int)cvGetTickCount(), CV_RAND_UNI);
  cvRandInit(&(particle_filter->RandS[1]), -50, 50, (int)cvGetTickCount(), CV_RAND_UNI);
  cvRandInit(&(particle_filter->RandS[2]), -10, 10, (int)cvGetTickCount(), CV_RAND_UNI);
  cvRandInit(&(particle_filter->RandS[3]), -10, 10, (int)cvGetTickCount(), CV_RAND_UNI);

  m_cnt = 0;
}

void CPF::clear()
{
  if (particle_filter)
  {
    cvReleaseConDensation(&particle_filter);
  }
}

void CPF::SetTarget(double t[2])
{
  target[0] = t[0];
  target[1] = t[1];
}

void CPF::GetTarget(double t[2])
{
  t[0] = target[0];
  t[1] = target[1];
}

void CPF::update()
{
  for (int i = 0; i < Num_of_particles; ++i)
  {
    double p[2];
    p[0] = particle_filter->flSamples[i][0];
    p[1] = particle_filter->flSamples[i][1];

    if ((p[0] < Area[0]) || (p[0] >= Area[2]) || (p[1] < Area[1]) || (p[1] >= Area[3]))
    {
      particle_filter->flConfidence[i] = 0.0;
    }
    else
    {
      particle_filter->flConfidence[i] = likelihood(p);
    }
  }

  cvConDensUpdateByTime(particle_filter);

  state[0] = particle_filter->State[0];
  state[1] = particle_filter->State[1];

  m_cnt++;
}

double CPF::likelihood(double p[2])
{
  double dist = sqrt(pow(p[0] - target[0], 2) + pow(p[1] - target[1], 2));
  return 1.0 / (sqrt(2.0 * M_PI) * sigma_) * exp(-pow(dist, 2.0) / (2.0 * pow(sigma_, 2.0)));
}

void CPF::SetID(int ID)
{
  m_ID = ID;
}

int CPF::GetID()
{
  return m_ID;
}

int CPF::GetCnt()
{
  return m_cnt;
}
