#ifndef LASER_H
#define LASER_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_LRF_RANGE 55.00
#define MIN_DIFF_DIST 0.50
#define MIN_OBJ_PROB 0.01
#define N_RING 100

typedef struct
{
  double x;
  double y;
} pos_t;

typedef struct
{
  int n;
  double range;
} range_t;

typedef struct
{
  int n;
  double range;
  int length;
} cluster_t;

using namespace std;

class CLaser
{
public:
  CLaser();
  virtual ~CLaser();

public:
  static const int m_cnMaxConnect = MAX_CONNECT;
  static const int m_cnMaxDataSizeLRF = MAX_DATA_SIZE_LRF;
  static const int m_cnMaxTrackingObject = MAX_TRACKING_OBJECT;
  static const int m_cnMaxParticleNum = MAX_PARTICLE_NUM + MAX_PARTICLE_NUM_MCMC;
  static const int m_cnTrackingHistory = TRACKING_HISTORY;
  bool m_bNodeActive[m_cnMaxConnect];
  int m_nConnectNum;

  std::vector< std::vector< double > > m_LRFData;
  std::vector< std::vector< double > > m_BackLRFData;
  std::vector< std::vector< range_t > > m_DiffLRFData;
  std::vector< std::vector< pos_t > > m_LRFPoints;
  std::vector< std::vector< cluster_t > > m_LRFClsData;
  std::vector< std::vector< pos_t > > m_LRFClsPoints;

  CvMat* m_LRFPos[m_cnMaxConnect][m_cnMaxDataSizeLRF];
  CvMat* m_LRFClsPos[m_cnMaxConnect][m_cnMaxDataSizeLRF];

  int m_nStep[m_cnMaxConnect];
  double m_StartAngle[m_cnMaxConnect];
  double m_DivAngle[m_cnMaxConnect];
  LRFParam m_LRFParam[m_cnMaxConnect];

  CvMat* m_Particle[m_cnMaxParticleNum];
  double m_ParticleLikelihood[m_cnMaxParticleNum];
  int m_ParticleLabel[m_cnMaxParticleNum];
  int m_nParticleNum;

  CTarget* m_pTarget[m_cnMaxTrackingObject];
  int m_Target_cnt[m_cnMaxTrackingObject];

  int m_TargetNum;
  double m_Grid[(STAGE_X / GRID_DISTANCE) * (STAGE_Y / GRID_DISTANCE)];

  int m_n_ring;
  std::vector< std::vector< std::vector< double > > > m_BackLRFDataRing;
  std::vector< std::vector< double > > m_BackLRFDataAve;
  std::vector< std::vector< double > > m_BackLRFDataVar;

public:
  bool Init();

  int GetLRFParam();
  int GetDiffLRFData();
  int GetBackLRFData();
  int GetDiffLRFCluster(int n);
  bool m_bResetBackRangeData;
  int GetBackLRFDataGaussian();
  int GetDiffLRFDataGaussian();

  int IsFrontFData(double currentdata, double backdata, double min_dist);
  int IsFrontFData(double currentdata, double backdata_ave, double backdata_var, double min_prob);

  int m_diff_dist;
  double m_obj_prob;
  int m_ring;
};

#endif  // LASER_H
