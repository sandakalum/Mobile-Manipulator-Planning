#pragma once

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

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

  CTarget* m_pTarget[m_cnMaxTrackingObject];
  int m_Target_cnt[m_cnMaxTrackingObject];

  int m_n_ring;
  std::vector< std::vector< std::vector< double > > > m_BackLRFDataRing;
  std::vector< std::vector< double > > m_BackLRFDataAve;
  std::vector< std::vector< double > > m_BackLRFDataVar;

public:
  bool Init();  // OnNewDocument()

  int GetLRFParam();
  int GetDiffLRFData();
  int GetBackLRFData();
  int GetDiffLRFCluster(int n);
  bool m_bResetBackRangeData;
  int GetBackLRFDataGaussian();
  int GetDiffLRFDataGaussian();
  int UpdateBackLRFDataGaussian();

  int IsFrontFData(double currentdata, double backdata, double min_dist);
  int IsFrontFData(double currentdata, double backdata_ave, double backdata_var, double min_prob);

  bool ReadBackgroundData();
  bool WriteBackgroundData();

  int m_diff_dist;
  double m_obj_prob;
  int m_ring;
};
