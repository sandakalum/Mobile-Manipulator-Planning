//------------------------------------------------------------------------------
// Ifdefs
//------------------------------------------------------------------------------
#ifndef QNODE_HPP
#define QNODE_HPP

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <QThread>
#include <QStringListModel>
#include <qapplication.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <string>

#include <tms_msg_ss/fss_pre_data.h>
#include <tms_msg_ss/fss_tf_data.h>

//------------------------------------------------------------------------------
// Structs
//------------------------------------------------------------------------------
typedef struct
{
  float fX1, fY1;
  float fX2, fY2;
} COORDINATE;

typedef struct
{
  float fDistance;
  float fIntensity;
  float fIntrinsicIntensity;
} LRF;

typedef struct
{
  bool bIsReflect;
  bool bIsForwardPoint;
  float fDistance;
  float fIntensity;
  float fIntrinsicIntensity;
  float fAcuteAngle;
  COORDINATE stCoordinate;
} LRF_DATA;

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
namespace fss_graph
{
//------------------------------------------------------------------------------
// Class
//------------------------------------------------------------------------------
class QNode : public QThread
{
  Q_OBJECT

public:
  LRF_DATA m_stTmepData[721];
  LRF m_stMaxErrorData[721];
  LRF m_stMinErrorData[721];

  QNode(int argc, char** argv);
  virtual ~QNode();

  bool init();
  void run();
  void shutdown();

Q_SIGNALS:
  void rosShutdown();

private:
  void chatterCallback(const tms_msg_ss::fss_tf_data::ConstPtr& msg);

  int init_argc;
  char** init_argv;

  ros::Subscriber fss_graph_subscriber;
  ros::Publisher fss_graph_publisher;

  std::vector< tms_msg_ss::fss_tf_data > m_vstTfData;
};

//------------------------------------------------------------------------------
}  // namespace fss_graph
#endif  // QNODE_HPP
