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

#include <string>

#include <tms_msg_ss/bas_behavior_data.h>

//------------------------------------------------------------------------------
// Structs
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
namespace bas_graph
{
//------------------------------------------------------------------------------
// Class
//------------------------------------------------------------------------------
class QNode : public QThread
{
  Q_OBJECT

public:
  QNode(int argc, char** argv);
  virtual ~QNode();

  bool init();
  void run();
  void shutdown();

  tms_msg_ss::bas_behavior_data m_bas_behavior_data;

Q_SIGNALS:
  void rosShutdown();

private:
  void behaviorCallback(const tms_msg_ss::bas_behavior_data::ConstPtr& msg);

  int init_argc;
  char** init_argv;

  ros::Subscriber bas_graph_subscriber;
  ros::Publisher bas_graph_publisher;
};

//------------------------------------------------------------------------------
}  // namespace bas_graph
#endif  // QNODE_HPP

//------------------------------------------------------------------------------
