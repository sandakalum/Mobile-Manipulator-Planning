//------------------------------------------------------------------------------
// @file   : qnode.hpp
// @brief  : rosnode function
// @author : Watanabe Yuuta
// @version: Ver0.1.1
// @date   : 2015.4.1
//------------------------------------------------------------------------------
#ifndef NODE_HPP
#define NODE_HPP

#ifndef Q_MOC_RUN

//------------------------------------------------------------------------------
#include <QStringListModel>
#include <QThread>

#include <string>

#include <ros/ros.h>

//------------------------------------------------------------------------------
class QNode : public QThread
{
  Q_OBJECT

public:
  int init_argc;
  char** init_argv;
  const std::string node_name;

  QNode(int argc, char** argv, const std::string& name);
  virtual ~QNode();
};

#endif

#endif  // NODE_HPP

//------------------------------------------------------------------------------
