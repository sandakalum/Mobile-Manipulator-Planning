#ifndef NODE_HPP
#define NODE_HPP

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

    QNode(int argc, char** argv, const std::string &name );
    virtual ~QNode();

};

#endif // NODE_HPP

//------------------------------------------------------------------------------
