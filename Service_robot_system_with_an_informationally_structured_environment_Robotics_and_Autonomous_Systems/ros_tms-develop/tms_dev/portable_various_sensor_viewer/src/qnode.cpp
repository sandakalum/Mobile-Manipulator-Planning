
#include <ros/ros.h>
#include <ros/network.h>

#include <string>
#include <sstream>

#include "qnode.hpp"

//------------------------------------------------------------------------------
// Implementation
//------------------------------------------------------------------------------
QNode::QNode(int argc, char** argv, const std::string &name ) :
    init_argc(argc),
    init_argv(argv),
    node_name(name)
{

}

//------------------------------------------------------------------------------
QNode::~QNode()
{

}
