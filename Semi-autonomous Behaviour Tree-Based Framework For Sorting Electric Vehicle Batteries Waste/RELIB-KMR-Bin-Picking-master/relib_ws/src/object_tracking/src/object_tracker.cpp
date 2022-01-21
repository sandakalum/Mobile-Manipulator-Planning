//  BOOST
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

//  C++
#include <stdexcept>

//  ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <kuka_msgs/PoseExtraction.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

//Visp ROS
#include "libobject_tracking/tracker.hh"

// class ObjectTracker{

// private:
//   ros::NodeHandle nh_;
//   ros::NodeHandle nh_private_;

//   ros::ServiceServer object_tracker_srv;

//   boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> object_pose;

//   volatile bool& exiting;

// public:

//   ObjectTracker(ros::NodeHandle* public_nh, ros::NodeHandle* private_nh, volatile bool& exting_): nh_(*public_nh), nh_private_(*private_nh), exiting(exting_) {

//     object_tracker_srv = nh_.advertiseService("object_tracking", &ObjectTracker::objectTrackerCallback, this);
//   }
//   ~ObjectTracker(){

//   }

//   bool objectTrackerCallback(kuka_msgs::PoseExtraction::Request& req, kuka_msgs::PoseExtraction::Response& res);
// };

// bool ObjectTracker::objectTrackerCallback(kuka_msgs::PoseExtraction::Request& req, kuka_msgs::PoseExtraction::Response& res){

//   return true;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_tracker");
    ros::NodeHandle nh_public;
    ros::NodeHandle nh_private("~");
    volatile bool exiting_ = false;

    visp_tracker::Tracker tracker(nh_public, nh_private, exiting_, 5u);
    tracker.spin();
    
    ROS_INFO("Object Tracker Service Ready!");

    return 0;
}
