//  BOOST
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

//  STL C++
#include <stdexcept>

//  ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <kuka_msgs/PoseExtraction.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>


#include <std_srvs/Empty.h>


//Visp ROS
#include "libobject_tracking/tracker-client.hh"


class ObjectTrackerClient{

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  tf2_ros::Buffer& tfBuffer;

  ros::ServiceServer object_tracker_srv;
  // visp_tracker::TrackerClient tracker_client;
  boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> object_pose;

  volatile bool& exiting;


public:

  ObjectTrackerClient(ros::NodeHandle* public_nh, ros::NodeHandle* private_nh, tf2_ros::Buffer& _tf2_Buffer, volatile bool& exting_): nh_(*public_nh), nh_private_(*private_nh), 
                                                                                  tfBuffer(_tf2_Buffer), exiting(exting_) {
                  
    object_tracker_srv = nh_.advertiseService("object_tracking", &ObjectTrackerClient::objectTrackerCallback, this);
  }
  ~ObjectTrackerClient(){

  }

  bool objectTrackerCallback(kuka_msgs::PoseExtraction::Request& req, kuka_msgs::PoseExtraction::Response& res);
};

bool ObjectTrackerClient::objectTrackerCallback(kuka_msgs::PoseExtraction::Request& req, kuka_msgs::PoseExtraction::Response& res){

  boost::shared_ptr<visp_tracker::TrackerClient> trackerClient_(new visp_tracker::TrackerClient(nh_, nh_private_, exiting, req.object, 5u));
  trackerClient_->spin();

  ROS_INFO("Waiting for object position covariance");
  object_pose = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("object_position_covariance", nh_, ros::Duration(5.0));
  if (object_pose != NULL)
  {

    ROS_INFO("Object Pose received: frame_id: %s, at (%f, %f, %f)",object_pose->header.frame_id.c_str(), object_pose->pose.pose.position.x, 
                                                                 object_pose->pose.pose.position.y, object_pose->pose.pose.position.z);
    ros::Rate rate(10.0);
    geometry_msgs::TransformStamped transformStamped;
    tf::Vector3 object_translation;
    object_translation.setValue(object_pose->pose.pose.position.x, object_pose->pose.pose.position.y, object_pose->pose.pose.position.z);
    while (true)
    {
      try
      {
        transformStamped = tfBuffer.lookupTransform("base_footprint", "camera_depth_optical_frame",
                                                    ros::Time(0));
        break;
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
      rate.sleep();
    }
    /////////////////
    // CONVERT QUATERNION TO RPY FROM TRANSFORM_STAMPED

    tf::Quaternion q(
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w);
    tf::Matrix3x3 m(q);
    tf::Vector3 translation(
        transformStamped.transform.translation.x,
        transformStamped.transform.translation.y,
        transformStamped.transform.translation.z);

    // Compute object pose respective to the lbr reference frame (base_footprint)
    tf::Transform camera2baseFootprint(m, translation);
  
    tf::Vector3 model2baseFootprint = camera2baseFootprint * object_translation;

    ROS_INFO_STREAM("baseFootprint x: " << model2baseFootprint.getX() << " Y: " << model2baseFootprint.getY() << " Z: " << model2baseFootprint.getZ());

    res.poseStamped.header.frame_id = "base_footprint";
    res.poseStamped.pose.position.x = model2baseFootprint.getX();
    res.poseStamped.pose.position.y = model2baseFootprint.getY();
    res.poseStamped.pose.position.z = model2baseFootprint.getZ();
    res.state = "succes";
    res.extractionSuccesfull = true;                                            
  }else{
    ROS_INFO("Object pose waiting timeout! Aborting...");
    res.extractionSuccesfull = false;
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_tracker_client");
  ros::NodeHandle nh_public;
  ros::NodeHandle nh_private("~");
  tf2_ros::Buffer tfBuffer2;
  tf2_ros::TransformListener tfListener(tfBuffer2);
  volatile bool exiting_ = false;

  ObjectTrackerClient tracker_client_custom(&nh_public, &nh_private, tfBuffer2, exiting_);

  ROS_INFO("Object Tracker Client Service Ready!");

  ros::spin();

  return 0;
}
