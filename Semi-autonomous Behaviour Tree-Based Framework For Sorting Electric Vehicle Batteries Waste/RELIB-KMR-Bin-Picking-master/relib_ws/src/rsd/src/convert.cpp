#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
    
int main(int argc, char** argv){

    ros::init(argc, argv, "converter");
    
    ros::NodeHandle nh;  
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf::Vector3 basis(atof(argv[1]),atof(argv[2]),atof(argv[3]));
   
    ros::Rate rate(10.0);
    geometry_msgs::TransformStamped transformStamped;

    while (nh.ok()){
        try{
              transformStamped = tfBuffer.lookupTransform("base_footprint", "camera_depth_optical_frame",
                                        ros::Time(0));
            break;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        rate.sleep();
    }
    tf::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w
    );
    tf::Matrix3x3 m(q);
    tf::Vector3 translation(
        transformStamped.transform.translation.x,
        transformStamped.transform.translation.y,
        transformStamped.transform.translation.z
    );

    tf::Transform camera2base(m,translation);
    tf::Vector3 world;
    world = camera2base*basis;

    ROS_INFO_STREAM("X: "<<world.getX()<<" Y: "<<world.getY()<<" Z: "<<world.getZ());

   return 0;
  };