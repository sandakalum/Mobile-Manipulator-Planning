#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <kuka_msgs/ArmPose.h>
#include <kuka_msgs/ObjectPose.h>

#define PI 3.141516

static const std::string PLANNING_GROUP = "lbr_arm";
static const std::string EYE_POSE = "eye";
static const std::string TOWARDS_TABLE = "towards_table";
static const std::string BACK = "back";
static const std::string HOME = "home table";
static const std::string EXTENDED = "extended";
static const std::string FRONT_1 = "front_1";
static const std::string FRONT_2 = "front_2";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "services_tester");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<kuka_msgs::ObjectPose>("grasp");
    kuka_msgs::ObjectPose srv;

    float x, y, z, R, P, Y = 0;
    std::string base_frame = "base_footprint";

    if (argc > 1)
    {

        if (argv[1] != NULL)
            x = std::atof(argv[1]);
        if (argv[2] != NULL)
            y = std::atof(argv[2]);
        if (argv[3] != NULL)
            z = std::atof(argv[3]);
        if (argv[4] != NULL)
            R = std::atof(argv[4]) * 180 / M_PI;
        if (argv[5] != NULL)
            P = std::atof(argv[5]) * 180 / M_PI;
        if (argv[6] != NULL)
            Y = std::atof(argv[6]) * 180 / M_PI;
    }
    else
    {
        ROS_INFO("Enter params  x y z R P Y");
        return 0;
    }
    srv.request.obj_pose.header.frame_id = base_frame;
    srv.request.obj_pose.pose.position.x = x;
    srv.request.obj_pose.pose.position.y = y;
    srv.request.obj_pose.pose.position.z = z;

    tf2::Quaternion orientation;
    orientation.setRPY(R, P, Y);
    srv.request.obj_pose.pose.orientation = tf2::toMsg(orientation);

    if (client.call(srv))
    {
        ROS_INFO("Response: %u",srv.response.status);
        // ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    // ros::AsyncSpinner spinner(2);
    // spinner.start();
    // ros::waitForShutdown();
    //ros::spin();
    return 0;
}