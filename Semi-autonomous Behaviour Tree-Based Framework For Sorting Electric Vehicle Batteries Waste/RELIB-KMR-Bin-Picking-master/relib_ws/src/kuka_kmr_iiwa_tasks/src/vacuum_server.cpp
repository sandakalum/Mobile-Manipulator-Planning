
// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#define PI 3.141516
// #define N_SERVERS 5

int N_SERVERS = 1;
static const int gripper_depth = 0.11788;
std_srvs::Empty srv;

std::vector<ros::ServiceClient> on_servers_clients;
std::vector<ros::ServiceClient> off_servers_clients;

void vaccum(bool action, std::vector<ros::ServiceClient>& on_server , std::vector<ros::ServiceClient>& off_server, std_srvs::Empty& _srv){
    if(action){
        for(int j=0; j<N_SERVERS;j++){
            if(on_server[j].call(_srv) ){
                ROS_INFO_STREAM("Vaccum "<< j<<" ON");
            }
            else{
                 ROS_ERROR("For some reason the vaccum %i was not turned ON",j);
            }
        }
    }else{
        for(int j=0; j<N_SERVERS;j++){
            if(off_server[j].call(_srv) ){
                ROS_INFO_STREAM("Vaccum "<< j<<" OFF");
            }
            else{
                ROS_ERROR("For some reason the vaccum %i was not turned OFF",j);
            }
        }
    }
}



bool vg10_callback (std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp){

    if(req.data){
        vaccum(1,on_servers_clients, off_servers_clients, srv);
    }else{
        vaccum(0,on_servers_clients, off_servers_clients, srv);
    }
    return true;
} 



int main(int argc, char** argv)
{

    ros::init(argc,argv,"vacuum_server");

    ros::NodeHandle nh;  
    N_SERVERS = atoi(argv[1]);

    for(int i = 0; i<N_SERVERS; i++){
        on_servers_clients.push_back(nh.serviceClient<std_srvs::Empty>("/vaccum_" + std::to_string(i) + "/on"));
    }
    for(int i = 0; i<N_SERVERS; i++){
        off_servers_clients.push_back(nh.serviceClient<std_srvs::Empty>("/vaccum_" + std::to_string(i) + "/off"));
    }

    ros::ServiceServer vg10_switch_server = nh.advertiseService("vg10_actioner", vg10_callback);

    ROS_INFO("Vaccum server ready");
    ros::spin();


    
    return 0;
}
