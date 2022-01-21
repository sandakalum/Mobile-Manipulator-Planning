// include for ROS
#include "ros/ros.h"
#include <unistd.h>
#include <boost/date_time/local_time/local_time.hpp>
#include <tms_msg_ss/ods_change_detection.h>
#include <iostream>
#include <stdio.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

ros::ServiceServer service;
ros::ServiceClient commander_to_ods_change_detection;

int main(int argc, char **argv)
{
  printf("init\n");
  ros::init(argc, argv, "ods_changedt_trigger");
  ros::NodeHandle n;
  tms_msg_ss::ods_change_detection srv;
  commander_to_ods_change_detection = n.serviceClient< tms_msg_ss::ods_change_detection >("ods_change_detection");
  int id;

  while (1)
  {
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr model(new pcl::PointCloud< pcl::PointXYZRGB >);

    printf("1:table1\n2:table2\n3:small shelf\n4:big shelf\n5:chair\n6:bed\n7:robot_position\n0:finish\n\nid = ");
    scanf("%d", &id);

    if (id == 0)
      break;

    float furniture_x, furniture_y, furniture_z, furniture_theta;
    float robot_x, robot_y, robot_z, robot_theta;

    // printf("furniture position : ");
    // scanf("%f %f %f %f", &furniture_x, &furniture_y, &furniture_z, &furniture_theta);
    srv.request.furniture.position.x = 900;   // furniture_x;
    srv.request.furniture.position.y = 1900;  // furniture_y;
    srv.request.furniture.position.z = 700;   // furniture_z;
    srv.request.furniture.orientation.z = 0;  // furniture_theta;
    // printf("robot position : ");
    // scanf("%f %f %f %f", &robot_x, &robot_y, &robot_z, &robot_theta);
    srv.request.robot.x = 2250;     // robot_x;
    srv.request.robot.y = 1800;     // robot_y;
    srv.request.robot.theta = 180;  // robot_theta;

    switch (id)
    {
      case 1:
        srv.request.furniture_id = 14;
        pcl::io::loadPCDFile("src/ods_trigger/data/changedt_trigger/table.pcd", *cloud);
        pcl::io::loadPCDFile("src/ods_trigger/data/changedt_trigger/model_table1.pcd", *model);
        // srv.request.furniture.x = 1000;
        // srv.request.furniture.y = 3000;
        // srv.request.furniture.z = 700;
        break;

      case 2:
        srv.request.furniture_id = 15;
        pcl::io::loadPCDFile("src/ods_trigger/data/changedt_trigger/table2.pcd", *cloud);
        pcl::io::loadPCDFile("src/ods_trigger/data/changedt_trigger/model_table2.pcd", *model);
        // srv.request.furniture.x = 1500;
        // srv.request.furniture.y = 2500;
        // srv.request.furniture.z = 700;
        break;

      case 3:
        srv.request.furniture_id = 13;
        pcl::io::loadPCDFile("src/ods_trigger/data/changedt_trigger/smallshelf.pcd", *cloud);
        pcl::io::loadPCDFile("src/ods_trigger/data/changedt_trigger/model_smallshelf.pcd", *model);
        // srv.request.furniture.x = 1500;
        // srv.request.furniture.y = 2500;
        // srv.request.furniture.z = 700;
        break;

      case 4:
        srv.request.furniture_id = 12;
        pcl::io::loadPCDFile("src/ods_trigger/data/changedt_trigger/bigshelf.pcd", *cloud);
        pcl::io::loadPCDFile("src/ods_trigger/data/changedt_trigger/model_bigshelf.pcd", *model);
        // srv.request.furniture.x = 4000;
        // srv.request.furniture.y = 940;
        // srv.request.furniture.z = 1800;
        break;

      case 5:
        srv.request.furniture_id = 16;
        pcl::io::loadPCDFile("src/ods_trigger/data/changedt_trigger/chair.pcd", *cloud);
        pcl::io::loadPCDFile("src/ods_trigger/data/changedt_trigger/model_chair.pcd", *model);
        break;

      case 6:
        srv.request.furniture_id = 17;
        pcl::io::loadPCDFile("src/ods_trigger/data/changedt_trigger/bed.pcd", *cloud);
        pcl::io::loadPCDFile("src/ods_trigger/data/changedt_trigger/model_bed.pcd", *model);

        // srv.request.furniture.x = 3200;
        // srv.request.furniture.y = 3500;
        // srv.request.furniture.z = 0;
        // srv.request.furniture.theta = 90.0;
        break;

      case 7:
        srv.request.furniture_id = 20;
        // pcl::io::loadPCDFile ("src/ods_trigger/data/changedt_trigger/table.pcd", *cloud);
        pcl::io::loadPCDFile("src/ods_trigger/data/changedt_trigger/model_table1.pcd", *model);

        // srv.request.furniture.x = 1400;
        // srv.request.furniture.y = 1900;
        // srv.request.furniture.z = 700;
        break;

      default:
        printf("no funitures\n");
        break;
    }

    srv.request.cloud.header = pcl_conversions::fromPCL(cloud->header);
    srv.request.model.header = pcl_conversions::fromPCL(model->header);

    if (commander_to_ods_change_detection.call(srv))
    {
      for (int i = 0; i < srv.response.objects.poses.size(); i++)
      {
        std::cout << srv.response.objects.poses[i].position.x << " " << srv.response.objects.poses[i].position.y << " "
                  << srv.response.objects.poses[i].position.z << " " << std::endl;
      }
    }

    // pcl::io::savePCDFile ("src/ods_trigger/data/changedt_trigger/cloud2.pcd", srv.response.cloud);
    /*cylinder::cylinder srv;
    commander_to_ods_change_detection = n.serviceClient<cylinder::cylinder>("cylinder_detection");

    srv.request.start = 1;
    if(commander_to_ods_change_detection.call(srv)){
        for(int i=0;i<srv.response.x.size();i++){
            std::cout << srv.response.x[i] << " " <<
                         srv.response.y[i] << " " <<
                         srv.response.z[i] << " " <<
                         srv.response.radious[i] << std::endl;
        }
    }*/
  }

  // ros::spin();

  return 0;
}
