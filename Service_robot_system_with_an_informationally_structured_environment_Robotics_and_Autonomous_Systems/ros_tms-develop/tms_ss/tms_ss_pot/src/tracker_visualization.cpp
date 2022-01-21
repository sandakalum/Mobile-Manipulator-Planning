
#include "ros/ros.h"
#include "stdio.h"
#include "tms_msg_ss/tracking_points.h"
#include "tms_msg_ss/tracking_grid.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "iostream"
#include "math.h"

#define GRAB_POINT 3
#define WINDOW_TIME 1.0

ros::Publisher pub;
ros::Subscriber sub;

// tracking_object point
struct tracker_point
{
  double x;
  double y;
};

// tracker_parameter
struct tracker_param
{
  int id;
  int flag;
  int count;
  std::vector< tracker_point > xy;
  std::vector< double > point_time;
  double start_time;
};

tracker_param tmp_tracker_info;
std::vector< tracker_param > tracker_info_array;
std::vector< tracker_param >::iterator p;
int object_id_max = 0;

// colorset
float colorset[14][4] = {{1, 0, 0, 0},
                         {0, 1, 0, 0},
                         {0, 0, 1, 0},
                         {1, 1, 0, 0},
                         {0, 1, 1, 0},
                         {1, 0, 1, 0},
                         {1, 1, 1, 0},
                         {0.5, 0, 0, 0},
                         {0, 0.5, 0, 0},
                         {0, 0, 0.5, 0},
                         {0.5, 0.5, 0, 0},
                         {0, 0.5, 0.5, 0},
                         {0.5, 0, 0.5, 0},
                         {0.5, 0.5, 0.5, 0}};

void visualization_callback(const tms_msg_ss::tracking_points::ConstPtr &msg)
{
  // human point visualization
  uint32_t shape_circle = visualization_msgs::Marker::CYLINDER;
  visualization_msgs::MarkerArray markerArray;
  /*
    for(int i=0;i < msg->tracking_grid.size();i++)
    {
      visualization_msgs::Marker marker_circle;
      marker_circle.header.frame_id = "/laser";
      marker_circle.header.stamp = ros::Time::now();

      marker_circle.id = msg->tracking_grid[i].id;
      marker_circle.type = shape_circle;
      marker_circle.action = visualization_msgs::Marker::ADD;

      marker_circle.pose.position.x = msg->tracking_grid[i].x;
      marker_circle.pose.position.y = msg->tracking_grid[i].y;
      marker_circle.pose.position.z = 0.5;
      marker_circle.pose.orientation.x = 0.0;
      marker_circle.pose.orientation.y = 0.0;
      marker_circle.pose.orientation.z = 0.0;
      marker_circle.pose.orientation.w = 1.0;

      marker_circle.scale.x = 0.1;
      marker_circle.scale.y = 0.1;
      marker_circle.scale.z = 1.0;

      int color = msg->tracking_grid[i].id % 14;
      marker_circle.color.r = colorset[color][0];
      marker_circle.color.g = colorset[color][1];
      marker_circle.color.b = colorset[color][2];
      marker_circle.color.a = 1.0;

      marker_circle.lifetime = ros::Duration(0.5);
      markerArray.markers.push_back(marker_circle);
     }
  */

  // uint32_t shape_arrow = visualization_msgs::Marker::ARROW;
  uint32_t shape_arrow = visualization_msgs::Marker::SPHERE;
  uint32_t shape_line = visualization_msgs::Marker::LINE_STRIP;
  uint32_t shape_mesh = visualization_msgs::Marker::MESH_RESOURCE;

  // gradualline
  // flag
  for (p = tracker_info_array.begin(); p != tracker_info_array.end(); ++p)
  {
    p->flag = 0;  // all flag -> 0
  }

  int z_id = 0;  // object id
  bool object_key = 0;

  for (int i = 0; i < msg->tracking_grid.size(); i++)
  {
    object_key = true;
    if (tracker_info_array.size() > 0)
    {
      for (p = tracker_info_array.begin(); p != tracker_info_array.end(); ++p)
      {
        if (p->id == msg->tracking_grid[i].id)
        {
          p->count++;

          // candidate on the array
          if (p->count % GRAB_POINT == 0)
          {
            tracker_point tmp_xy;
            tmp_xy.x = msg->tracking_grid[i].x;
            tmp_xy.y = msg->tracking_grid[i].y;
            p->xy.push_back(tmp_xy);
            p->point_time.push_back(ros::Time::now().toSec());
          }
          p->flag = 1;  // flag ON
          std::vector< tracker_point >::iterator xy_iterator;
          xy_iterator = (p->xy).begin();

          std::vector< double >::iterator time_iterator;
          for (time_iterator = p->point_time.begin(); time_iterator < p->point_time.end();
               ++time_iterator, ++xy_iterator)
          {
            if (ros::Time::now().toSec() - *time_iterator > WINDOW_TIME)
            {
              p->point_time.erase(time_iterator);
              p->xy.erase(xy_iterator);
              break;
            }
          }
          (p->xy).resize((p->xy).size());
          (p->point_time).resize((p->point_time).size());

          // line
          visualization_msgs::Marker marker_line;
          marker_line.header.frame_id = "/world_link";
          marker_line.header.stamp = ros::Time::now();
        Z_REFRESH1:
          for (int j = 0; j < msg->tracking_grid.size(); j++)
          {
            if (z_id == msg->tracking_grid[j].id)
            {
              z_id++;
              goto Z_REFRESH1;
            }
          }
          marker_line.id = z_id;
          marker_line.type = shape_line;
          marker_line.action = visualization_msgs::Marker::ADD;
          marker_line.scale.x = 0.03;
          marker_line.points.resize(0);
          int color_l = p->id % 14;
          marker_line.color.r = colorset[color_l][0];
          marker_line.color.g = colorset[color_l][1];
          marker_line.color.b = colorset[color_l][2];
          marker_line.color.a = 1.0;
          marker_line.lifetime = ros::Duration(0.5);

          // track history
          int z;
          for (z = 0; z < (p->xy).size(); z++)
          {
            geometry_msgs::Point ps;
            ps.x = (p->xy)[z].x;
            ps.y = (p->xy)[z].y;
            ps.z = 1.0;

            marker_line.points.push_back(ps);
            // std::cout << "(" << (p->xy)[z].x << "," << (p->xy)[z].y << ")";

            visualization_msgs::Marker marker;

            marker.header.frame_id = "/world_link";
            marker.header.stamp = ros::Time::now();

          Z_REFRESH:
            for (int j = 0; j < msg->tracking_grid.size(); j++)
            {
              if (z_id == msg->tracking_grid[j].id)
              {
                z_id++;
                goto Z_REFRESH;
              }
            }
            marker.id = z_id;
            marker.type = shape_arrow;

            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = (p->xy)[z].x;
            marker.pose.position.y = (p->xy)[z].y;
            marker.pose.position.z = 1.0;

            // marker.pose.orientation.x = (p->xy)[z].x - (p->xy)[z+1].x;
            // marker.pose.orientation.y = (p->xy)[z].y - (p->xy)[z+1].y;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 0.0;

            // marker.scale.x = sqrt(pow(((p->xy)[z].x - (p->xy)[z+1].x), 2.0) + pow(((p->xy)[z].y - (p->xy)[z+1].y),
            // 2.0));
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            int color = p->id % 14;
            marker.color.r = colorset[color][0];
            marker.color.g = colorset[color][1];
            marker.color.b = colorset[color][2];
            marker.color.a = 0.4;
            marker.lifetime = ros::Duration(0.001);

            //markerArray.markers.push_back(marker);
            z_id++;

            if (object_id_max < z_id)
            {
              object_id_max = z_id;
            }
            if (z == (p->xy).size() - 1)
            {
              // circle
              visualization_msgs::Marker marker_circle;
              marker_circle.header.frame_id = "/world_link";
              marker_circle.header.stamp = ros::Time::now();
            Z_REFRESH2:
              for (int j = 0; j < msg->tracking_grid.size(); j++)
              {
                if (z_id == msg->tracking_grid[j].id)
                {
                  z_id++;
                  goto Z_REFRESH2;
                }
              }
              marker_circle.id = msg->tracking_grid[i].id;
//              marker_circle.type = shape_circle;
              marker_circle.type = shape_mesh;

              marker_circle.action = visualization_msgs::Marker::ADD;

              if(marker_circle.type == shape_mesh){
                marker_circle.mesh_resource = "package://tms_ss_pot/meshes/WalkingMan4.dae";
                marker_circle.mesh_use_embedded_materials = true;
                marker_circle.scale.x = 0.025;
                marker_circle.scale.y = 0.025;
                marker_circle.scale.z = 0.025;
                marker_circle.pose.position.z = 0.0;
              } else {
                int color_c = p->id % 14;
                marker_circle.color.r = colorset[color_c][0];
                marker_circle.color.g = colorset[color_c][1];
                marker_circle.color.b = colorset[color_c][2];
                marker_circle.color.a = 1.0;
                marker_circle.scale.x = 0.102;
                marker_circle.scale.y = 0.102;
                marker_circle.scale.z = 1.0;
                marker_circle.pose.position.z = 1.0;
              }

              marker_circle.pose.position.x = (p->xy)[z].x;
              marker_circle.pose.position.y = (p->xy)[z].y;
              marker_circle.pose.orientation.x = 0.0;
              marker_circle.pose.orientation.y = 0.0;
              marker_circle.pose.orientation.z = 0.0;
              marker_circle.pose.orientation.w = 1.0;

              marker_circle.lifetime = ros::Duration(0.5);

              markerArray.markers.push_back(marker_circle);
            }
          }
          markerArray.markers.push_back(marker_line);
          std::cout << "object_id_max " << z_id << std::endl;
          break;
        }
        else if (p == tracker_info_array.end() - 1)  // no candidate on the array
        {
          // candidate not on the array
          tmp_tracker_info.id = msg->tracking_grid[i].id;
          tmp_tracker_info.flag = 1;
          tmp_tracker_info.count = 0;
          tracker_point tmp_xy;
          tmp_xy.x = msg->tracking_grid[i].x;
          tmp_xy.y = msg->tracking_grid[i].y;
          tmp_tracker_info.xy.resize(0);
          tmp_tracker_info.xy.clear();
          tmp_tracker_info.xy.push_back(tmp_xy);
          tmp_tracker_info.start_time = ros::Time::now().toSec();
          tmp_tracker_info.point_time.resize(0);
          tmp_tracker_info.point_time.clear();
          tmp_tracker_info.point_time.push_back(ros::Time::now().toSec());
          tracker_info_array.insert(tracker_info_array.begin(), tmp_tracker_info);
          break;
        }
      }
    }
    else
    {
      // no array
      tmp_tracker_info.id = msg->tracking_grid[i].id;
      tmp_tracker_info.flag = 1;
      tmp_tracker_info.count = 0;
      tracker_point tmp_xy;
      tmp_xy.x = msg->tracking_grid[i].x;
      tmp_xy.y = msg->tracking_grid[i].y;
      tmp_tracker_info.xy.resize(0);
      tmp_tracker_info.xy.clear();
      tmp_tracker_info.xy.push_back(tmp_xy);
      tmp_tracker_info.start_time = ros::Time::now().toSec();
      tmp_tracker_info.point_time.resize(0);
      tmp_tracker_info.point_time.clear();
      tmp_tracker_info.point_time.push_back(ros::Time::now().toSec());
      tracker_info_array.push_back(tmp_tracker_info);
    }
  }

  // flag 0 function
  tracker_info_array.resize(tracker_info_array.size());
L:
  int n_flag = 0;
  for (p = tracker_info_array.begin(); p != tracker_info_array.end(); ++p)
  {
    if (p->flag == 0)
    {
      // p-> xy.resize(0);
      p->xy.clear();
      p->point_time.resize(0);
      p->xy.resize(0);
      tracker_info_array.erase(p);
      n_flag = 1;
      break;
    }
    pub.publish(markerArray);
  }
  if (n_flag == 1)
    goto L;

  // tracking_gradual delete
  pub.publish(markerArray);
}

// main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "portable_visualization");
  ros::NodeHandle n;
  pub = n.advertise< visualization_msgs::MarkerArray >("visualization_marker_array", 1);
  sub = n.subscribe("/tracking_points", 1000, visualization_callback);
  ros::spin();
  return 0;
}
