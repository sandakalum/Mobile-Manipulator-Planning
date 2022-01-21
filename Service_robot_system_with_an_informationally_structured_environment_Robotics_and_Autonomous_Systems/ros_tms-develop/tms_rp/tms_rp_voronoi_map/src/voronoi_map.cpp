//------------------------------------------------------------------------------
// @file   : tms_rp_voronoi_map.cpp
// @brief  : voronoi map for robot's path planning
// @author : Yoonseok Pyo, Kouhei Nakashima
// @version: Ver0.0.3 (since 2015.08.10)
// @date   : 2015.09.03
//------------------------------------------------------------------------------
// include for ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>

#include <tms_msg_db/TmsdbGetData.h>
#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_db/Tmsdb.h>

#include <tms_msg_ss/tracking_points.h>

#include <tms_msg_rp/rps_map_full.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/date_time/posix_time/posix_time.hpp>

// include for std
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>

namespace tms_rp
{
using namespace std;
using Eigen::Vector3d;
using Eigen::Matrix3d;
typedef Eigen::Vector3d Vector3;
typedef Eigen::Matrix3d Matrix3;

class CollisionMapData
{
public:
  bool object_;
  double dist_from_obj_;
  bool collision_;
  bool voronoi_;
  int thinning_flg_;
  double dist_from_voronoi_;
  bool path_;
  double dist_from_path_;
  double dist_from_goal_;
  bool person_view_;
  double table_height_;
};

class TmsRpVoronoiMap
{
private:
  // ROS NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // ROS Timer
  ros::Timer static_map_update_timer;
  ros::Timer dynamic_map_update_timer;
  // ROS Topic Subscriber
  ros::Publisher static_map_pub_;
  ros::Publisher dynamic_map_pub_;
  vector< vector< CollisionMapData > > collision_map_;
  tms_msg_rp::rps_map_full static_map_;
  tms_msg_rp::rps_map_full dynamic_map_;
  string result_msg_;
  ros::ServiceClient get_data_client_;
  ros::Publisher nonvoronoi_map_marker_pub, static_map_marker_pub, dynamic_marker_pub;
  // ROS Parameters:
  double update_time;
  bool is_debug;

public:
  TmsRpVoronoiMap()
    : nh_priv("~")
    , update_time(1)
    ,  // sec
    is_debug(false)
  {
    // Init parameter
    nh_priv.param("update_time", update_time, update_time);
    nh_priv.param("is_debug", is_debug, is_debug);
    ROS_ASSERT(initTmsRpVoronoiMap());
    // Subscriber for tms_db_data topic
    get_data_client_ = nh.serviceClient< tms_msg_db::TmsdbGetData >("/tms_db_reader");
    static_map_pub_ = nh.advertise< tms_msg_rp::rps_map_full >("rps_map_data", 1);
    dynamic_map_pub_ = nh.advertise< tms_msg_rp::rps_map_full >("rps_dynamic_map", 1);
    nonvoronoi_map_marker_pub = nh.advertise< visualization_msgs::Marker >("nonvoronoi_map_marker", 1);
    static_map_marker_pub = nh.advertise< visualization_msgs::Marker >("voronoi_map_marker", 1);
    dynamic_marker_pub = nh.advertise< visualization_msgs::Marker >("dynamic_map_marker", 1);
    // TimerEvent
    static_map_update_timer = nh.createTimer(ros::Duration(update_time), &TmsRpVoronoiMap::staticMapPublish, this);
    dynamic_map_update_timer = nh.createTimer(ros::Duration(update_time), &TmsRpVoronoiMap::dynamicMapPublish, this);

    initCollisionMap(collision_map_);
    setVoronoiLine(collision_map_, result_msg_);
    calcDistFromObj(collision_map_, result_msg_);
    convertMap(collision_map_, static_map_);
  }

  ~TmsRpVoronoiMap()
  {
    ROS_ASSERT(shutdownTmsRpVoronoiMap());
  }

private:
  double x_llimit_, x_ulimit_, y_llimit_, y_ulimit_, cell_size_;  // Meter
  bool initTmsRpVoronoiMap()
  {
    ROS_INFO("tms_rp_voronoi_map : Init OK!\n");
    return true;
  }

  bool shutdownTmsRpVoronoiMap()
  {
    return true;
  }

  bool initCollisionMap(vector< vector< CollisionMapData > >& map)
  {
    FILE* fp;
    string file_name;
    char home_dir[255];
    const char* fname;

    //      strcpy(home_dir, getenv("HOME"));
    //      file_name = home_dir;
    //      file_name += "/catkin_ws/src/ros_tms/tms_rp/tms_rp_action/map/use_collision_map.csv";
    file_name = ros::package::getPath("tms_rp_voronoi_map") + "/map/use_collision_map.csv";
    ROS_INFO(file_name.c_str());
    fname = file_name.c_str();

    while (1)
    {
      if ((fp = fopen(fname, "r")) == NULL)
      {
        cout << "Error: collision map cannot open" << endl;
        return false;
      }
      else
        break;
    }

    map.clear();
    vector< CollisionMapData > tempMapLine;
    CollisionMapData tempMapData;

    char* tp, buff[4096];

    if (fgets(buff, 4096, fp) == NULL)
      return false;

    tp = strtok(buff, ",");
    x_llimit_ = atof(tp);
    tp = strtok(NULL, ",");
    x_ulimit_ = atof(tp);
    tp = strtok(NULL, ",");
    y_llimit_ = atof(tp);
    tp = strtok(NULL, ",");
    y_ulimit_ = atof(tp);
    tp = strtok(NULL, ",");
    cell_size_ = atof(tp);

    int CommaCount = 0;

    while (fgets(buff, 4096, fp) != NULL)  // collision
    {
      CommaCount = 0;
      tp = strtok(buff, ",");
      tempMapData.object_ = atof(tp);

      if (tempMapData.object_)
      {
        tempMapData.dist_from_obj_ = 0.0;
        tempMapData.collision_ = true;
      }
      else
      {
        tempMapData.dist_from_obj_ = (x_ulimit_ - x_llimit_) * (y_ulimit_ - y_llimit_);
        tempMapData.collision_ = false;
      }

      tempMapData.voronoi_ = false;
      tempMapData.path_ = false;
      tempMapData.thinning_flg_ = 0;
      tempMapData.dist_from_voronoi_ = (x_ulimit_ - x_llimit_) * (y_ulimit_ - y_llimit_);
      tempMapData.dist_from_goal_ = (x_ulimit_ - x_llimit_) * (y_ulimit_ - y_llimit_);
      tempMapData.dist_from_path_ = (x_ulimit_ - x_llimit_) * (y_ulimit_ - y_llimit_);
      tempMapLine.push_back(tempMapData);

      CommaCount++;

      while (tp != NULL)
      {
        if (CommaCount == int(round((y_ulimit_ - y_llimit_) / cell_size_)) + 1)
          break;

        tp = strtok(NULL, ",");
        CommaCount++;

        if (tp != NULL)
        {
          tempMapData.object_ = atof(tp);
          if (tempMapData.object_)
          {
            tempMapData.dist_from_obj_ = 0.0;
            tempMapData.collision_ = true;
          }
          else
          {
            tempMapData.dist_from_obj_ = (x_ulimit_ - x_llimit_) * (y_ulimit_ - y_llimit_);
            tempMapData.collision_ = false;
          }

          tempMapData.voronoi_ = false;
          tempMapData.path_ = false;
          tempMapData.thinning_flg_ = 0;
          tempMapData.dist_from_voronoi_ = (x_ulimit_ - x_llimit_) * (y_ulimit_ - y_llimit_);
          tempMapData.dist_from_goal_ = (x_ulimit_ - x_llimit_) * (y_ulimit_ - y_llimit_);
          tempMapData.dist_from_path_ = (x_ulimit_ - x_llimit_) * (y_ulimit_ - y_llimit_);
          tempMapLine.push_back(tempMapData);
        }
      }

      map.push_back(tempMapLine);
      tempMapLine.clear();
    }

    fclose(fp);

    return true;
  }

  bool setVoronoiLine(vector< vector< CollisionMapData > >& map, string& message)
  {
    if (map.empty())
    {
      message = "Error : Map is empty";
      cout << message << endl;
      return false;
    }

    for (unsigned int x = 0; x < map.size(); x++)
    {
      for (unsigned int y = 0; y < map[x].size(); y++)
      {
        map[x][y].voronoi_ = false;
        if (!map[x][y].collision_)
          map[x][y].thinning_flg_ = 1;
        else
          map[x][y].thinning_flg_ = 0;
      }
    }

    bool flg = true;

    while (flg)
    {
      flg = false;
      for (int k = 0; k < 4; k++)
      {
        for (int x = 1; x < map.size() - 1; x++)
        {
          for (int y = 1; y < map[x].size() - 1; y++)
          {
            int i, j;

            switch (k)
            {
              case 0:
                i = x + 1;
                j = y;
                break;
              case 1:
                i = x;
                j = y - 1;
                break;
              case 2:
                i = x - 1;
                j = y;
                break;
              case 3:
                i = x;
                j = y + 1;
                break;
            }

            if ((map[x][y].thinning_flg_ == 1) && (map[i][j].thinning_flg_ == 0))
            {
              if (((map[x][y - 1].thinning_flg_ == 0) && (map[x + 1][y].thinning_flg_ == 1) &&
                   (map[x][y + 1].thinning_flg_ == 1) && (map[x + 1][y + 1].thinning_flg_ == 0)) ||
                  ((map[x - 1][y - 1].thinning_flg_ == 0) && (map[x][y - 1].thinning_flg_ == 1) &&
                   (map[x - 1][y].thinning_flg_ == 1) && (map[x + 1][y].thinning_flg_ == 0)) ||
                  ((map[x - 1][y - 1].thinning_flg_ == 0) && (map[x][y - 1].thinning_flg_ == 1) &&
                   (map[x - 1][y].thinning_flg_ == 1) && (map[x][y + 1].thinning_flg_ == 0)) ||
                  ((map[x - 1][y].thinning_flg_ == 0) && (map[x + 1][y].thinning_flg_ == 1) &&
                   (map[x][y + 1].thinning_flg_ == 1) && (map[x + 1][y + 1].thinning_flg_ == 0)) ||
                  ((map[x - 1][y].thinning_flg_ == 0) && (map[x + 1][y].thinning_flg_ == 0) &&
                   (map[x][y + 1].thinning_flg_ == 1)) ||
                  ((map[x][y - 1].thinning_flg_ == 0) && (map[x - 1][y].thinning_flg_ == 1) &&
                   (map[x][y + 1].thinning_flg_ == 0)) ||
                  ((map[x][y - 1].thinning_flg_ == 1) && (map[x - 1][y].thinning_flg_ == 0) &&
                   (map[x + 1][y].thinning_flg_ == 0)) ||
                  ((map[x][y - 1].thinning_flg_ == 0) && (map[x + 1][y].thinning_flg_ == 1) &&
                   (map[x][y + 1].thinning_flg_ == 0)) ||
                  ((map[x - 1][y].thinning_flg_ == 0) && (map[x - 1][y + 1].thinning_flg_ == 1) &&
                   (map[x][y + 1].thinning_flg_ == 0)) ||
                  ((map[x - 1][y - 1].thinning_flg_ == 1) && (map[x][y - 1].thinning_flg_ == 0) &&
                   (map[x - 1][y].thinning_flg_ == 0)) ||
                  ((map[x][y - 1].thinning_flg_ == 0) && (map[x + 1][y - 1].thinning_flg_ == 1) &&
                   (map[x + 1][y].thinning_flg_ == 0)) ||
                  ((map[x + 1][y].thinning_flg_ == 0) && (map[x][y + 1].thinning_flg_ == 0) &&
                   (map[x + 1][y + 1].thinning_flg_ == 1)) ||
                  ((map[x - 1][y - 1].thinning_flg_ == 0) && (map[x][y - 1].thinning_flg_ == 1) &&
                   (map[x + 1][y - 1].thinning_flg_ == 0) && (map[x - 1][y].thinning_flg_ == 1) &&
                   (map[x + 1][y].thinning_flg_ == 1) && (map[x - 1][y + 1].thinning_flg_ == 0) &&
                   (map[x + 1][y + 1].thinning_flg_ == 0)) ||
                  ((map[x - 1][y - 1].thinning_flg_ == 0) && (map[x][y - 1].thinning_flg_ == 1) &&
                   (map[x + 1][y - 1].thinning_flg_ == 0) && (map[x + 1][y].thinning_flg_ == 1) &&
                   (map[x - 1][y + 1].thinning_flg_ == 0) && (map[x][y + 1].thinning_flg_ == 1) &&
                   (map[x + 1][y + 1].thinning_flg_ == 0)) ||
                  ((map[x - 1][y - 1].thinning_flg_ == 0) && (map[x + 1][y - 1].thinning_flg_ == 0) &&
                   (map[x - 1][y].thinning_flg_ == 1) && (map[x + 1][y].thinning_flg_ == 1) &&
                   (map[x - 1][y + 1].thinning_flg_ == 0) && (map[x][y + 1].thinning_flg_ == 1) &&
                   (map[x + 1][y + 1].thinning_flg_ == 0)) ||
                  ((map[x - 1][y - 1].thinning_flg_ == 0) && (map[x][y - 1].thinning_flg_ == 1) &&
                   (map[x + 1][y - 1].thinning_flg_ == 0) && (map[x - 1][y].thinning_flg_ == 1) &&
                   (map[x - 1][y + 1].thinning_flg_ == 0) && (map[x][y + 1].thinning_flg_ == 1) &&
                   (map[x + 1][y + 1].thinning_flg_ == 0)) ||
                  ((map[x - 1][y - 1].thinning_flg_ == 0) && (map[x][y - 1].thinning_flg_ == 0) &&
                   (map[x + 1][y - 1].thinning_flg_ == 0) && (map[x - 1][y].thinning_flg_ == 0) &&
                   (map[x - 1][y + 1].thinning_flg_ == 0)) ||
                  ((map[x - 1][y - 1].thinning_flg_ == 0) && (map[x][y - 1].thinning_flg_ == 0) &&
                   (map[x + 1][y - 1].thinning_flg_ == 0) && (map[x + 1][y].thinning_flg_ == 0) &&
                   (map[x + 1][y + 1].thinning_flg_ == 0)) ||
                  ((map[x - 1][y - 1].thinning_flg_ == 0) && (map[x - 1][y].thinning_flg_ == 0) &&
                   (map[x - 1][y + 1].thinning_flg_ == 0) && (map[x][y + 1].thinning_flg_ == 0) &&
                   (map[x + 1][y + 1].thinning_flg_ == 0)) ||
                  ((map[x + 1][y - 1].thinning_flg_ == 0) && (map[x + 1][y].thinning_flg_ == 0) &&
                   (map[x - 1][y + 1].thinning_flg_ == 0) && (map[x][y + 1].thinning_flg_ == 0) &&
                   (map[x + 1][y + 1].thinning_flg_ == 0)))
                map[x][y].thinning_flg_ = 1;
              else
              {
                flg = true;
                map[x][y].thinning_flg_ = 3;
              }
            }
          }
        }
        for (int x = 1; x < map.size() - 1; x++)
        {
          for (int y = 1; y < map[x].size() - 1; y++)
          {
            if (map[x][y].thinning_flg_ == 3)
              map[x][y].thinning_flg_ = 0;
          }
        }
      }
    }

    for (int x = 0; x < map.size(); x++)
    {
      for (int y = 0; y < map[x].size(); y++)
      {
        if (map[x][y].thinning_flg_ == 1)
        {
          map[x][y].voronoi_ = 1;
          map[x][y].dist_from_voronoi_ = 0.0;
        }
        else
        {
          map[x][y].voronoi_ = 0;
        }
      }
    }
    return true;
  }

  bool calcDistFromObj(vector< vector< CollisionMapData > >& map, string& message)
  {
    if (map.empty())
    {
      message = "Error : Map is empty";
      cout << message << endl;
      return false;
    }

    for (int x = 0; x < map.size(); x++)
    {
      for (int y = 0; y < map[x].size(); y++)
      {
        if (map[x][y].object_)
          map[x][y].dist_from_obj_ = 0.0;
        else
          map[x][y].dist_from_obj_ = (x_ulimit_ - x_llimit_) * (y_ulimit_ - y_llimit_);
      }
    }

    for (int x = 1; x < map.size() - 1; x++)
    {
      for (int y = 1; y < map[x].size() - 1; y++)
      {
        if (!map[x][y].object_)
        {
          map[x][y].dist_from_obj_ = min(min(min(map[x - 1][y - 1].dist_from_obj_ + sqrt(2.0) * cell_size_,
                                                 map[x][y - 1].dist_from_obj_ + 1.0 * cell_size_),
                                             min(map[x - 1][y + 1].dist_from_obj_ + sqrt(2.0) * cell_size_,
                                                 map[x - 1][y].dist_from_obj_ + 1.0 * cell_size_)),
                                         map[x][y].dist_from_obj_);
        }
      }
    }

    for (int x = map.size() - 2; x > 0; x--)
    {
      for (int y = map[x].size() - 2; y > 0; y--)
      {
        if (!map[x][y].object_)
        {
          map[x][y].dist_from_obj_ = min(min(min(map[x + 1][y].dist_from_obj_ + 1.0 * cell_size_,
                                                 map[x + 1][y - 1].dist_from_obj_ + sqrt(2.0) * cell_size_),
                                             min(map[x][y + 1].dist_from_obj_ + 1.0 * cell_size_,
                                                 map[x + 1][y + 1].dist_from_obj_ + sqrt(2.0) * cell_size_)),
                                         map[x][y].dist_from_obj_);
        }
      }
    }

    return true;
  }

  void convertMap(vector< vector< CollisionMapData > > map, tms_msg_rp::rps_map_full& pp_map)
  {
    pp_map.rps_map_x.clear();

    pp_map.x_llimit = x_llimit_;
    pp_map.x_ulimit = x_ulimit_;
    pp_map.y_llimit = y_llimit_;
    pp_map.y_ulimit = y_ulimit_;
    pp_map.cell_size = cell_size_;

    tms_msg_rp::rps_map_data temp_map_d;
    tms_msg_rp::rps_map_y temp_map_y;

    for (unsigned int x = 0; x < map.size(); x++)
    {
      temp_map_y.rps_map_y.clear();
      for (unsigned int y = 0; y < map[x].size(); y++)
      {
        temp_map_d.object = map[x][y].object_;
        temp_map_d.voronoi = map[x][y].voronoi_;
        temp_map_d.dist_from_obj_f = map[x][y].dist_from_obj_;

        temp_map_y.rps_map_y.push_back(temp_map_d);
      }
      pp_map.rps_map_x.push_back(temp_map_y);
    }
  }

  void staticMapPublish(const ros::TimerEvent& e)
  {
    static_map_pub_.publish(static_map_);

    uint32_t shape = visualization_msgs::Marker::POINTS;

    visualization_msgs::Marker voronoi_marker;
    voronoi_marker.header.frame_id = "world_link";
    voronoi_marker.header.stamp = ros::Time::now();
    voronoi_marker.ns = "voronoi_map";
    voronoi_marker.id = 0;
    voronoi_marker.type = shape;
    voronoi_marker.action = visualization_msgs::Marker::ADD;

    voronoi_marker.pose.orientation.x = 0.0;
    voronoi_marker.pose.orientation.y = 0.0;
    voronoi_marker.pose.orientation.z = 0.0;
    voronoi_marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    voronoi_marker.scale.x = 0.1;
    voronoi_marker.scale.y = 0.1;
    voronoi_marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    voronoi_marker.color.r = 0.0f;
    voronoi_marker.color.g = 1.0f;
    voronoi_marker.color.b = 0.0f;
    voronoi_marker.color.a = 1.0;

    visualization_msgs::Marker nonvoronoi_marker;
    nonvoronoi_marker.header.frame_id = "world_link";
    nonvoronoi_marker.header.stamp = ros::Time::now();
    nonvoronoi_marker.ns = "nonvoronoi_map";
    nonvoronoi_marker.id = 0;
    nonvoronoi_marker.type = shape;
    nonvoronoi_marker.action = visualization_msgs::Marker::ADD;

    nonvoronoi_marker.pose.orientation.x = 0.0;
    nonvoronoi_marker.pose.orientation.y = 0.0;
    nonvoronoi_marker.pose.orientation.z = 0.0;
    nonvoronoi_marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    nonvoronoi_marker.scale.x = 0.1;
    nonvoronoi_marker.scale.y = 0.1;
    nonvoronoi_marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    nonvoronoi_marker.color.r = 0.65;
    nonvoronoi_marker.color.g = 0.65;
    nonvoronoi_marker.color.b = 0.65;
    nonvoronoi_marker.color.a = 1.0;

    geometry_msgs::Point p;

    for (unsigned int x = 0; x < static_map_.rps_map_x.size(); x++)
    {
      for (unsigned int y = 0; y < static_map_.rps_map_x[x].rps_map_y.size(); y++)
      {
        if (static_map_.rps_map_x[x].rps_map_y[y].voronoi)
        {
          // p.x = x*0.1 - 0.3;
          // p.y = y*0.1 - 0.5;
          p.x = x * 0.1;
          p.y = y * 0.1;
          p.z = 0.01;
          voronoi_marker.points.push_back(p);
        }

        if (!static_map_.rps_map_x[x].rps_map_y[y].object)
        {
          p.x = x * 0.1;
          p.y = y * 0.1;
          p.z = 0.005;
          nonvoronoi_marker.points.push_back(p);
        }
      }
    }
    static_map_marker_pub.publish(voronoi_marker);
    nonvoronoi_map_marker_pub.publish(nonvoronoi_marker);
  }

  void dynamicMapPublish(const ros::TimerEvent& e)
  {
    int map_x = 0, map_y = 0;
    vector< vector< CollisionMapData > > temp_Map;
    string result_msg;

    temp_Map.clear();
    temp_Map = collision_map_;

    Vector3 object_pos = Vector3(0, 0, 0);
    Matrix3 object_ori;
    Vector3 object_rpy;
    //    BodyItemPtr item;
    //    TmsRpController trc;

    //    // add current position of obstacle (wagon)
    //    item = trc.objTag2Item()["wagon"];
    //    if(!item){
    //      ROS_INFO("Error: The tagId is not recorded.");
    //    }
    //    object_pos = item->body()->link(0)->p();
    //    object_ori = item->body()->link(0)->R();
    //    item->calcForwardKinematics();
    //    item->notifyKinematicStateChange();

    //    object_rpy = grasp::rpyFromRot(object_ori);

    //    map_x = (int)round( ( object_pos(0) - x_llimit_ ) / cell_size_ );
    //    map_y = (int)round( ( object_pos(1) - y_llimit_ ) / cell_size_ );

    //    for(int check_x_size=map_x-2;check_x_size<=map_x+2;check_x_size++)
    //    {
    //      for(int check_y_size=map_y-2;check_y_size<=map_y+2;check_y_size++)
    //      {
    //        if (check_x_size < 0 || check_y_size < 0 || check_x_size > 80 || check_y_size > 45) continue;
    //        temp_Map[check_x_size][check_y_size].object_        = 1;
    //        temp_Map[check_x_size][check_y_size].dist_from_obj_ = 0.0;
    //        temp_Map[check_x_size][check_y_size].collision_     = true;
    //      }
    //    }

    setVoronoiLine(temp_Map, result_msg);
    calcDistFromObj(temp_Map, result_msg);
    convertMap(temp_Map, dynamic_map_);

    dynamic_map_pub_.publish(dynamic_map_);

    uint32_t shape = visualization_msgs::Marker::POINTS;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "dynamic_map";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    geometry_msgs::Point p;

    for (unsigned int x = 0; x < dynamic_map_.rps_map_x.size(); x++)
    {
      for (unsigned int y = 0; y < dynamic_map_.rps_map_x[x].rps_map_y.size(); y++)
      {
        if (dynamic_map_.rps_map_x[x].rps_map_y[y].voronoi)
        {
          marker.id = 0;
          // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
          p.x = x * 0.1;
          p.y = y * 0.1;
          p.z = 0.01;
          marker.points.push_back(p);
        }
      }
    }

    dynamic_marker_pub.publish(marker);
  }

  void dynamicMapPublish(tms_msg_ss::tracking_points unknown_moving_object_pos)
  {
    int map_x = 0, map_y = 0;
    int obstacle_pos_x = 0, obstacle_pos_y = 0;
    vector< vector< CollisionMapData > > temp_Map;
    string result_msg;

    temp_Map.clear();
    temp_Map = collision_map_;

    Vector3 object_pos = Vector3(0, 0, 0);
    Matrix3 object_ori;
    Vector3 object_rpy;
    //    BodyItemPtr item;
    //    TmsRpController trc;

    //    // add current position of obstacle (wagon)
    //    item = trc.objTag2Item()["wagon"];
    //    if(!item){
    //      ROS_INFO("Error: The tagId is not recorded.");
    //    }
    //    object_pos = item->body()->link(0)->p();
    //    object_ori = item->body()->link(0)->R();
    //    item->calcForwardKinematics();
    //    item->notifyKinematicStateChange();

    //    object_rpy = grasp::rpyFromRot(object_ori);

    //    map_x = (int)round( ( object_pos(0) - x_llimit_ ) / cell_size_ );
    //    map_y = (int)round( ( object_pos(1) - y_llimit_ ) / cell_size_ );

    //    for(int check_x_size=map_x-2;check_x_size<=map_x+2;check_x_size++)
    //    {
    //      for(int check_y_size=map_y-2;check_y_size<=map_y+2;check_y_size++)
    //      {
    //        if (check_x_size < 0 || check_y_size < 0 || check_x_size > 80 || check_y_size > 45) continue;
    //        temp_Map[check_x_size][check_y_size].object_        = 1;
    //        temp_Map[check_x_size][check_y_size].dist_from_obj_ = 0.0;
    //        temp_Map[check_x_size][check_y_size].collision_     = true;
    //      }
    //    }

    //    // add current position of unknown moving object (umo)
    //    for(unsigned int i=0;i<unknown_moving_object_pos.tracking_grid.size();i++)
    //    {

    //      obstacle_pos_x = unknown_moving_object_pos.tracking_grid[i].x/1000;
    //      obstacle_pos_y = unknown_moving_object_pos.tracking_grid[i].y/1000;

    //      map_x = (int)round( ( obstacle_pos_x - x_llimit_ ) / cell_size_ );
    //      map_y = (int)round( ( obstacle_pos_y - y_llimit_ ) / cell_size_ );

    //      for(int check_x_size=map_x-2;check_x_size<=map_x+2;check_x_size++)
    //      {
    //        for(int check_y_size=map_y-2;check_y_size<=map_y+2;check_y_size++)
    //        {
    //        if (check_x_size < 0 || check_y_size < 0 || check_x_size > 80 || check_y_size > 45) continue;
    //          temp_Map[check_x_size][check_y_size].object_        = 1;
    //          temp_Map[check_x_size][check_y_size].dist_from_obj_ = 0.0;
    //          temp_Map[check_x_size][check_y_size].collision_     = true;
    //        }
    //      }
    //    }

    setVoronoiLine(temp_Map, result_msg);
    calcDistFromObj(temp_Map, result_msg);
    convertMap(temp_Map, dynamic_map_);

    dynamic_map_pub_.publish(dynamic_map_);
  }
};
}  // namespace tms_rp

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "tms_rp_voronoi_map");
  tms_rp::TmsRpVoronoiMap vm;
  ros::spin();
  return 0;
}
// EOF
