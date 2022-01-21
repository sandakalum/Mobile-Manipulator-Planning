//------------------------------------------------------------------------------
// @file   : umo_tracker.cpp
// @brief  : Tracker of Unidentified Moving Object (UMO)
// @author : Yoonseok Pyo
// @version: Ver0.0.1 (since 2014.11.21)
// @date   : 2014.11.21
//------------------------------------------------------------------------------
#include <ros/ros.h>
#include <tms_msg_ss/tracking_points.h>
#include <tms_msg_db/TmsdbGetData.h>

//------------------------------------------------------------------------------
class UmoTracker
{
private:
  ros::NodeHandle nh_;
  ros::ServiceClient get_data_client_;
  ros::Subscriber all_tracker_sub_;
  ros::Publisher umo_tracker_pub_;

  bool dbQueryKnownObject(uint32_t id, uint32_t sensor, double *pos_x, double *pos_y)
  {
    tms_msg_db::TmsdbGetData known_object;

    known_object.request.tmsdb.id = id;
    known_object.request.tmsdb.sensor = sensor;

    if (!get_data_client_.call(known_object))
    {
      *pos_x = *pos_y = 0.0;
      return false;
    }
    else if (known_object.response.tmsdb.empty() == true)
    {
      *pos_x = *pos_y = 0.0;
      return false;
    }

    if (known_object.response.tmsdb[0].state == 1)
    {
      *pos_x = known_object.response.tmsdb[0].x / 1000;
      *pos_y = known_object.response.tmsdb[0].y / 1000;
    }

    return true;
  }

  void msgCallback(const tms_msg_ss::tracking_points::ConstPtr &msg)
  {
    unsigned int object_num = msg->tracking_grid.size();
    ROS_INFO("object num: %d", object_num);

    tms_msg_ss::tracking_points umo_tracker_points;
    uint32_t known_object_num = 6;
    double umo_pos_x = 0.0, umo_pos_y = 0.0;
    double known_object_x[known_object_num], known_object_y[known_object_num], distance_known_object_and_umo;

    dbQueryKnownObject(1001, 3001, &known_object_x[0], &known_object_y[0]);  // person, Vicon
    dbQueryKnownObject(2002, 3001, &known_object_x[1], &known_object_y[1]);  // sp5_1, Vicon
    dbQueryKnownObject(2003, 3001, &known_object_x[2], &known_object_y[2]);  // sp5_2, Vicon
    dbQueryKnownObject(2007, 3001, &known_object_x[3], &known_object_y[3]);  // wheelchair, Vicon
    dbQueryKnownObject(2011, 3001, &known_object_x[4], &known_object_y[4]);  // kxp2, Vicon
    dbQueryKnownObject(6019, 3001, &known_object_x[5], &known_object_y[5]);  // wheelchair, Vicon

    for (uint32_t i = 0; i < object_num; i++)
    {
      bool isMatching = false;

      umo_pos_x = msg->tracking_grid[i].x / 1000;
      umo_pos_y = msg->tracking_grid[i].y / 1000;

      for (uint32_t j = 0; j < known_object_num; j++)
      {
        distance_known_object_and_umo = sqrt((umo_pos_x - known_object_x[j]) * (umo_pos_x - known_object_x[j]) +
                                             (umo_pos_y - known_object_y[j]) * (umo_pos_y - known_object_y[j]));

        if (distance_known_object_and_umo < 1.0)  // 1 Meter
        {
          isMatching = true;
        }
      }

      if (isMatching == false)
      {
        umo_tracker_points.tracking_grid.push_back(msg->tracking_grid[i]);
      }
    }
    umo_tracker_pub_.publish(umo_tracker_points);
  }

public:
  UmoTracker()
  {
    all_tracker_sub_ = nh_.subscribe("tracking_points", 10, &UmoTracker::msgCallback, this);
    get_data_client_ = nh_.serviceClient< tms_msg_db::TmsdbGetData >("/tms_db_reader/dbreader");
    umo_tracker_pub_ = nh_.advertise< tms_msg_ss::tracking_points >("umo_tracking_points", 10);
  }
  ~UmoTracker()
  {
  }
};

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "unidentified_moving_object_tracker");
  UmoTracker ut;
  ros::spin();
  return 0;
}

//------------------------------------------------------------------------------
// EOF
