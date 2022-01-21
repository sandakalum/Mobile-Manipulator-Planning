#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../../rps.h"

#include <tms_msg_ss/fss_class_data.h>
#include <tms_msg_rp/rps_map_full.h>
#include <tms_msg_db/tmsdb_get_robots_info.h>
#include <tms_msg_db/tmsdb_get_movable_furnitures_info.h>

#include <sstream>

using namespace std;

ros::Publisher rps_map_pub;
ros::ServiceClient commander_to_get_movable_furnitures_info;
ros::ServiceClient commander_to_get_robots_info;

vector< vector< CollisionMapData > > temp_Map, init_Map;
tms_msg_rp::rps_map_full pub_Map;
string msg;

void map_make(const tms_msg_ss::fss_class_data::ConstPtr &msgClass)
{
  cout << "map checking..." << endl;

  int map_x = 0, map_y = 0;

  temp_Map.clear();
  temp_Map = init_Map;

  unsigned int iLength = msgClass->iID.size();
  if (iLength != 0)
  {
    cout << "new cluster appeared!" << endl;
    for (unsigned int i = 0; i < iLength; i++)
    {
      if (msgClass->iID[i] == 1)  // smartPal
        continue;
      else if ((msgClass->iID[i] == 21) || (msgClass->iID[i] == 22))  // wagon
        continue;
      else if (msgClass->fAvgIntrinsicIntensity[i] > 3000.0)  // high intensity obj
        continue;
      else
      {
        unsigned int iLengthx = msgClass->LrfData[i].fX2.size();
        for (unsigned int j = 0; j < iLengthx; j++)
        {
          //~ cout<<"X1:"<<msgClass->LrfData[i].fX1[j]<<"	Y1:"<<msgClass->LrfData[i].fY1[j]<<"
          // X2:"<<msgClass->LrfData[i].fX2[j]<<"	Y2:"<<msgClass->LrfData[i].fY2[j]<<endl;
          convertPos(msgClass->LrfData[i].fX1[j] / 1000.0, msgClass->LrfData[i].fY1[j] / 1000.0, map_x, map_y);
          //~ cout<<"	map_X1:"<<map_x<<"	map_Y1:"<<map_y<<endl;
          temp_Map[map_x][map_y].object = 1;
          temp_Map[map_x][map_y].dist_from_obj = 0.0;
          temp_Map[map_x][map_y].collision = true;
          convertPos(msgClass->LrfData[i].fX2[j] / 1000.0, msgClass->LrfData[i].fY2[j] / 1000.0, map_x, map_y);
          //~ cout<<"	map_X2:"<<map_x<<"	map_Y2:"<<map_y<<endl;
          temp_Map[map_x][map_y].object = 1;
          temp_Map[map_x][map_y].dist_from_obj = 0.0;
          temp_Map[map_x][map_y].collision = true;
        }
      }
    }
  }

  setVoronoiLine(temp_Map, msg);
  calcDistFromObj(temp_Map, msg);
  convertMap(temp_Map, pub_Map);

  for (int k = 0; k < 100; k++)
    rps_map_pub.publish(pub_Map);

  ROS_INFO("MAP publish");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rps_map_checker");

  ros::NodeHandle n;
  ros::Subscriber fss_class_data_subscriber = n.subscribe("fss_class_data", 1, map_make);
  rps_map_pub = n.advertise< tms_msg_rp::rps_map_full >("rps_map_data", 1);

  commander_to_get_movable_furnitures_info =
      n.serviceClient< tms_msg_db::tmsdb_get_movable_furnitures_info >("tmsdb_get_movable_furnitures_info");
  commander_to_get_robots_info = n.serviceClient< tms_msg_db::tmsdb_get_robots_info >("tmsdb_get_robots_info");

  initCollisionMap(init_Map);
  setVoronoiLine(init_Map, msg);
  calcDistFromObj(init_Map, msg);

  ros::spin();

  return 0;
}
