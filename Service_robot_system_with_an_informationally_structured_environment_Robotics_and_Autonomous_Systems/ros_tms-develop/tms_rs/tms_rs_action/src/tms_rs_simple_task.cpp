//------------------------------------------------------------------------------
// @file   : tms_rs_simple_task.cpp
// @brief  : user request for simple robot tasks
// @author : Alaoui Hassani Atlas Omar
// @version: Ver1.0.0 (since 2014.06.16)
// @date   : 2014.06.30
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <ros/ros.h>
#include <unistd.h>
#include <tms_msg_rs/robot_task.h>
#include <tms_msg_rs/rs_task.h>
#include <tms_msg_rp/rp_cmd.h>
#include <tms_msg_db/TmsdbGetData.h>

#include <sstream>
#include <string>

//------------------------------------------------------------------------------
#define deg2rad(x) ((x)*M_PI / 180.0)

//------------------------------------------------------------------------------
using namespace std;

//------------------------------------------------------------------------------
float x, y, theta;
int go_to = 0;

// Position of glasses
float gPosX;
float gPosY;
float gY;

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "smartpal_simple_task");
  ros::NodeHandle nh;
  ros::Publisher send_task = nh.advertise< tms_msg_rs::robot_task >("rs_simple_task", 10);

  ros::ServiceClient move_robot = nh.serviceClient< tms_msg_rp::rp_cmd >("rp_cmd");
  ros::ServiceClient get_data_client = nh.serviceClient< tms_msg_db::TmsdbGetData >("/tms_db_reader/dbreader");

  cout << "Enter a task and press Enter\n\n";

  while (ros::ok())
  {
    tms_msg_rs::robot_task tempdata;

    // 2002 -> smartpal 5
    tempdata.id = 2002;

    // Get task from keybord
    getline(cin, tempdata.task);

    if (tempdata.task == "incline")
    {
      send_task.publish(tempdata);
      // wait for the state update
      ros::Duration(1).sleep();
      tempdata.task = "";
      // return to the original position
      send_task.publish(tempdata);
    }
    else if (tempdata.task == "shake hand")
    {
      send_task.publish(tempdata);
      // wait for the state update
      ros::Duration(0.05).sleep();
      // return to the original position
      tempdata.task = "";
      send_task.publish(tempdata);
    }
    else if (tempdata.task == "go to")
    {
      string robot;
      bool choice = false;
      tms_msg_rp::rp_cmd rpCMD;

      do
      {
        cout << "Press \"r\" for real robot or \"v\" for virtual robot : ";
        cin >> robot;
        if (robot == "r" || robot == "v")
        {
          choice = true;
        }
      } while (choice = false);

      cout << ">> X : ";
      cin >> x;
      cout << ">> Y : ";
      cin >> y;
      cout << ">> Theta : ";
      cin >> theta;
      cout << "Robot destination : [" << x << " ," << y << " ," << theta << "] \n";
      cout << "Please wait...\n";
      send_task.publish(tempdata);

      if (robot == "r")
      {
        // ATTENTION : Position of real robot initialized in oculus_robot_move.cpp !!
        rpCMD.request.command = 9001;
        rpCMD.request.robot_id = 2002;
        rpCMD.request.type = true;  // real robot

        rpCMD.request.arg.resize(4);

        rpCMD.request.arg[0] = -1;
        rpCMD.request.arg[1] = x;
        rpCMD.request.arg[2] = y;
        rpCMD.request.arg[3] = theta;

        if (!(move_robot.call(rpCMD)))
        {
          cout << "[rpCMD] Failed to call service rp_cmd" << endl;
        }
      }
      else if (robot == "v")
      {
        rpCMD.request.command = 9001;
        rpCMD.request.robot_id = 2002;
        rpCMD.request.type = false;  // virtual robot

        rpCMD.request.arg.resize(4);

        rpCMD.request.arg[0] = -1;
        rpCMD.request.arg[1] = x;
        rpCMD.request.arg[2] = y;
        rpCMD.request.arg[3] = theta;

        if (!(move_robot.call(rpCMD)))
        {
          cout << "[rpCMD] Failed to call service rp_cmd" << endl;
        }
      }

      tempdata.task = "";
      send_task.publish(tempdata);
      go_to = 1;
    }
    else if (tempdata.task == "find person")
    {
      string isSafe;
      bool choice = false;

      tms_msg_db::TmsdbGetData getGlassesData;
      // find glasses
      getGlassesData.request.tmsdb.id = 1001;
      getGlassesData.request.tmsdb.sensor = 3001;

      if (!(get_data_client.call(getGlassesData)))
      {
        cout << "[TmsAction] Failed to call service getRGalssesData ID: " << getGlassesData.request.tmsdb.id << endl;
      }
      else if (getGlassesData.response.tmsdb.empty() == true)
      {
        cout << "[TmsAction] nothing on floor (ID=" << getGlassesData.request.tmsdb.id << ")" << endl;
      }

      if (getGlassesData.response.tmsdb[0].state == 1)
      {
        gPosX = getGlassesData.response.tmsdb[0].x;
        gPosY = getGlassesData.response.tmsdb[0].y;
        gY = getGlassesData.response.tmsdb[0].ry;

        tms_msg_rp::rp_cmd rpCMD;

        // in front of the glasses, 1m far
        x = 1000 * cos(deg2rad(gY)) + gPosX;
        y = 1000 * sin(deg2rad(gY)) + gPosY;
        theta = 180 + gY;

        // Virtual Robot
        rpCMD.request.command = 9001;
        rpCMD.request.robot_id = 2002;
        rpCMD.request.type = false;

        rpCMD.request.arg.resize(4);

        rpCMD.request.arg[0] = -1;
        rpCMD.request.arg[1] = x;
        rpCMD.request.arg[2] = y;
        rpCMD.request.arg[3] = theta;

        cout << "Please wait...\n";

        if (!(move_robot.call(rpCMD)))
        {
          cout << "[rpCMD] Failed to call service rp_cmd" << endl;
        }

        // Safety control
        do
        {
          cout << "Is it safe? The real robot can do the same? (ok/no) : ";
          cin >> isSafe;
          if ((isSafe == "ok") || (isSafe == "no"))
          {
            choice = true;
          }
        } while (choice == false);

        if (isSafe == "ok")
        {
          rpCMD.request.command = 9001;
          rpCMD.request.robot_id = 2002;
          rpCMD.request.type = true;  // real robot

          rpCMD.request.arg.resize(4);

          rpCMD.request.arg[0] = -1;
          rpCMD.request.arg[1] = x;
          rpCMD.request.arg[2] = y;
          rpCMD.request.arg[3] = theta;

          cout << "Please wait...\n";

          if (!(move_robot.call(rpCMD)))
          {
            cout << "[rpCMD] Failed to call service rp_cmd" << endl;
          }
        }

        tempdata.task = "";
        send_task.publish(tempdata);
        go_to = 1;
      }
      else
      {
        printf("Glasses not found, maybe Vicon Tracker isn't open...");
      }
    }
    else if (go_to == 1)
    {
      cout << ">>>Job finished, now you can ask for a new task !\n";
      go_to = 0;
    }
    else
    {
      ROS_INFO("Wrong Task...");
    }
  }

  ros::spin();
  return (0);
}
//------------------------------------------------------------------------------
// EOF