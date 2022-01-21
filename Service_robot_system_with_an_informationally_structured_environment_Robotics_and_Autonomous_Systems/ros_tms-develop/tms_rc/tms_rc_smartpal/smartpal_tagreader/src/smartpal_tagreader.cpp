///-----------------------------------------------------------------------------
/// @FileName smartpal_tagreader.cpp
/// @Date 2013.05.27
/// @author Yoonseok Pyo (passionvirus@gmail.com)
///-----------------------------------------------------------------------------
#include <ros/ros.h>
#include <tms_msg_rc/tag_data.h>

#include "corba_client.h"

//------------------------------------------------------------------------------
Client *smartpal = new Client;

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //--------------------------------------------------------------------------
  ros::init(argc, argv, "smartpal_tagreader");
  ros::NodeHandle nh;

  //--------------------------------------------------------------------------
  while (!smartpal->Initialize())
  {
    printf("CORBA client object initialization has failed.\n");
    printf("Retry initializing...\n");

    sleep(3);
  }

  printf("CORBA client object initialization has been completed.\n\n");

  //--------------------------------------------------------------------------
  smartpal->TagInit();           // initialize of tag-reader
  smartpal->TagSetConfig(1, 3);  // set time-slots = 16
  smartpal->TagSetPower(2);      // Max Power

  ros::Publisher pub_fmtagdata = nh.advertise< tms_msg_rc::tag_data >("spfm_tag_data", 1000);
  ros::Publisher pub_bmtagdata = nh.advertise< tms_msg_rc::tag_data >("spbm_tag_data", 1000);
  ros::Publisher pub_lhtagdata = nh.advertise< tms_msg_rc::tag_data >("splh_tag_data", 1000);
  ros::Publisher pub_rhtagdata = nh.advertise< tms_msg_rc::tag_data >("sprh_tag_data", 1000);

  vector< uint8_t > vstData;

  ros::Rate loop_rate(10);

  unsigned char tagdatasize[4] = {0};
  unsigned char tagdata[4][128] = {0};

  //--------------------------------------------------------------------------
  while (ros::ok())
  {
    tms_msg_rc::tag_data fmTagdata;
    tms_msg_rc::tag_data bmTagdata;
    tms_msg_rc::tag_data lhTagdata;
    tms_msg_rc::tag_data rhTagdata;

    smartpal->TagInventory(tagdatasize, tagdata);

    ros::Time tNow = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9;

    //----------------------------------------------------------------------
    fmTagdata.header.frame_id = "spfm_tag_data";
    fmTagdata.header.stamp = tNow;
    fmTagdata.tMeasuredTime = tNow;

    fmTagdata.uiObjectNum = tagdatasize[0] / 8;
    fmTagdata.uiTagDataSize = tagdatasize[0];

    for (int i = 0; i < tagdatasize[0]; i++)
    {
      fmTagdata.ucTagData.push_back((unsigned char)tagdata[0][i]);
    }

    //----------------------------------------------------------------------
    bmTagdata.header.frame_id = "spbm_tag_data";
    bmTagdata.header.stamp = tNow;
    bmTagdata.tMeasuredTime = tNow;

    bmTagdata.uiObjectNum = tagdatasize[1] / 8;
    bmTagdata.uiTagDataSize = tagdatasize[1];

    for (int i = 0; i < tagdatasize[1]; i++)
    {
      bmTagdata.ucTagData.push_back((unsigned char)tagdata[1][i]);
    }

    //----------------------------------------------------------------------
    lhTagdata.header.frame_id = "splh_tag_data";
    lhTagdata.header.stamp = tNow;
    lhTagdata.tMeasuredTime = tNow;

    lhTagdata.uiObjectNum = tagdatasize[2] / 8;
    lhTagdata.uiTagDataSize = tagdatasize[2];

    for (int i = 0; i < tagdatasize[2]; i++)
    {
      lhTagdata.ucTagData.push_back((unsigned char)tagdata[2][i]);
    }

    //----------------------------------------------------------------------
    rhTagdata.header.frame_id = "sprh_tag_data";
    rhTagdata.header.stamp = tNow;
    rhTagdata.tMeasuredTime = tNow;

    rhTagdata.uiObjectNum = tagdatasize[3] / 8;
    rhTagdata.uiTagDataSize = tagdatasize[3];

    for (int i = 0; i < tagdatasize[3]; i++)
    {
      rhTagdata.ucTagData.push_back((unsigned char)tagdata[3][i]);
    }
    //----------------------------------------------------------------------
    pub_fmtagdata.publish(fmTagdata);
    pub_bmtagdata.publish(bmTagdata);
    pub_lhtagdata.publish(lhTagdata);
    pub_rhtagdata.publish(rhTagdata);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return (0);
  //--------------------------------------------------------------------------
}
//------------------------------------------------------------------------------
