
#ifndef POZYX_PANEL_H
#define POZYX_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <QHBoxLayout>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <geometry_msgs/PoseStamped.h>
#include <boost/algorithm/string.hpp>
#endif

namespace tms_ss_ninebot_pozyx
{
  class PozyxPanel: public rviz::Panel
  {
    Q_OBJECT

  public:
    PozyxPanel( QWidget* parent = 0 );

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::string topic_name = "/pozyx";

    static const int num_item = 7;
    QHBoxLayout* layout_ = new QHBoxLayout;
    QTableWidget* data_table_ = new QTableWidget(num_item, 1); // rows, columns
    QTableWidgetItem *item[num_item]; // Table Item
    void processItem(const geometry_msgs::PoseStamped::ConstPtr& msg);

  };
}

#endif // POZYX_PANEL_H
