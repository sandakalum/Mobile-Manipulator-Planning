
#include <stdio.h>
#include <QString>
#include <QStringList>
#include <QAbstractItemView>
#include <QHeaderView>
#include <vector>

#include "pozyx_panel.h"

namespace tms_ss_ninebot_pozyx
{
  PozyxPanel::PozyxPanel( QWidget* parent ) : rviz::Panel( parent )
  {
    // Table
    data_table_->setEditTriggers(QAbstractItemView::NoEditTriggers); // Prohibit Editting
    data_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch); // Resize Column
    layout_->addWidget( data_table_ );
    setLayout( layout_ );

    // Labels
    QStringList H_Label, V_Labels;
    H_Label << "Pozyx Data";
    V_Labels << "Coor.X" << "Coor.Y" << "Coor.Z" << "Quat.X" << "Quat.Y" << "Quat.Z" << "Quat.W";
    data_table_->setHorizontalHeaderLabels(H_Label);
    data_table_->setVerticalHeaderLabels(V_Labels);

    sub_ = nh_.subscribe(topic_name, 10, &PozyxPanel::processItem, this);

  }

  void PozyxPanel::processItem(const geometry_msgs::PoseStamped::ConstPtr& msg){

    std::vector<QString> data;
    data.resize(num_item);

    data[0] = QString(QString::number(msg->pose.position.x));
    data[1] = QString(QString::number(msg->pose.position.y));
    data[2] = QString(QString::number(msg->pose.position.z));
    data[3] = QString(QString::number(msg->pose.orientation.x));
    data[4] = QString(QString::number(msg->pose.orientation.y));
    data[5] = QString(QString::number(msg->pose.orientation.z));
    data[6] = QString(QString::number(msg->pose.orientation.w));

    for(int i = 0; i < num_item; i++){
      item[i] = new QTableWidgetItem( data[i] );
      item[i]->setTextAlignment( Qt::AlignCenter );  // Centering Item
      data_table_->setItem( i, 0, item[i]);
    }

  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( tms_ss_ninebot_pozyx::PozyxPanel, rviz::Panel )
