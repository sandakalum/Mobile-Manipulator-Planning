#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

using namespace std;

tf2_ros::Buffer tfBuffer_;
tf2_ros::TransformListener *tfListener_;
tf2_ros::TransformBroadcaster *tfBroadcaster;
geometry_msgs::TransformStamped mapOdomTransMsg, odomBaseTransMsg;
tf2::Transform baseOdomTrans;
bool is_got_trans = false;
bool is_2d_mode;

/**
 * @brief PFで得られた姿勢と"map"->"base_footprint"が一致するよう，"map"->"odom"を逆算して発行．
 * @param input
 * PFで得られた姿勢.frame_idは"map"であることが前提．姿勢が３次元の場合はz=0平面にプロジェクションするかを選択
 */
// void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &arg) {
void pose_callback(const nav_msgs::Odometry::ConstPtr &arg) {
  // ROS_INFO("pose_callback");

  tf2::Transform sframePoseTrans, mapPoseTrans, baseOdomTrans;
  tf2::fromMsg(arg->pose.pose, sframePoseTrans);
  try {
    // "map"フレームでの姿勢を取得
    tf2::Transform mapSframeTrans;
    tf2::fromMsg(tfBuffer_.lookupTransform("map", arg->header.frame_id,
                                           /*ros::Time::now()*/ arg->header.stamp,
                                           ros::Duration(3.0)).transform,
                 mapSframeTrans);
    mapPoseTrans = mapSframeTrans * sframePoseTrans;
    if (is_2d_mode) {
      // 高さを0に
      mapPoseTrans.getOrigin().setZ(0);
      // ロール，ピッチを０に
      tf2::Quaternion new_quat;
      new_quat.setRPY(0, 0, tf2::getYaw(mapPoseTrans.getRotation()));
      mapPoseTrans.setRotation(new_quat);
    }

    ROS_DEBUG_STREAM("mapPoseTrans x:" << mapPoseTrans.getOrigin().getX()
                                      << " y:" << mapPoseTrans.getOrigin().getY()
                                      << " z:" << mapPoseTrans.getOrigin().getZ() << " yaw:"
                                      << tf2::getYaw(mapPoseTrans.getRotation()) * 180.0 / 3.1415);
    // "base_footprint" -> "odom"
    odomBaseTransMsg = tfBuffer_.lookupTransform(
        "odom", "base_footprint", /*ros::Time::now()*/ arg->header.stamp, ros::Duration(3.0));
    tf2::fromMsg(odomBaseTransMsg.transform, baseOdomTrans);
    baseOdomTrans = baseOdomTrans.inverse();
    // tf2::fromMsg(
    //     tfBuffer_.lookupTransform("base_footprint", "odom", /*ros::Time::now()*/
    //     arg->header.stamp,
    //                               ros::Duration(3.0)).transform,
    //     baseOdomTrans);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Could NOT transform : %s", ex.what());
    return;
  }
  // "map"->"odom"の取るべき位置関係を算出
  tf2::Transform mapOdomTrans = mapPoseTrans * baseOdomTrans;
  mapOdomTransMsg.transform = tf2::toMsg(mapOdomTrans);
  //! @todo: 時間が遅れすぎるので現在時刻を利用している．
  string odomFrame = "odom";
  if (is_2d_mode) {
    // 2dモードの場合は座標系を複製
    odomFrame = "odom_2d";
    odomBaseTransMsg.header.frame_id = odomFrame;
    odomBaseTransMsg.child_frame_id = "base_footprint_2d";
  }
  // mapOdomTransMsg.header.stamp = ros::Time::now(); /*arg->header.stamp*/
  mapOdomTransMsg.header.frame_id = "map";
  mapOdomTransMsg.child_frame_id = odomFrame;

  is_got_trans = true;
}

/**
 * @brief 10Hz周期で"map"->"odom"を発行．
 * @param input 実行時間の遅れとか．不要．
 */
void timer_callback(const ros::TimerEvent &event) {
  if (!is_got_trans) {
    return;
  }
  //"map" -> "odom"のtfを発行
  mapOdomTransMsg.header.stamp = ros::Time::now();
  tfBroadcaster->sendTransform(mapOdomTransMsg);
  if (is_2d_mode) {
    odomBaseTransMsg.header.stamp = ros::Time::now();
    tfBroadcaster->sendTransform(odomBaseTransMsg);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose2frame", ros::init_options::AnonymousName);

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // param
  private_nh.param<bool>("mode_2d", is_2d_mode, false);
  private_nh.deleteParam("mode_2d");

  // tf
  tf2_ros::TransformListener tmp_listener(tfBuffer_);
  tfListener_ = &tmp_listener;
  tfBroadcaster = new tf2_ros::TransformBroadcaster();

  // Subscribers
  ros::Subscriber pose_sub = nh.subscribe("pose", 10, pose_callback);
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), timer_callback);

  ros::spin();
  delete tfBroadcaster;
  return 0;
}
