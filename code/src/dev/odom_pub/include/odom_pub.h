#ifndef _ODOM_PUB_H_
#define _ODOM_PUB_H_

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "ros/forwards.h"
#include "ros/timer.h"
#include "sensor_msgs/Imu.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <ros/ros.h>

namespace OdomSubPubNS {

class OdomSubPub {
public:
  OdomSubPub();
  void init(ros::NodeHandle &nh);

private:
  ros::Subscriber odom_sub, imu_sub;
  ros::Publisher odom_nav_pub, cam_pose_pub;
  nav_msgs::Odometry odom_nav;
  sensor_msgs::Imu imu;

  tf2_ros::TransformBroadcaster br_world_to_base;
  geometry_msgs::TransformStamped world_to_base;

  ros::Timer tf_timer;
  tf2_ros::Buffer tf_buff;
  tf2_ros::TransformListener listener;
  geometry_msgs::TransformStamped cam_tf_stamp;
  geometry_msgs::PoseStamped cam_pose;

  void odomCb(const geometry_msgs::PoseStampedConstPtr &);
  void imuCb(const sensor_msgs::ImuConstPtr &);
  void tf_timerCb(const ros::TimerEvent &);
};

} // namespace OdomSubPubNS

#endif