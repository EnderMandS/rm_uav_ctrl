#ifndef _ODOM_PUB_H_
#define _ODOM_PUB_H_

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <ros/ros.h>

namespace OdomSubPubNS {

class OdomSubPub {
public:
  void init(ros::NodeHandle &nh);

private:
  ros::Subscriber odom_sub, imu_sub;
  ros::Publisher odom_nav_pub;

  tf2_ros::TransformBroadcaster br_world_to_base;
  geometry_msgs::TransformStamped world_to_base;

  nav_msgs::Odometry odom_nav;
  sensor_msgs::Imu imu;

  void odomCb(const geometry_msgs::PoseStampedConstPtr &);
  void imuCb(const sensor_msgs::ImuConstPtr &);
};

} // namespace OdomSubPubNS

#endif