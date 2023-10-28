#ifndef _ODOM_PUB_H_
#define _ODOM_PUB_H_

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"

namespace OdomSubPubNS {

class OdomSubPub {
public:
  void init(ros::NodeHandle &nh);

private:
  ros::Subscriber odom_sub, imu_sub;
  ros::Publisher odom_pub;
  nav_msgs::Odometry odom;
  sensor_msgs::Imu imu;
  void odomCb(const geometry_msgs::PoseStampedConstPtr &);
  void imuCb(const sensor_msgs::ImuConstPtr &);
};

} // namespace OdomSubPubNS

#endif