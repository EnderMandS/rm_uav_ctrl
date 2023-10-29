#include "odom_pub.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_puh");
  ros::NodeHandle nh;
  OdomSubPubNS::OdomSubPub odom_obj;
  odom_obj.init(nh);
  ros::spin();
  return 0;
}

namespace OdomSubPubNS {

void OdomSubPub::init(ros::NodeHandle &nh) {
  odom_sub = nh.subscribe("/airsim_node/drone_1/pose_gt", 10,
                          &OdomSubPub::odomCb, this);
  imu_sub =
      nh.subscribe("airsim_node/drone_1/imu/imu", 1, &OdomSubPub::imuCb, this);

  odom_nav_pub = nh.advertise<nav_msgs::Odometry>("/odom_nav", 100);
  odom_nav.header.frame_id = "odom";
  odom_nav.child_frame_id = "base_link";

  world_to_base.header.frame_id = "world";
  world_to_base.child_frame_id = "base_link";

  ROS_INFO("OdomSubPub init success");
}

void OdomSubPub::odomCb(const geometry_msgs::PoseStampedConstPtr &msg) {
  world_to_base.transform.translation.x = odom_nav.pose.pose.position.x =
      msg->pose.position.x;
  world_to_base.transform.translation.y = odom_nav.pose.pose.position.y =
      msg->pose.position.y;
  world_to_base.transform.translation.z = odom_nav.pose.pose.position.z =
      msg->pose.position.z;
  world_to_base.transform.rotation.w = odom_nav.pose.pose.orientation.w =
      msg->pose.orientation.w;
  world_to_base.transform.rotation.x = odom_nav.pose.pose.orientation.x =
      msg->pose.orientation.x;
  world_to_base.transform.rotation.y = odom_nav.pose.pose.orientation.y =
      msg->pose.orientation.y;
  world_to_base.transform.rotation.z = odom_nav.pose.pose.orientation.z =
      msg->pose.orientation.z;
  world_to_base.header.stamp = odom_nav.header.stamp = ros::Time::now();
  odom_nav_pub.publish(odom_nav);
  br_world_to_base.sendTransform(world_to_base);
}

void OdomSubPub::imuCb(const sensor_msgs::ImuConstPtr &msg) {
  odom_nav.twist.twist.angular.x = msg->angular_velocity.x;
  odom_nav.twist.twist.angular.y = msg->angular_velocity.y;
  odom_nav.twist.twist.angular.z = msg->angular_velocity.z;
  odom_nav.twist.twist.linear.x = msg->linear_acceleration.x;
  odom_nav.twist.twist.linear.y = msg->linear_acceleration.y;
  odom_nav.twist.twist.linear.z = msg->linear_acceleration.z;
  // odom_nav.twist.covariance = msg->angular_velocity_covariance
}

} // namespace OdomSubPubNS
